/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Wireless endpoint perf-test mode (CONFIG_AVB_ENDPOINT_PERF_TEST_MODE).
 *
 * Replaces the production bring-up. Brings up only NVS + esp_event +
 * WiFi STA, associates to the bridge SoftAP, then registers an
 * esp_wifi_internal_reg_rxcb hook that counts AVTP frames matching the
 * configured dest MAC + stream ID.
 *
 * RX hot path is six compares + four 64-bit increments + a free —
 * everything else (logging, rate computation) lives in a 1 Hz stats
 * task that reads plain volatile counters.
 */

#include "avb_endpoint_perf_test.h"
#include "sdkconfig.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "perf_rx";

/* esp_wifi internal RX hook + buffer-free symbol. Same forward-decl
 * pattern as esp_avb/avbnet.c so this file doesn't take a hard
 * esp_private include. */
extern esp_err_t esp_wifi_internal_reg_rxcb(
    int wifi_if, esp_err_t (*fn)(void *buffer, uint16_t len, void *eb));
extern esp_err_t esp_wifi_internal_free_rx_buffer(void *eb);
#ifdef CONFIG_AVB_ENDPOINT_PERF_TEST_TX_MODE
/* Same forward-decl pattern for the TX direction. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);
extern esp_err_t esp_wifi_internal_set_fix_rate(int ifx, bool en, int rate);
#endif

#define PERF_AP_SSID "ESP-AVB-Bridge"

#define ETH_HDR_LEN  14
#define VLAN_TAG_LEN  4
#define AVTP_HDR_LEN 24

/* Per-frame physical-layer overhead — same accounting as the bridge
 * side so the two endpoints' numbers are directly comparable.
 * preamble + SFD (8) + CRC (4) + IFG (12) = 24 B. */
#define PHY_FRAMING_BYTES 24

/* Hot-path counters. One writer (rx callback in the WiFi driver task),
 * one reader (perf_stats_task at 1 Hz). 64-bit reads are torn-tolerant
 * in this use because the stats task reports deltas over a 1 s window. */
static volatile uint64_t s_rx_pkts = 0;
static volatile uint64_t s_rx_bytes_l2 = 0;
static volatile uint64_t s_rx_bytes_wire = 0;
static volatile uint64_t s_rx_seq_gaps = 0;
static volatile uint64_t s_rx_filtered_dest = 0;  /* dest MAC mismatch */
static volatile uint64_t s_rx_filtered_stream = 0;/* stream ID mismatch */
static volatile uint8_t  s_rx_last_seq = 0;
static volatile bool     s_rx_seq_valid = false;
static volatile int64_t  s_rx_first_us = 0;
static volatile int64_t  s_rx_last_us = 0;

/* Match values populated from Kconfig at startup. Read-only after init,
 * so safe to read concurrently from the RX callback. */
static uint8_t s_match_dest_mac[6];
static uint8_t s_match_stream_id[8];

static EventGroupHandle_t s_wifi_events;
#define BIT_STA_CONNECTED BIT0

static int parse_mac(const char *s, uint8_t out[6]) {
  unsigned a, b, c, d, e, f;
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &a, &b, &c, &d, &e, &f) != 6) {
    return -1;
  }
  out[0] = a; out[1] = b; out[2] = c;
  out[3] = d; out[4] = e; out[5] = f;
  return 0;
}

static int parse_stream_id(const char *s, uint8_t out[8]) {
  unsigned v[8];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x:%x:%x",
             &v[0], &v[1], &v[2], &v[3],
             &v[4], &v[5], &v[6], &v[7]) == 8) {
    for (int i = 0; i < 8; i++) out[i] = (uint8_t)v[i];
    return 0;
  }
  unsigned long long u = 0;
  if (sscanf(s, "%llx", &u) == 1) {
    for (int i = 0; i < 8; i++) {
      out[i] = (uint8_t)((u >> (56 - 8 * i)) & 0xff);
    }
    return 0;
  }
  return -1;
}

/* Inline RX hook fired by the WiFi driver task for every L2 frame on
 * WIFI_IF_STA. We must call esp_wifi_internal_free_rx_buffer(eb) before
 * returning — the eb owns the buffer. No logging, no allocation. */
static esp_err_t perf_rx_cb(void *buffer, uint16_t len, void *eb) {
  if (len < ETH_HDR_LEN + VLAN_TAG_LEN + AVTP_HDR_LEN) {
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
  }
  const uint8_t *f = (const uint8_t *)buffer;

  /* Dest MAC + VLAN ethertype + inner AVTP ethertype check.
   * Bytes 12..13 = 0x8100, bytes 16..17 = 0x22f0. */
  if (memcmp(f, s_match_dest_mac, 6) != 0) {
    s_rx_filtered_dest++;
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
  }
  if (f[12] != 0x81 || f[13] != 0x00 || f[16] != 0x22 || f[17] != 0xf0) {
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
  }

  /* AVTP stream_id at bytes 18+4 .. 18+11. */
  const uint8_t *avtp = f + ETH_HDR_LEN + VLAN_TAG_LEN;
  if (memcmp(avtp + 4, s_match_stream_id, 8) != 0) {
    s_rx_filtered_stream++;
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
  }

  /* Sequence gap tracking. avtp[2] is the per-stream sequence number. */
  uint8_t seq = avtp[2];
  if (s_rx_seq_valid && seq != (uint8_t)(s_rx_last_seq + 1)) {
    s_rx_seq_gaps++;
  }
  s_rx_last_seq = seq;
  s_rx_seq_valid = true;

  int64_t now = esp_timer_get_time();
  if (s_rx_pkts == 0) s_rx_first_us = now;
  s_rx_last_us = now;
  s_rx_pkts++;
  s_rx_bytes_l2 += (uint64_t)len;
  s_rx_bytes_wire += (uint64_t)(len + PHY_FRAMING_BYTES);

  esp_wifi_internal_free_rx_buffer(eb);
  return ESP_OK;
}

#ifdef CONFIG_AVB_ENDPOINT_PERF_TEST_TX_MODE
/* ---- Uplink (STA -> AP) transmit role -------------------------------
 * Mirror of the bridge's perf_tx_task so uplink and downlink are measured
 * over the same esp_wifi_internal_tx path with the same frame shape. The
 * only differences are the interface (WIFI_IF_STA) and that the source MAC
 * is this STA's own address. */
static volatile uint64_t s_tx_pkts = 0;
static volatile uint64_t s_tx_bytes_l2 = 0;
static volatile uint64_t s_tx_fail = 0;
static volatile uint64_t s_tx_busy = 0;
static volatile bool     s_tx_done = false;
static volatile int64_t  s_tx_start_us = 0, s_tx_end_us = 0;

#define PERF_TX_MAX_PAYLOAD 1400

static void build_frame(uint8_t *frame, int *frame_len,
                        const uint8_t src_mac[6]) {
  int payload_bytes = CONFIG_AVB_ENDPOINT_PERF_TEST_TX_PAYLOAD_BYTES;
  if (payload_bytes > PERF_TX_MAX_PAYLOAD) payload_bytes = PERF_TX_MAX_PAYLOAD;
  int total = ETH_HDR_LEN + VLAN_TAG_LEN + AVTP_HDR_LEN + payload_bytes;
  *frame_len = total;
  memset(frame, 0, total);

  memcpy(frame + 0, s_match_dest_mac, 6);
  memcpy(frame + 6, src_mac, 6);
  frame[12] = 0x81; frame[13] = 0x00;
  uint16_t tci = (3u << 13) | (2u & 0x0fffu); /* PCP=3, VID=2 */
  frame[14] = (uint8_t)(tci >> 8);
  frame[15] = (uint8_t)(tci & 0xff);
  frame[16] = 0x22; frame[17] = 0xf0;

  uint8_t *avtp = frame + ETH_HDR_LEN + VLAN_TAG_LEN;
  avtp[0] = 0x02;            /* subtype AAF */
  avtp[1] = 0x81;            /* sv=1, tv=1 */
  avtp[2] = 0;               /* sequence number, bumped per packet */
  avtp[3] = 0x00;
  memcpy(avtp + 4, s_match_stream_id, 8);
  avtp[16] = 0x04;
  avtp[17] = (4u << 4);
  avtp[18] = 8;
  avtp[19] = 24;
  avtp[20] = (uint8_t)((payload_bytes >> 8) & 0xff);
  avtp[21] = (uint8_t)(payload_bytes & 0xff);
}

static void perf_tx_task(void *arg) {
  (void)arg;
  static uint8_t frame[ETH_HDR_LEN + VLAN_TAG_LEN + AVTP_HDR_LEN +
                       PERF_TX_MAX_PAYLOAD];
  uint8_t src_mac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, src_mac);
  int frame_len = 0;
  build_frame(frame, &frame_len, src_mac);

  ESP_LOGI(TAG, "uplink TX: src %02x:%02x:%02x:%02x:%02x:%02x frame=%d B "
                "interval=%d us",
           src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4],
           src_mac[5], frame_len,
           (int)CONFIG_AVB_ENDPOINT_PERF_TEST_TX_INTERVAL_US);

  uint32_t interval_us = CONFIG_AVB_ENDPOINT_PERF_TEST_TX_INTERVAL_US;
  uint32_t duration_sec = CONFIG_AVB_ENDPOINT_PERF_TEST_TX_DURATION_SEC;
  s_tx_start_us = esp_timer_get_time();
  int64_t deadline_us = (duration_sec > 0)
      ? s_tx_start_us + (int64_t)duration_sec * 1000000LL : 0;

  uint8_t seq = 0;
  while (deadline_us == 0 || esp_timer_get_time() < deadline_us) {
    frame[ETH_HDR_LEN + VLAN_TAG_LEN + 2] = seq++;
    esp_err_t r = esp_wifi_internal_tx(WIFI_IF_STA, frame, frame_len);
    if (r == ESP_OK) {
      s_tx_pkts++;
      s_tx_bytes_l2 += (uint64_t)frame_len;
    } else if (r == ESP_ERR_NO_MEM) {
      s_tx_busy++;
      vTaskDelay(1);
    } else {
      s_tx_fail++;
      vTaskDelay(1);
    }
    if (interval_us > 0) esp_rom_delay_us(interval_us);
  }
  s_tx_end_us = esp_timer_get_time();
  s_tx_done = true;

  double run_s = (double)(s_tx_end_us - s_tx_start_us) / 1e6;
  ESP_LOGW(TAG, "UPLINK FINAL tx run=%.2fs pkts=%llu L2_bytes=%llu "
                "fail=%llu busy=%llu mean_pps=%.0f mean_L2=%.2fMbps",
           run_s, (unsigned long long)s_tx_pkts,
           (unsigned long long)s_tx_bytes_l2,
           (unsigned long long)s_tx_fail, (unsigned long long)s_tx_busy,
           (double)s_tx_pkts / run_s,
           (double)(s_tx_bytes_l2 * 8ULL) / (run_s * 1e6));
  vTaskDelete(NULL);
}
#endif /* CONFIG_AVB_ENDPOINT_PERF_TEST_TX_MODE */

static void perf_stats_task(void *arg) {
  (void)arg;
  uint64_t last_pkts = 0;
  uint64_t last_bytes_l2 = 0;
  uint64_t last_bytes_wire = 0;
  int64_t last_t = esp_timer_get_time();
  uint32_t duration_sec = CONFIG_AVB_ENDPOINT_PERF_TEST_DURATION_SEC;
  int64_t deadline_us = 0;  /* set on first matching RX */
  bool summarised = false;
  bool done = false;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint64_t pkts = s_rx_pkts;
    uint64_t bytes_l2 = s_rx_bytes_l2;
    uint64_t bytes_wire = s_rx_bytes_wire;
    uint64_t gaps = s_rx_seq_gaps;
    uint64_t filt_dest = s_rx_filtered_dest;
    uint64_t filt_stream = s_rx_filtered_stream;
    int64_t first_us = s_rx_first_us;
    int64_t now = esp_timer_get_time();
    int64_t dt_us = now - last_t;
    if (dt_us <= 0) continue;
    uint64_t dp = pkts - last_pkts;
    uint64_t db_l2 = bytes_l2 - last_bytes_l2;
    uint64_t db_wire = bytes_wire - last_bytes_wire;
    last_pkts = pkts;
    last_bytes_l2 = bytes_l2;
    last_bytes_wire = bytes_wire;
    last_t = now;
    uint64_t pps = (dp * 1000000ULL) / (uint64_t)dt_us;
    double mbps_l2 = (double)(db_l2 * 8ULL) / (double)dt_us;
    double mbps_wire = (double)(db_wire * 8ULL) / (double)dt_us;
    ESP_LOGI(TAG,
             "rx pps=%llu L2=%.2fMbps wire=%.2fMbps gaps=%llu total=%llu "
             "filt(dest=%llu,stream=%llu)",
             (unsigned long long)pps, mbps_l2, mbps_wire,
             (unsigned long long)gaps, (unsigned long long)pkts,
             (unsigned long long)filt_dest, (unsigned long long)filt_stream);

    if (duration_sec > 0 && first_us > 0 && !done) {
      if (deadline_us == 0) {
        deadline_us = first_us + (int64_t)duration_sec * 1000000LL;
      }
      if (now >= deadline_us) done = true;
    }
    if (done && !summarised) {
      int64_t run_us = s_rx_last_us - first_us;
      if (run_us <= 0) run_us = 1;
      double mean_pps = (double)pkts * 1.0e6 / (double)run_us;
      double mean_l2 = (double)(bytes_l2 * 8ULL) / (double)run_us;
      double mean_wire = (double)(bytes_wire * 8ULL) / (double)run_us;
      ESP_LOGW(TAG,
               "FINAL rx run=%.2fs pkts=%llu L2_bytes=%llu wire_bytes=%llu "
               "gaps=%llu filt(dest=%llu,stream=%llu) mean_pps=%.0f "
               "mean_L2=%.2fMbps mean_wire=%.2fMbps",
               (double)run_us / 1.0e6,
               (unsigned long long)pkts, (unsigned long long)bytes_l2,
               (unsigned long long)bytes_wire, (unsigned long long)gaps,
               (unsigned long long)filt_dest, (unsigned long long)filt_stream,
               mean_pps, mean_l2, mean_wire);
      summarised = true;
    }
  }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data) {
  (void)arg; (void)base; (void)data;
  switch (id) {
  case WIFI_EVENT_STA_START:
    ESP_LOGI(TAG, "STA started — connecting to '%s'", PERF_AP_SSID);
    esp_wifi_connect();
    break;
  case WIFI_EVENT_STA_CONNECTED: {
    wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
    ESP_LOGI(TAG, "Associated. BSSID %02x:%02x:%02x:%02x:%02x:%02x ch=%u",
             e->bssid[0], e->bssid[1], e->bssid[2],
             e->bssid[3], e->bssid[4], e->bssid[5], e->channel);
#ifdef CONFIG_AVB_ENDPOINT_PERF_TEST_TX_MODE
    /* Uplink role: start blasting once associated. One-shot — the task
     * deletes itself at the end of its run. */
    {
      static bool tx_started = false;
      if (!tx_started) {
        tx_started = true;
        xTaskCreate(perf_tx_task, "perf_tx", 4096, NULL, 5, NULL);
      }
    }
#else
    /* Register the AVTP RX hook now that the STA interface is up.
     * Idempotent — last writer wins per IDF docs. */
    esp_err_t r = esp_wifi_internal_reg_rxcb(WIFI_IF_STA, perf_rx_cb);
    if (r != ESP_OK) {
      ESP_LOGE(TAG, "esp_wifi_internal_reg_rxcb failed: %d", r);
    } else {
      ESP_LOGI(TAG, "AVTP RX hook registered on WIFI_IF_STA");
    }
#endif
    xEventGroupSetBits(s_wifi_events, BIT_STA_CONNECTED);
    break;
  }
  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGW(TAG, "Disconnected — retry");
    xEventGroupClearBits(s_wifi_events, BIT_STA_CONNECTED);
    esp_wifi_connect();
    break;
  default:
    break;
  }
}

void avb_endpoint_perf_test_run(void) {
  ESP_LOGW(TAG, "perf test mode — PTP, AVB and codec NOT started");

  if (parse_mac(CONFIG_AVB_ENDPOINT_PERF_TEST_DEST_MAC,
                s_match_dest_mac) != 0) {
    ESP_LOGW(TAG, "bad dest MAC '%s', using 91:e0:f0:00:fe:00",
             CONFIG_AVB_ENDPOINT_PERF_TEST_DEST_MAC);
    s_match_dest_mac[0] = 0x91; s_match_dest_mac[1] = 0xe0;
    s_match_dest_mac[2] = 0xf0; s_match_dest_mac[3] = 0x00;
    s_match_dest_mac[4] = 0xfe; s_match_dest_mac[5] = 0x00;
  }
  if (parse_stream_id(CONFIG_AVB_ENDPOINT_PERF_TEST_STREAM_ID,
                      s_match_stream_id) != 0) {
    ESP_LOGW(TAG, "bad stream id '%s', using default",
             CONFIG_AVB_ENDPOINT_PERF_TEST_STREAM_ID);
    memset(s_match_stream_id, 0, 8);
    s_match_stream_id[7] = 1;
  }
  ESP_LOGI(TAG,
           "match dest=%02x:%02x:%02x:%02x:%02x:%02x "
           "stream_id=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
           s_match_dest_mac[0], s_match_dest_mac[1], s_match_dest_mac[2],
           s_match_dest_mac[3], s_match_dest_mac[4], s_match_dest_mac[5],
           s_match_stream_id[0], s_match_stream_id[1], s_match_stream_id[2],
           s_match_stream_id[3], s_match_stream_id[4], s_match_stream_id[5],
           s_match_stream_id[6], s_match_stream_id[7]);

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  esp_err_t loop_r = esp_event_loop_create_default();
  if (loop_r != ESP_OK && loop_r != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(loop_r);
  }
  esp_netif_create_default_wifi_sta();

  s_wifi_events = xEventGroupCreate();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  wifi_config_t sta_cfg = {
      .sta = {
          .ssid = PERF_AP_SSID,
          .password = "",
          .threshold.authmode = WIFI_AUTH_OPEN,
      },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &on_wifi_event, NULL));
  ESP_ERROR_CHECK(esp_wifi_start());

#ifdef CONFIG_AVB_ENDPOINT_PERF_TEST_TX_MODE
  if (CONFIG_AVB_ENDPOINT_PERF_TEST_FIX_RATE >= 0) {
    esp_err_t fr = esp_wifi_internal_set_fix_rate(
        WIFI_IF_STA, true, CONFIG_AVB_ENDPOINT_PERF_TEST_FIX_RATE);
    ESP_LOGW(TAG, "fix_rate(0x%02x) -> %s",
             CONFIG_AVB_ENDPOINT_PERF_TEST_FIX_RATE, esp_err_to_name(fr));
  }
#endif
  xTaskCreate(perf_stats_task, "perf_stats", 4096, NULL, 5, NULL);
}
