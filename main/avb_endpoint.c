/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * AVB Endpoint application for ESP-IDF.
 *
 * Single project that targets multiple endpoint configurations via
 * the esp_ptp per-port Kconfig taxonomy
 * (MEDIUM × HOST_IF × TYPE × WIFI_MODE × LINK_SPEED_MBPS):
 *
 *   - Wired endpoint (default, target esp32p4): PORT0 MEDIUM=ETHERNET,
 *     HOST_IF=EMAC, TYPE=PRIMARY. Full AVB stack on Ethernet.
 *
 *   - Wireless endpoint (target esp32c6): PORT0 MEDIUM=WIFI,
 *     HOST_IF=AHB, TYPE=PRIMARY, WIFI_MODE=STA. Wi-Fi STA associates
 *     to the bridge, consumes beacon-IE FollowUpInformation, initiates
 *     FTM.
 *
 *   - Mixed endpoint (e.g. ETHERNET PRIMARY + WIFI/SDIO STA FAILOVER):
 *     future, both medium blocks fire.
 *
 * Per-medium blocks are gated with #ifdef on the relevant port-medium
 * Kconfig. main/CMakeLists.txt additionally target-gates PRIV_REQUIRES
 * so unused deps (esp_wifi on p4, esp_eth on c6) don't appear in the
 * component graph.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <esp_heap_trace.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
/* esp_ptp public API and internal wire-format types. Both used in
 * either-medium builds: the public API for ptpd_start_port etc., the
 * internal types for the §12.7 Vendor IE parser. esp_ptp's
 * CMakeLists exposes "." (for ptp.h) and "./include" (for esp_ptp.h)
 * via INCLUDE_DIRS, and esp_ptp is in our PRIV_REQUIRES. */
#include <esp_ptp.h>
#include "ptp.h"

#ifdef CONFIG_ESP_PTP_PORT0_MEDIUM_ETH_HWTS
#include "esp_avb.h"
#include "esp_eth_clock.h"
#include <esp_check.h>
#include <esp_eth.h>
#include <esp_eth_phy_ip101.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_vfs_l2tap.h>
#endif

#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM) ||                               \
    defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM)
#include "esp_avb.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#endif

static const char *TAG = "avb_endpoint";

/* ETHERNET medium (PORT0) */
#ifdef CONFIG_ESP_PTP_PORT0_MEDIUM_ETH_HWTS

static esp_eth_handle_t s_eth_handle;
static char s_avb_eth_interface[10];

static void init_ethernet_and_netif(void) {
  /* Default event loop may already exist (e.g. created by esp_ptp at
   * constructor time when a wifi_cp port is in the build).
   * ESP_ERR_INVALID_STATE means "already created", which is fine. */
  esp_err_t loop_r = esp_event_loop_create_default();
  if (loop_r != ESP_OK && loop_r != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(loop_r);
  }

  eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

  emac_config.dma_burst_len = ETH_DMA_BURST_LEN_32;
  emac_config.intr_priority = 0;
  mac_config.rx_task_stack_size = 16384;
  mac_config.rx_task_prio = 22;
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = 5;

  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_ERROR_CHECK(esp_eth_driver_install(&config, &s_eth_handle));
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_vfs_l2tap_intf_register(NULL));

  esp_netif_inherent_config_t esp_netif_base_config =
      ESP_NETIF_INHERENT_DEFAULT_ETH();
  esp_netif_config_t esp_netif_config = {
      .base = &esp_netif_base_config, .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH};
  esp_netif_base_config.if_key = "ETH_0";
  esp_netif_base_config.if_desc = "eth0";
  esp_netif_base_config.route_prio = 50;
  esp_netif_t *eth_netif = esp_netif_new(&esp_netif_config);

  ESP_ERROR_CHECK(
      esp_netif_attach(eth_netif, esp_eth_new_netif_glue(s_eth_handle)));

  memcpy(s_avb_eth_interface, esp_netif_base_config.if_key,
         strlen(esp_netif_base_config.if_key));
  ESP_LOGI(TAG, "AVB ethernet interface: %s", s_avb_eth_interface);

  ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
}

static void start_ethernet_endpoint(void) {
  struct timespec cur_time;

  init_ethernet_and_netif();
  ESP_LOGI(TAG, "Ethernet started");

  ptpd_start(s_avb_eth_interface);

  while (clock_gettime(CLOCK_PTP_SYSTEM, &cur_time) == -1) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  avb_config_s avb_config = AVB_DEFAULT_CONFIG();
  avb_config.entity_name = CONFIG_EXAMPLE_AVB_ENTITY_NAME;
  avb_config.model_id = CONFIG_EXAMPLE_AVB_MODEL_ID;
  avb_config.talker = CONFIG_EXAMPLE_AVB_TALKER;
  avb_config.listener = CONFIG_EXAMPLE_AVB_LISTENER;
  avb_config.default_mic_gain_tenth_db = CONFIG_EXAMPLE_AVB_MIC_GAIN_TENTH_DB;
  avb_config.default_speaker_vol_tenth_db =
      CONFIG_EXAMPLE_AVB_SPEAKER_VOL_TENTH_DB;
  avb_config.atdecc_control = CONFIG_EXAMPLE_AVB_REMOTE_CONTROL;
  avb_config.eth_handle = s_eth_handle;
  avb_config.eth_interface = "ETH_0";
#ifdef CONFIG_EXAMPLE_AVB_CODEC_DISABLED
  avb_config.codec_disabled = true;
#endif

#ifdef CONFIG_EXAMPLE_AVB_CODEC_ES8388_HAT
  /* Override the default ES8311 onboard codec selection + pins with
   * the Scramble Hat ES8388 wiring (pinout documented in
   * esp_avb/avbconfig.h's "Scramble Hat" comment block). codec_type
   * is const-qualified in avb_config_s — cast away const for this
   * init-time override; nothing reads codec_type before avb_start()
   * runs so the one-shot mutation is safe. */
  *(avb_codec_type_t *)&avb_config.codec_type = avb_codec_type_es8388;
  avb_config.codec_pins.mclk    = 16;
  avb_config.codec_pins.bclk    = 17;
  avb_config.codec_pins.ws      = 19;
  avb_config.codec_pins.dout    = 18;
  avb_config.codec_pins.din     = 54;
  avb_config.codec_pins.i2c_sda = 21;
  avb_config.codec_pins.i2c_scl = 20;
  avb_config.codec_pins.pa      = -1;
  avb_config.codec_pins.pa_reverted = false;
#endif

  esp_eth_io_cmd_t cmd = ETH_CMD_S_PROMISCUOUS;
  bool promiscuous = true;
  if (esp_eth_ioctl(s_eth_handle, cmd, &promiscuous) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ethernet to promiscuous mode");
    abort();
  }

  avb_start(&avb_config);
}

#endif /* CONFIG_ESP_PTP_PORT0_MEDIUM_ETH_HWTS */

/* Wi-Fi medium (STA role).
 * Associates to the bridge SoftAP, consumes the beacon Vendor IE
 * FollowUpInformation, and runs an FTM initiator burst at the
 * §12.8.2 cadence. Full AVB stack runs on the Wi-Fi data plane
 * via avb_start() with codec disabled. */
#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM) ||                               \
    defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM)

#define AVB_AP_SSID "ESP-AVB-Bridge"

static EventGroupHandle_t s_wifi_events;
#define BIT_STA_CONNECTED BIT0

/* RX-stall watchdog. The C6 Wi-Fi receive path wedges after a few
 * minutes of uptime: layered RX instrumentation (esp_avb avbstats.c)
 * proved the radio/driver KEEPS receiving (esp_wifi_get_rx_statistics
 * legacy/ht counters climb the whole time) but frames stop being handed
 * up to our registered rxcb (avb_wifi_rx_cb) — so the dispatcher RX
 * counter freezes while heap stays flat. The leak is in the esp_wifi
 * driver's deliver-to-host buffer/descriptor pool, ABOVE the MAC and
 * BELOW our callback. A soft disassoc/reassoc does NOT clear it (tested:
 * the dispatcher counter stayed frozen across 9 forced reassociations) —
 * the radio never lost the link, so reassoc doesn't touch the wedged
 * pool. Recovery therefore re-inits the Wi-Fi driver itself with
 * esp_wifi_stop()+esp_wifi_start(); the WIFI_EVENT_STA_START handler
 * then reconnects. Detection: watch esp_avb's monotonic dispatcher RX
 * counter; if it stops advancing for RX_STALL_TIMEOUT_US of WALL-CLOCK
 * time while associated, restart the driver.
 *
 * Timing is wall-clock via esp_timer_get_time(), NOT a count of
 * heartbeat-loop iterations: when RX wedges, the main loop's own cadence
 * slows markedly (observed ~10x), so an iteration counter took ~300 s to
 * reach a nominal "30 s" threshold. esp_timer is unaffected by loop
 * starvation, so the trip fires at a true RX_STALL_TIMEOUT_US.
 * avb_net_rx_breakdown is declared extern here (it lives in esp_avb's
 * internal avb.h, not the public esp_avb.h) following the same pattern
 * as other internal esp_avb/esp_wifi entry points in this codebase. */
extern void avb_net_rx_breakdown(uint32_t *total, uint32_t *avtp,
                                 uint32_t *msrp, uint32_t *mvrp,
                                 uint32_t *vlan, uint32_t *other);
/* Light first-line RX recovery: re-point the WIFI_IF_STA rxcb slot back
 * at esp_avb's dispatcher. Same local-extern rationale as above. */
extern esp_err_t avb_net_wifi_reregister_rxcb(void);
#define RX_STALL_TIMEOUT_US (10 * 1000 * 1000) /* 10 s of no RX advance */

/* Heavy second-line RX recovery: re-initialize the Wi-Fi driver.
 * stop() tears down the driver's RX path (freeing any wedged delivery
 * buffers); start() re-arms it and fires WIFI_EVENT_STA_START, whose
 * handler calls esp_wifi_connect() to rejoin. The watchdog only
 * escalates here when a cheap rxcb re-registration first failed to
 * restore delivery — i.e. the stall is deeper than a displaced rxcb
 * slot. Heavier than a reassoc. */
static void wifi_rx_stall_recover(void) {
  esp_err_t r = esp_wifi_stop();
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "RX-stall recover: esp_wifi_stop failed: %s",
             esp_err_to_name(r));
  }
  r = esp_wifi_start();
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "RX-stall recover: esp_wifi_start failed: %s",
             esp_err_to_name(r));
  }
}

/* Application-side Wi-Fi housekeeping. Connect/reconnect logic and a
 * connection-ready signal that start_wifi_endpoint waits on before
 * bringing up the AVB stack. All §12.7 / FTM handling lives inside
 * esp_ptp's ptp_wifi_sta module — see ptp_wifi_sta.c. */
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data) {
  (void)arg;
  (void)base;
  switch (id) {
  case WIFI_EVENT_STA_START:
    ESP_LOGI(TAG, "STA started — connecting to '%s'", AVB_AP_SSID);
    esp_wifi_connect();
    break;
  case WIFI_EVENT_STA_CONNECTED: {
    wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
    ESP_LOGI(TAG, "Associated. BSSID %02x:%02x:%02x:%02x:%02x:%02x channel %u",
             e->bssid[0], e->bssid[1], e->bssid[2], e->bssid[3], e->bssid[4],
             e->bssid[5], e->channel);
    xEventGroupSetBits(s_wifi_events, BIT_STA_CONNECTED);
    break;
  }
  case WIFI_EVENT_STA_DISCONNECTED: {
    wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
    /* reason distinguishes AP-not-found (201) from auth/handshake failure
     * (2/15) from a deauth by the AP — essential for telling "bridge AP
     * down" apart from a local config/credential problem. */
    ESP_LOGW(TAG, "Disconnected — retry (reason %u, rssi %d)", e->reason,
             e->rssi);
    xEventGroupClearBits(s_wifi_events, BIT_STA_CONNECTED);
    esp_wifi_connect();
    break;
  }
  default:
    break;
  }
}

static void wifi_sta_init(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  esp_err_t loop_r = esp_event_loop_create_default();
  if (loop_r != ESP_OK && loop_r != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(loop_r);
  }
  /* Deliberately NOT creating the default STA netif. Its STA event
   * handlers re-attach the netif's own Wi-Fi RX callback on every
   * (re)association, which displaces the raw esp_wifi_internal_reg_rxcb
   * hook esp_avb installs for the AVB data plane — the root cause of the
   * post-reassociation RX freeze. This endpoint is pure L2 (gPTP/AVTP
   * over raw frames, no IP), so it needs no lwIP netif; esp_avb reads
   * the STA MAC directly via esp_wifi_get_mac when no netif is present. */

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  /* Power save off — beacon-IE timing precision and FTM accuracy both
   * degrade with PS active. AVB-active STAs run with PS off. */
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  wifi_config_t sta_cfg = {
      .sta =
          {
              .ssid = AVB_AP_SSID,
              .password = "",
              .threshold.authmode = WIFI_AUTH_OPEN,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &on_wifi_event, NULL));

  ESP_ERROR_CHECK(esp_wifi_start());
}

static void start_wifi_endpoint(void) {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  ESP_LOGI(TAG, "Wi-Fi STA endpoint boot. MAC %02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  s_wifi_events = xEventGroupCreate();
  wifi_sta_init();

  /* Wait for STA association before bringing up AVB so the RX callback that
   * avb_net_init registers (esp_wifi_internal_reg_rxcb) sees a
   * connected interface. eth_handle=NULL signals the wifi data-plane
   * to avb_net_transmit_raw + the senders. eth_interface is just a
   * label here ("WIFI_STA_DEF"): with no default STA netif created,
   * the if_key lookup in avb_net_init returns NULL and it reads the
   * STA MAC directly via esp_wifi_get_mac. */
  ESP_LOGI(TAG, "Waiting for STA association before starting AVB");
  xEventGroupWaitBits(s_wifi_events, BIT_STA_CONNECTED, pdFALSE, pdTRUE,
                      portMAX_DELAY);

  /* Bootstrap ptpd on the wifi_ftm port. esp_ptp internally spawns
   * the §12.7 beacon-IE parser, the FTM initiator burst loop, and
   * the FTM_REPORT handler that feeds inject_peer_delay +
   * inject_sync_pair (see ptp_wifi_sta.c). The application no longer
   * touches §12.7 IE bytes or FTM cadence. "WIFI_STA_DEF" is just a
   * medium label here; the daemon does not open a netif. */
  if (ptpd_start_port(0, "WIFI_STA_DEF", ptp_port_medium_wifi_ftm) < 0) {
    ESP_LOGE(TAG, "ptpd_start_port wifi_ftm failed — wireless clock will "
                  "not lock");
  }

  avb_config_s avb_config = AVB_DEFAULT_CONFIG();
  avb_config.entity_name = CONFIG_EXAMPLE_AVB_ENTITY_NAME;
  avb_config.model_id = CONFIG_EXAMPLE_AVB_MODEL_ID;
  avb_config.talker = CONFIG_EXAMPLE_AVB_TALKER;
  avb_config.listener = CONFIG_EXAMPLE_AVB_LISTENER;
  avb_config.default_mic_gain_tenth_db = CONFIG_EXAMPLE_AVB_MIC_GAIN_TENTH_DB;
  avb_config.default_speaker_vol_tenth_db =
      CONFIG_EXAMPLE_AVB_SPEAKER_VOL_TENTH_DB;
  avb_config.atdecc_control = CONFIG_EXAMPLE_AVB_REMOTE_CONTROL;
  avb_config.eth_handle = NULL;
  avb_config.eth_interface = "WIFI_STA_DEF";
  /* C6 test board's onboard ES8311 (per avbconfig.h comment block).
   * Default codec pins (12/13 = MCLK/BCLK) collide with C6 USB-Serial-
   * JTAG (D+/D-), so we override to the C6 board's actual ES8311
   * wiring. None of these pins overlap with USB on the C6. */
  avb_config.codec_pins.mclk = 19;
  avb_config.codec_pins.bclk = 20;
  avb_config.codec_pins.ws = 22;
  avb_config.codec_pins.dout = 21;
  avb_config.codec_pins.din = 23;
  avb_config.codec_pins.i2c_sda = 8;
  avb_config.codec_pins.i2c_scl = 7;
  avb_config.codec_pins.pa = 6;
  avb_config.codec_pins.pa_reverted = false;

  avb_start(&avb_config);
  ESP_LOGI(TAG, "AVB stack started on wifi port");
}

#endif /* WIFI medium configured */

#ifdef CONFIG_HEAP_TRACING_STANDALONE
static volatile bool s_heap_trace_dump_pending = false;

void avb_endpoint_heap_trace_start(void *arg) {
  (void)arg;
  esp_err_t r = heap_trace_start(HEAP_TRACE_LEAKS);
  ESP_LOGW(TAG, "heap_trace_start: %s", esp_err_to_name(r));
}

/* esp_timer callbacks run in a high-priority task with a short watchdog
 * — dumping hundreds of records blocks too long and panics. Defer to
 * the main heartbeat task. */
void avb_endpoint_heap_trace_stop(void *arg) {
  (void)arg;
  heap_trace_stop();
  s_heap_trace_dump_pending = true;
}
#endif

void app_main(void) {
#ifdef CONFIG_AVB_ENDPOINT_PERF_TEST_MODE
  /* Perf-test mode bypasses every production subsystem. Brings up only
   * NVS + WiFi STA and counts matching AVTP frames via an
   * esp_wifi_internal_reg_rxcb hook. Never returns to the production
   * bring-up below. */
  extern void avb_endpoint_perf_test_run(void);
  avb_endpoint_perf_test_run();
  return;
#endif

#ifdef CONFIG_ESP_PTP_PORT0_MEDIUM_ETH_HWTS
  start_ethernet_endpoint();
#endif
#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM) ||                               \
    defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM)
  start_wifi_endpoint();
#endif

#ifdef CONFIG_HEAP_TRACING_STANDALONE
  /* Leak-hunt scaffold: start tracing after the boot transient
   * quiesces, stop+flag after HEAP_TRACE_DURATION_S. Pair with the
   * hexdump loop further down — backtraces are empty on RISC-V for
   * the wifi blob's alloc sites, so we identify by buffer content. */
  #define HEAP_TRACE_RECORDS       128
  #define HEAP_TRACE_START_DELAY_S 30
  #define HEAP_TRACE_DURATION_S    15
  static heap_trace_record_t s_trace_buf[HEAP_TRACE_RECORDS];
  if (heap_trace_init_standalone(s_trace_buf, HEAP_TRACE_RECORDS) != ESP_OK) {
    ESP_LOGE(TAG, "heap_trace_init_standalone failed");
  } else {
    static esp_timer_handle_t s_start_timer, s_stop_timer;
    const esp_timer_create_args_t start_args = {
        .callback = &avb_endpoint_heap_trace_start,
        .name = "heap_trace_start",
    };
    const esp_timer_create_args_t stop_args = {
        .callback = &avb_endpoint_heap_trace_stop,
        .name = "heap_trace_stop",
    };
    esp_timer_create(&start_args, &s_start_timer);
    esp_timer_create(&stop_args, &s_stop_timer);
    esp_timer_start_once(s_start_timer,
                         (uint64_t)HEAP_TRACE_START_DELAY_S * 1000000ULL);
    esp_timer_start_once(
        s_stop_timer,
        (uint64_t)(HEAP_TRACE_START_DELAY_S + HEAP_TRACE_DURATION_S) *
            1000000ULL);
    ESP_LOGI(TAG, "Heap trace scheduled: start @%ds, dump @%ds",
             HEAP_TRACE_START_DELAY_S,
             HEAP_TRACE_START_DELAY_S + HEAP_TRACE_DURATION_S);
  }
#endif

  vTaskDelay(pdMS_TO_TICKS(3000));

  /* Task handles for memory consumption monitoring */
  static const unsigned int task_monitor_period = 1000;
  static const unsigned int task_monitor_threshold = 1000;
  char t0_name[] = "main_task";
  char t1_name[] = "AVB";
  char t2_name[] = "PTPD";
  TaskHandle_t t0 = xTaskGetHandle(t0_name);
  TaskHandle_t t1 = xTaskGetHandle(t1_name);
  TaskHandle_t t2 = xTaskGetHandle(t2_name);

#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM) ||                               \
    defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM)
  /* RX-stall watchdog state. Wall-clock based: remember the dispatcher
   * RX total and the esp_timer timestamp when it last advanced. */
  uint32_t rx_stall_prev_total = 0;
  int64_t rx_stall_last_advance_us = 0;
  bool rx_stall_primed = false;
  bool rx_stall_rereg_tried = false; /* gentle rxcb re-reg attempted? */
#endif

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(task_monitor_period));
    ESP_LOGI(TAG, "heartbeat");

#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM) ||                               \
    defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM)
    /* RX-stall watchdog — only meaningful while associated; the driver
     * restart clears BIT_STA_CONNECTED (via WIFI_EVENT_STA_DISCONNECTED)
     * and re-arms the detector once it reassociates. */
    if (s_wifi_events &&
        (xEventGroupGetBits(s_wifi_events) & BIT_STA_CONNECTED)) {
      uint32_t rx_total = 0;
      avb_net_rx_breakdown(&rx_total, NULL, NULL, NULL, NULL, NULL);
      int64_t now_us = esp_timer_get_time();
      if (!rx_stall_primed || rx_total != rx_stall_prev_total) {
        /* First sample after (re)association, or RX advanced — reset the
         * stall clock and the recovery escalation ladder. */
        rx_stall_prev_total = rx_total;
        rx_stall_last_advance_us = now_us;
        rx_stall_primed = true;
        rx_stall_rereg_tried = false;
      } else if (now_us - rx_stall_last_advance_us >= RX_STALL_TIMEOUT_US) {
        if (!rx_stall_rereg_tried) {
          /* First-line recovery: the single rxcb slot may have been
           * displaced (netif reattach on reconnect re-points it at the
           * lwIP path). Re-register ours and grant another
           * RX_STALL_TIMEOUT_US to prove out before escalating. */
          ESP_LOGW(TAG,
                   "Wi-Fi RX stalled %lldms (dispatcher total=%u frozen) — "
                   "re-registering rxcb",
                   (long long)((now_us - rx_stall_last_advance_us) / 1000),
                   (unsigned)rx_total);
          esp_err_t rr = avb_net_wifi_reregister_rxcb();
          if (rr != ESP_OK) {
            ESP_LOGE(TAG, "rxcb re-register failed: %s", esp_err_to_name(rr));
          }
          rx_stall_rereg_tried = true;
          rx_stall_last_advance_us = now_us; /* grace period for re-reg */
        } else {
          /* Re-reg did not restore flow within the grace period — the
           * stall is deeper than a displaced rxcb. Escalate to a driver
           * restart. */
          ESP_LOGW(TAG,
                   "Wi-Fi RX still stalled %lldms after rxcb re-register "
                   "(dispatcher total=%u) — restarting Wi-Fi driver",
                   (long long)((now_us - rx_stall_last_advance_us) / 1000),
                   (unsigned)rx_total);
          rx_stall_primed = false;      /* re-arm on next sample post-restart */
          rx_stall_rereg_tried = false; /* reset escalation ladder */
          wifi_rx_stall_recover();
        }
      }
    } else {
      rx_stall_primed = false;
      rx_stall_rereg_tried = false;
    }
#endif
#ifdef CONFIG_HEAP_TRACING_STANDALONE
    if (s_heap_trace_dump_pending) {
      s_heap_trace_dump_pending = false;
      /* Skip heap_trace_dump() — its esp_rom_printf hammers UART and
       * trips the interrupt watchdog. Iterate via the public API,
       * print one ESP_LOG per record with a yield in between, and
       * hexdump the first 32 bytes so leaks can be identified by
       * content (backtraces are empty on RISC-V wifi-blob allocs
       * even with FRAME_POINTER on). */
      size_t cnt = heap_trace_get_count();
      ESP_LOGW(TAG, "=== HEAP LEAK HEXDUMP (%u records) ===",
               (unsigned)cnt);
      for (size_t i = 0; i < cnt; i++) {
        heap_trace_record_t rec;
        if (heap_trace_get(i, &rec) != ESP_OK) continue;
        if (rec.size == 0 || rec.address == NULL) continue;
        size_t n = rec.size < 32 ? rec.size : 32;
        char hex[3 * 32 + 8] = {0};
        for (size_t j = 0; j < n; j++) {
          snprintf(hex + 3 * j, 4, "%02x ", ((uint8_t *)rec.address)[j]);
        }
        ESP_LOGW(TAG, "  leak[%u] size=%u @%p : %s", (unsigned)i,
                 (unsigned)rec.size, rec.address, hex);
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      ESP_LOGW(TAG, "=== HEAP LEAK HEXDUMP END ===");
    }
#endif

    if (t0 && uxTaskGetStackHighWaterMark(t0) < task_monitor_threshold)
      ESP_LOGI(TAG, "TASK %s high water mark = %d", t0_name,
               uxTaskGetStackHighWaterMark(t0));
    if (t1 && uxTaskGetStackHighWaterMark(t1) < task_monitor_threshold)
      ESP_LOGI(TAG, "TASK %s high water mark = %d", t1_name,
               uxTaskGetStackHighWaterMark(t1));
    if (t2 && uxTaskGetStackHighWaterMark(t2) < task_monitor_threshold)
      ESP_LOGI(TAG, "TASK %s high water mark = %d", t2_name,
               uxTaskGetStackHighWaterMark(t2));
  }
}
