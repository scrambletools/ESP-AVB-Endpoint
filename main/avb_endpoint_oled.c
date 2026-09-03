/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * AVB Endpoint status OLED.
 *
 * Drives a small SSD1306 I2C panel as a stream-status indicator:
 * while idle it shows the last two octets of the endpoint MAC
 * (e.g. "92:20"), when input streams are active it shows "IN", when
 * output streams are active "OUT", and "IN OUT" when both are. Text
 * is rendered from a 5x7 font, integer-scaled to the largest size
 * that fits the panel, and centered.
 *
 * The panel shares the codec I2C bus, so the task attaches to the
 * bus that avb_start()'s codec init creates (via
 * i2c_master_get_bus_handle) instead of creating its own. With no
 * panel fitted the address probe fails and the task exits quietly,
 * so the option is safe to leave enabled on unequipped units.
 */

#include "avb_endpoint_oled.h"

#include "driver/i2c_master.h"
#include "esp_avb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define OLED_I2C_PORT 0 /* codec bus, created by avb_start() */
#define OLED_I2C_ADDR CONFIG_EXAMPLE_AVB_OLED_I2C_ADDR
#define OLED_I2C_SPEED_HZ 400000

/* Panel geometry. The SSD1306 RAM is always 128x64; smaller panels
 * map a window of it. The 0.49" 64x32 modules show columns 32..95,
 * so their framebuffer writes are shifted by a column offset. */
#if defined(CONFIG_EXAMPLE_AVB_OLED_64X32)
#define OLED_WIDTH 64
#define OLED_HEIGHT 32
#define OLED_COL_OFFSET 32
#define OLED_COM_PINS 0x12
#elif defined(CONFIG_EXAMPLE_AVB_OLED_128X32)
#define OLED_WIDTH 128
#define OLED_HEIGHT 32
#define OLED_COL_OFFSET 0
#define OLED_COM_PINS 0x02
#else
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_COL_OFFSET 0
#define OLED_COM_PINS 0x12
#endif
#define OLED_PAGES (OLED_HEIGHT / 8)

#define OLED_POLL_MS 250          /* status poll cadence */
#define OLED_BUS_WAIT_MS 500      /* codec-bus poll interval */
#define OLED_BUS_WAIT_TRIES 60    /* give up after 30 s without a bus */

static const char *TAG = "avb_oled";

static i2c_master_dev_handle_t s_oled_dev;
static uint8_t s_framebuf[OLED_WIDTH * OLED_PAGES];

/* 5x7 font, column-major, bit 0 = top row. Only the glyphs this
 * display needs: hex digits, colon, space, and the IN/OUT letters. */
typedef struct {
  char ch;
  uint8_t col[5];
} oled_glyph_s;

static const oled_glyph_s OLED_FONT[] = {
    {'0', {0x3e, 0x51, 0x49, 0x45, 0x3e}},
    {'1', {0x00, 0x42, 0x7f, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4b, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7f, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3c, 0x4a, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1e}},
    {'A', {0x7e, 0x11, 0x11, 0x11, 0x7e}},
    {'B', {0x7f, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3e, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7f, 0x41, 0x41, 0x22, 0x1c}},
    {'E', {0x7f, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7f, 0x09, 0x09, 0x09, 0x01}},
    {'I', {0x00, 0x41, 0x7f, 0x41, 0x00}},
    {'N', {0x7f, 0x04, 0x08, 0x10, 0x7f}},
    {'O', {0x3e, 0x41, 0x41, 0x41, 0x3e}},
    {'T', {0x01, 0x01, 0x7f, 0x01, 0x01}},
    {'U', {0x3f, 0x40, 0x40, 0x40, 0x3f}},
    {'K', {0x7f, 0x08, 0x14, 0x22, 0x41}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
};

static const uint8_t *oled_glyph(char ch) {
  for (size_t i = 0; i < sizeof(OLED_FONT) / sizeof(OLED_FONT[0]); i++) {
    if (OLED_FONT[i].ch == ch) {
      return OLED_FONT[i].col;
    }
  }
  return OLED_FONT[sizeof(OLED_FONT) / sizeof(OLED_FONT[0]) - 1].col; /* ' ' */
}

static esp_err_t oled_write_cmds(const uint8_t *cmds, size_t len) {
  /* Control byte 0x00, all following bytes are commands */
  uint8_t buf[32];
  if (len + 1 > sizeof(buf)) {
    return ESP_ERR_INVALID_SIZE;
  }
  buf[0] = 0x00;
  memcpy(&buf[1], cmds, len);
  return i2c_master_transmit(s_oled_dev, buf, len + 1, 100);
}

static esp_err_t oled_flush(void) {
  /* Reset the addressing window, then push the whole framebuffer in
   * one data transaction (control byte 0x40 = data stream). */
  const uint8_t window[] = {0x21, OLED_COL_OFFSET,
                            OLED_COL_OFFSET + OLED_WIDTH - 1, /* columns */
                            0x22, 0x00, OLED_PAGES - 1};      /* pages */
  esp_err_t err = oled_write_cmds(window, sizeof(window));
  if (err != ESP_OK) {
    return err;
  }
  static uint8_t txbuf[1 + sizeof(s_framebuf)];
  txbuf[0] = 0x40;
  memcpy(&txbuf[1], s_framebuf, sizeof(s_framebuf));
  return i2c_master_transmit(s_oled_dev, txbuf, sizeof(txbuf), 200);
}

static esp_err_t oled_init_panel(void) {
  const uint8_t init_cmds[] = {
      0xae,             /* display off */
      0xd5, 0x80,       /* clock divide ratio / oscillator */
      0xa8, OLED_HEIGHT - 1, /* multiplex ratio */
      0xd3, 0x00,       /* display offset */
      0x40,             /* start line 0 */
      0x8d, 0x14,       /* charge pump on */
      0x20, 0x00,       /* horizontal addressing mode */
      0xa1,             /* segment remap, column 127 at the left */
      0xc8,             /* COM scan direction remapped */
      0xda, OLED_COM_PINS, /* COM pins config for the panel geometry */
      0x81, 0xcf,       /* contrast */
      0xd9, 0xf1,       /* pre-charge period */
      0xdb, 0x40,       /* VCOMH deselect level */
      0xa4,             /* resume from RAM content */
      0xa6,             /* normal (non-inverted) polarity */
  };
  esp_err_t err = oled_write_cmds(init_cmds, sizeof(init_cmds));
  if (err != ESP_OK) {
    return err;
  }
  memset(s_framebuf, 0, sizeof(s_framebuf));
  err = oled_flush();
  if (err != ESP_OK) {
    return err;
  }
  const uint8_t display_on = 0xaf;
  return oled_write_cmds(&display_on, 1);
}

static void oled_set_pixel(int x, int y) {
  if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
    return;
  }
  s_framebuf[(y / 8) * OLED_WIDTH + x] |= (uint8_t)(1u << (y % 8));
}

/* Glyph columns for a string, character cell is 6x8 (5x7 glyph plus
 * one spacing column, none trailing). */
static int oled_text_cols(size_t text_len) {
  return (int)text_len * 6 - 1;
}

/* Draw one line of text at a given scale, horizontally centered. */
static void oled_draw_line(const char *text, size_t text_len, int scale,
                           int y_origin) {
  int x_origin = (OLED_WIDTH - oled_text_cols(text_len) * scale) / 2;
  for (size_t char_idx = 0; char_idx < text_len; char_idx++) {
    const uint8_t *columns = oled_glyph(text[char_idx]);
    for (int col = 0; col < 5; col++) {
      for (int row = 0; row < 7; row++) {
        if (!(columns[col] & (1u << row))) {
          continue;
        }
        int px = x_origin + ((int)char_idx * 6 + col) * scale;
        int py = y_origin + row * scale;
        for (int dx = 0; dx < scale; dx++) {
          for (int dy = 0; dy < scale; dy++) {
            oled_set_pixel(px + dx, py + dy);
          }
        }
      }
    }
  }
}

static int oled_fit_scale(int glyph_cols, int glyph_rows) {
  int scale_x = OLED_WIDTH / glyph_cols;
  int scale_y = OLED_HEIGHT / glyph_rows;
  return scale_x < scale_y ? scale_x : scale_y;
}

/* Render text centered at the largest integer scale that fits. Text
 * with a space (only "IN OUT" today) is also tried as two stacked
 * lines, whichever layout yields the bigger glyphs wins. */
static void oled_render_text(const char *text) {
  size_t text_len = strlen(text);
  memset(s_framebuf, 0, sizeof(s_framebuf));
  if (text_len == 0) {
    return;
  }

  int single_scale = oled_fit_scale(oled_text_cols(text_len), 8);

  const char *space = strchr(text, ' ');
  if (space && space != text && space[1] != '\0') {
    size_t first_len = (size_t)(space - text);
    size_t second_len = text_len - first_len - 1;
    size_t widest_len = first_len > second_len ? first_len : second_len;
    /* Stacked rows: 7 + 1 gap + 7, plus one for breathing room */
    int stacked_scale = oled_fit_scale(oled_text_cols(widest_len), 16);
    if (stacked_scale > single_scale) {
      int total_height = 15 * stacked_scale;
      int y_origin = (OLED_HEIGHT - total_height) / 2;
      oled_draw_line(text, first_len, stacked_scale, y_origin);
      oled_draw_line(space + 1, second_len, stacked_scale,
                     y_origin + 8 * stacked_scale);
      return;
    }
  }

  if (single_scale < 1) {
    single_scale = 1;
  }
  oled_draw_line(text, text_len, single_scale,
                 (OLED_HEIGHT - 7 * single_scale) / 2);
}

static void oled_task(void *task_param) {
  (void)task_param;

  /* The codec init inside the AVB task owns bus creation, wait for
   * it. This also guarantees the AVB task is up before avb_status()
   * is ever called. */
  i2c_master_bus_handle_t bus = NULL;
  for (int attempt = 0; attempt < OLED_BUS_WAIT_TRIES; attempt++) {
    /* Delay first: polling before codec init logs a driver error. */
    vTaskDelay(pdMS_TO_TICKS(OLED_BUS_WAIT_MS));
    if (i2c_master_get_bus_handle(OLED_I2C_PORT, &bus) == ESP_OK && bus) {
      break;
    }
    bus = NULL;
  }
  if (!bus) {
    ESP_LOGW(TAG, "codec I2C bus never appeared, status OLED disabled");
    vTaskDelete(NULL);
    return;
  }

  if (i2c_master_probe(bus, OLED_I2C_ADDR, 100) != ESP_OK) {
    ESP_LOGW(TAG, "no OLED at 0x%02x, status OLED disabled", OLED_I2C_ADDR);
    vTaskDelete(NULL);
    return;
  }

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = OLED_I2C_ADDR,
      .scl_speed_hz = OLED_I2C_SPEED_HZ,
  };
  if (i2c_master_bus_add_device(bus, &dev_cfg, &s_oled_dev) != ESP_OK ||
      oled_init_panel() != ESP_OK) {
    ESP_LOGW(TAG, "OLED init failed, status OLED disabled");
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG, "status OLED up (%dx%d at 0x%02x)", OLED_WIDTH, OLED_HEIGHT,
           OLED_I2C_ADDR);

  char shown_text[24] = "";
  while (1) {
    avb_status_s status;
    char text[24];
    if (avb_status(&status) == 0) {
      /* Sample rate as a short label, e.g. 44.1K, 48K, 192K */
      char rate_label[16] = "";
      uint32_t rate_khz = status.sample_rate / 1000;
      uint32_t rate_frac = (status.sample_rate % 1000) / 100;
      if (rate_frac) {
        snprintf(rate_label, sizeof(rate_label), " %u.%uK",
                 (unsigned)rate_khz, (unsigned)rate_frac);
      } else if (rate_khz) {
        snprintf(rate_label, sizeof(rate_label), " %uK", (unsigned)rate_khz);
      }
      if (status.streaming_in && status.streaming_out) {
        snprintf(text, sizeof(text), "IN OUT");
      } else if (status.streaming_in) {
        /* Space triggers the stacked layout: IN over the rate */
        snprintf(text, sizeof(text), "IN%s", rate_label);
      } else if (status.streaming_out) {
        snprintf(text, sizeof(text), "OUT%s", rate_label);
      } else {
        /* Entity ID is the MAC plus a 2-byte suffix, bytes 4 and 5
         * are the last two MAC octets. */
        snprintf(text, sizeof(text), "%02X:%02X", status.entity.id[4],
                 status.entity.id[5]);
      }
    } else {
      snprintf(text, sizeof(text), "--:--"); /* AVB not answering */
    }

    if (strcmp(text, shown_text) != 0) {
      oled_render_text(text);
      if (oled_flush() == ESP_OK) {
        ESP_LOGI(TAG, "display: \"%s\" (was \"%s\")", text, shown_text);
        strcpy(shown_text, text);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(OLED_POLL_MS));
  }
}

void avb_endpoint_oled_start(void) {
  /* Priority must stay below the AVB task so avb_status() requests
   * cannot starve it (see the avb_status() contract). */
  xTaskCreate(oled_task, "avb_oled", 3072, NULL, 2, NULL);
}
