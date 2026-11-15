/*
 * Echo cancellation test path (CONFIG_EXAMPLE_AVB_AEC, test builds).
 *
 * The talker hands avb_talker_capture_hook() each packet's codec capture
 * and, from the listener mirror, the samples the listener played over
 * the same frames (both 48 kHz stereo 24-in-32, MSB first, codec
 * clock). The hook queues the left mic and the playout (left and right
 * averaged) for a lower-priority task, which converts them to 16 kHz,
 * runs ESP-SR's echo canceller, converts the result back to 48 kHz and
 * queues it for the hook to send. On the wire:
 *   channel 0: the echo-cancelled mic,
 *   channel 1: the raw mic (DC removed), delayed by the same fixed
 *              amount so both channels stay sample aligned.
 * The fixed delay is the start-up fill plus the converters' group delay
 * (measured at start with an impulse), the mic lead over the reference
 * and the canceller's one-frame lag. A late frame is sent as silence and
 * made up by skipping, so alignment holds. Needs PSRAM (ESP-SR allocates
 * from it); 48 kHz streams only. CONFIG_EXAMPLE_AVB_AEC_SELFTEST and
 * _BENCH_CYCLE measure cost and echo reduction with nothing played.
 */
#include <math.h>
#include <stdio.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "esp_aec.h"
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_avb.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AVB-AEC";


#define AEC_RATE_IN 48000
#define AEC_RATE 16000
#define AEC_DECIM 3
#define AEC_FIR_TAPS 63 /* odd: linear phase, integer group delay */
#define AEC_FIR_PHASE_TAPS ((AEC_FIR_TAPS + AEC_DECIM - 1) / AEC_DECIM)
#define AEC_IN_CAP 8192  /* 48 kHz frames, power of two (~170 ms) */
#define AEC_OUT_CAP 8192 /* 48 kHz samples, power of two */
#define AEC_RAW_CAP 8192 /* raw delay line, power of two */
#define AEC_MIC_CAP 2048 /* mic lead line, power of two */
#define AEC_START_BLOCKS 3

/* Hook -> task: mic and reference at 48 kHz. */
typedef struct {
  int32_t mic;
  int32_t ref;
} aec_pair_t;

static aec_pair_t *s_in;
static _Atomic uint32_t s_in_head, s_in_tail;
/* Task -> hook: cleaned mic at 48 kHz. */
static int32_t *s_out;
static _Atomic uint32_t s_out_head, s_out_tail;

static aec_handle_t *s_aec;
static TaskHandle_t s_task;
static int s_chunk;   /* 16 kHz samples per AEC frame */
static int s_block;   /* 48 kHz samples per AEC frame */
static bool s_ready;

static float s_fir[AEC_FIR_TAPS];

/* Hook-side state (talker task only). */
static int32_t *s_raw;
static uint32_t s_raw_w;
static int32_t s_mic_line[AEC_MIC_CAP];
static uint32_t s_mic_w;
static uint32_t s_mic_lead; /* samples the mic waits behind the reference */
static uint32_t s_delay;    /* raw-channel delay = cleaned-channel latency */
static uint64_t s_in_count;
static bool s_started;
static uint32_t s_owed;   /* outputs that will still arrive late: skip them */
static uint32_t s_never;  /* inputs the hook dropped: their outputs never come */
static float s_dc_x1, s_dc_y1;

/* Stats. */
static uint32_t s_in_overflow, s_out_underrun, s_blocks, s_max_us;
static double s_e_mic, s_e_ref, s_e_out;

static inline int32_t be24(const uint8_t *p) {
  return ((int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                    ((uint32_t)p[2] << 8))) >> 8;
}

static inline void put_be24(uint8_t *p, int32_t v) {
  if (v > 8388607)
    v = 8388607;
  else if (v < -8388608)
    v = -8388608;
  p[0] = (uint8_t)(v >> 16);
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)v;
  p[3] = 0;
}

static inline int16_t to16(float v) {
  if (v > 32767.f)
    return 32767;
  if (v < -32768.f)
    return -32768;
  return (int16_t)lrintf(v);
}

/* Windowed-sinc low-pass for the 3:1 converters: cutoff 7.2 kHz at
 * 48 kHz (passes speech to 7 kHz, stops before 8 kHz), Blackman. */
static void fir_design(void) {
  const float fc = 7200.f / AEC_RATE_IN;
  const int mid = AEC_FIR_TAPS / 2;
  float sum = 0.f;
  for (int i = 0; i < AEC_FIR_TAPS; i++) {
    int n = i - mid;
    float sinc = n == 0 ? 2.f * fc : sinf(2.f * (float)M_PI * fc * n) / ((float)M_PI * n);
    float w = 0.42f - 0.5f * cosf(2.f * (float)M_PI * i / (AEC_FIR_TAPS - 1)) +
              0.08f * cosf(4.f * (float)M_PI * i / (AEC_FIR_TAPS - 1));
    s_fir[i] = sinc * w;
    sum += s_fir[i];
  }
  for (int i = 0; i < AEC_FIR_TAPS; i++)
    s_fir[i] /= sum;
}

/* 48 -> 16 kHz: one output per AEC_DECIM inputs. */
typedef struct {
  float hist[AEC_FIR_TAPS];
  int pos;   /* next write slot */
  int phase; /* inputs since the last output */
} decim_t;

static bool decim_push(decim_t *d, float x, float *y) {
  d->hist[d->pos] = x;
  d->pos = (d->pos + 1) % AEC_FIR_TAPS;
  if (++d->phase < AEC_DECIM)
    return false;
  d->phase = 0;
  float acc = 0.f;
  int k = d->pos; /* oldest */
  for (int i = AEC_FIR_TAPS - 1; i >= 0; i--) {
    acc += s_fir[i] * d->hist[k];
    k = (k + 1) % AEC_FIR_TAPS;
  }
  *y = acc;
  return true;
}

/* 16 -> 48 kHz: zero-stuff by AEC_DECIM, the same low-pass, gain 3. */
typedef struct {
  float hist[AEC_FIR_PHASE_TAPS];
  int pos;
} interp_t;

static void interp_push(interp_t *t, float x, float out[AEC_DECIM]) {
  t->hist[t->pos] = x;
  for (int p = 0; p < AEC_DECIM; p++) {
    float acc = 0.f;
    int k = t->pos; /* newest */
    for (int j = 0; p + AEC_DECIM * j < AEC_FIR_TAPS; j++) {
      acc += s_fir[p + AEC_DECIM * j] * t->hist[k];
      k = (k + AEC_FIR_PHASE_TAPS - 1) % AEC_FIR_PHASE_TAPS;
    }
    out[p] = acc * AEC_DECIM;
  }
  t->pos = (t->pos + 1) % AEC_FIR_PHASE_TAPS;
}

/* Converter round trip, in 48 kHz samples: an impulse through fresh
 * copies of both, peak position of the output. */
static int converter_delay(void) {
  decim_t d = {0};
  interp_t t = {0};
  float best = 0.f;
  int best_at = 0, n_out = 0;
  for (int n = 0; n < 6 * AEC_FIR_TAPS; n++) {
    float y;
    if (decim_push(&d, n == 0 ? 1.f : 0.f, &y)) {
      float o[AEC_DECIM];
      interp_push(&t, y, o);
      for (int p = 0; p < AEC_DECIM; p++, n_out++) {
        if (fabsf(o[p]) > best) {
          best = fabsf(o[p]);
          best_at = n_out;
        }
      }
    }
  }
  /* Output sample n_out leaves when input 3*(n_out/3)+2 arrives: the
   * stream positions line up once the hook pops at the input rate. */
  return best_at;
}

static char s_mode_name[32] = "?";
#define AEC_FRAME_MAX 2048 /* 16 kHz samples; ESP-SR frames are 256 or 512 */
static uint32_t s_dropped_blocks, s_sum_us;

static aec_handle_t *aec_make(aec_mode_t mode, int filter) {
  aec_config_t cfg = {
      .mic_num = 1,
      .ref_num = 1,
      .out_num = 1,
      .filter_length = filter,
      .sample_rate = AEC_RATE,
      .caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
      .mode = mode,
      .nlp_level = AEC_NLP_LEVEL_AGGR,
  };
  aec_handle_t *h = aec_create_from_config(&cfg);
  if (h)
    snprintf(s_mode_name, sizeof(s_mode_name), "%s/f%d",
             aec_get_mode_string(mode), filter);
  return h;
}

#if CONFIG_EXAMPLE_AVB_AEC_BENCH_CYCLE
/* Silent benchmark: each configuration runs 40 s (four stats lines) on
 * the self-test signal, then the next one takes over. Stream alignment
 * is not kept across a change; the stats are what counts. */
static const struct {
  aec_mode_t mode;
  int filter;
} s_bench[] = {
    {AEC_MODE_FD_HIGH_PERF, 4},   {AEC_MODE_FD_LOW_COST, 4},
    {AEC_MODE_VOIP_HIGH_PERF, 4}, {AEC_MODE_VOIP_LOW_COST, 4},
    {AEC_MODE_FD_HIGH_PERF, 2},   {AEC_MODE_FD_LOW_COST, 2},
};
static int s_bench_i, s_bench_ticks;

static void bench_next(void) {
  if (++s_bench_ticks < 4)
    return;
  s_bench_ticks = 0;
  s_bench_i = (s_bench_i + 1) % (int)(sizeof(s_bench) / sizeof(s_bench[0]));
  aec_handle_t *old = s_aec;
  aec_handle_t *h = aec_make(s_bench[s_bench_i].mode, s_bench[s_bench_i].filter);
  if (!h) {
    ESP_LOGE(TAG, "bench: create failed");
    return;
  }
  s_aec = h;
  aec_destroy(old);
  s_chunk = aec_get_chunksize(s_aec);
  s_block = s_chunk * AEC_DECIM;
  s_dropped_blocks = 0;
  ESP_LOGI(TAG, "bench: now %s, frame %d samples", s_mode_name, s_chunk);
}
#endif

#if CONFIG_EXAMPLE_AVB_AEC_SELFTEST
/* Silent self-test: a speech-like far end (coloured noise in 100-200 ms
 * bursts and pauses) replaces the playout as the reference, and a
 * digital echo of it (-6 dB at 20 ms, -14 dB at 35 ms) is added to the
 * mic. Nothing reaches the speaker. Runs in the hook (IRAM). */
static uint32_t s_lcg = 1;
static float s_lp1, s_lp2, s_env, s_env_target;
static uint32_t s_syl_left;
static int32_t s_echo_line[2048];
static uint32_t s_echo_w;

static inline IRAM_ATTR int32_t selftest_far(void) {
  s_lcg = s_lcg * 1664525u + 1013904223u;
  float n = (float)(int32_t)s_lcg * (1.f / 2147483648.f);
  s_lp1 += 0.18f * (n - s_lp1);
  s_lp2 += 0.18f * (s_lp1 - s_lp2);
  if (s_syl_left == 0) {
    s_lcg = s_lcg * 1664525u + 1013904223u;
    uint32_t r = s_lcg >> 20; /* 0..4095 */
    bool on = s_env_target == 0.f;
    s_env_target = on ? 1.f : 0.f;
    s_syl_left = (on ? 4800u : 3600u) + r;
  }
  s_syl_left--;
  s_env += 0.002f * (s_env_target - s_env);
  return (int32_t)(s_lp2 * s_env * 8388607.f * 2.0f);
}
#endif

static int16_t *s_mic16, *s_ref16, *s_out16;
static decim_t s_dm, s_dr;
static interp_t s_up;

/* Push one 16 kHz output frame (s_chunk samples) through the 1:3
 * converter into the hook's queue. */
static void out_push(const int16_t *out16) {
  uint32_t oh = atomic_load_explicit(&s_out_head, memory_order_relaxed);
  uint32_t ot = atomic_load_explicit(&s_out_tail, memory_order_acquire);
  for (int i = 0; i < s_chunk; i++) {
    float o[AEC_DECIM];
    interp_push(&s_up, (float)out16[i], o);
    for (int p = 0; p < AEC_DECIM; p++) {
      if (oh - ot >= AEC_OUT_CAP)
        break; /* hook stalled; it re-aligns by skipping */
      s_out[oh & (AEC_OUT_CAP - 1)] = (int32_t)lrintf(o[p] * 256.f);
      oh++;
    }
  }
  atomic_store_explicit(&s_out_head, oh, memory_order_release);
}

/* Run the queued blocks, at most two per call so the stats and the rest
 * of core 0 keep running. If the canceller can't keep up (more than two
 * blocks still queued), drop the oldest whole blocks and send silence in
 * their place: the output keeps one sample per input sample, so the
 * channels stay aligned, and the drops are counted. */
static void aec_run_ready(void) {
  int16_t *mic16 = s_mic16, *ref16 = s_ref16, *out16 = s_out16;
  for (int pass = 0; pass < 2; pass++) {
    uint32_t tail = atomic_load_explicit(&s_in_tail, memory_order_relaxed);
    uint32_t head = atomic_load_explicit(&s_in_head, memory_order_acquire);
    if (head - tail < (uint32_t)s_block)
      return;
    int got = 0;
    for (int i = 0; i < s_block; i++) {
      aec_pair_t pr = s_in[(tail + i) & (AEC_IN_CAP - 1)];
      float ym, yr;
      /* 24-bit to the AEC's 16-bit scale. */
      bool a = decim_push(&s_dm, pr.mic / 256.f, &ym);
      bool b = decim_push(&s_dr, pr.ref / 256.f, &yr);
      if (a && b && got < s_chunk) {
        mic16[got] = to16(ym);
        ref16[got] = to16(yr);
        got++;
      }
    }
    atomic_store_explicit(&s_in_tail, tail + s_block, memory_order_release);
    int64_t t0 = esp_timer_get_time();
    aec_process(s_aec, mic16, ref16, out16);
    uint32_t took = (uint32_t)(esp_timer_get_time() - t0);
    if (took > s_max_us)
      s_max_us = took;
    s_sum_us += took;
    s_blocks++;
    for (int i = 0; i < s_chunk; i++) {
      s_e_mic += (double)mic16[i] * mic16[i];
      s_e_ref += (double)ref16[i] * ref16[i];
      s_e_out += (double)out16[i] * out16[i];
    }
    out_push(out16);
  }
  uint32_t tail = atomic_load_explicit(&s_in_tail, memory_order_relaxed);
  uint32_t head = atomic_load_explicit(&s_in_head, memory_order_acquire);
  while (head - tail >= 3u * (uint32_t)s_block) {
    tail += s_block;
    memset(out16, 0, s_chunk * sizeof(int16_t));
    out_push(out16);
    s_dropped_blocks++;
  }
  atomic_store_explicit(&s_in_tail, tail, memory_order_release);
}

static void aec_task(void *arg) {
  (void)arg;
  int64_t next_log = esp_timer_get_time() + 10000000;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    aec_run_ready();
    int64_t now = esp_timer_get_time();
    if (now >= next_log) {
      next_log = now + 10000000;
      double n = (double)s_blocks * s_chunk;
      if (n < 1)
        n = 1;
      float mic_db = 10.f * log10f((float)(s_e_mic / n) + 1e-3f) - 90.3f;
      float ref_db = 10.f * log10f((float)(s_e_ref / n) + 1e-3f) - 90.3f;
      float out_db = 10.f * log10f((float)(s_e_out / n) + 1e-3f) - 90.3f;
      ESP_LOGI(TAG,
               "%s blocks=%lu avg=%luus max=%luus (frame %dus) mic=%.1f "
               "ref=%.1f out=%.1f dBFS erle=%.1f dB dropped=%lu in_over=%lu "
               "out_under=%lu owed=%lu delay=%lu",
               s_mode_name, (unsigned long)s_blocks,
               (unsigned long)(s_blocks ? s_sum_us / s_blocks : 0),
               (unsigned long)s_max_us, s_chunk * 1000 / 16, mic_db, ref_db,
               out_db, mic_db - out_db, (unsigned long)s_dropped_blocks,
               (unsigned long)s_in_overflow, (unsigned long)s_out_underrun,
               (unsigned long)s_owed, (unsigned long)s_delay);
      s_blocks = 0;
      s_max_us = 0;
      s_sum_us = 0;
#if CONFIG_EXAMPLE_AVB_AEC_BENCH_CYCLE
      bench_next();
#endif
      s_e_mic = s_e_ref = s_e_out = 0;
    }
  }
}

/* Runs in the talker task once per packet: kept in IRAM and free of
 * library calls, so it adds no cache misses to the send path. */
IRAM_ATTR void avb_talker_capture_hook(uint8_t *capture, const uint8_t *playout,
                                       int frames) {
  if (!s_ready)
    return;
  uint32_t head = atomic_load_explicit(&s_in_head, memory_order_relaxed);
  uint32_t tail = atomic_load_explicit(&s_in_tail, memory_order_acquire);
  uint32_t ot = atomic_load_explicit(&s_out_tail, memory_order_relaxed);
  uint32_t oh = atomic_load_explicit(&s_out_head, memory_order_acquire);
  for (int f = 0; f < frames; f++) {
    uint8_t *cap = capture + f * 8;
    const uint8_t *ply = playout + f * 8;
    /* DC block (one pole at ~8 Hz) on the mic; both channels use it. */
    float x = (float)be24(cap);
    float y = x - s_dc_x1 + 0.999f * s_dc_y1;
    s_dc_x1 = x;
    s_dc_y1 = y;
    int32_t mic = (int32_t)(y >= 0.f ? y + 0.5f : y - 0.5f);
    int32_t ref = (be24(ply) + be24(ply + 4)) / 2;
#if CONFIG_EXAMPLE_AVB_AEC_SELFTEST
    int32_t far = selftest_far();
    s_echo_line[s_echo_w & 2047] = far;
    mic += s_echo_line[(s_echo_w - 960) & 2047] / 2 +
           s_echo_line[(s_echo_w - 1680) & 2047] / 5;
    s_echo_w++;
    ref = far;
#endif
    /* The mic waits s_mic_lead samples so the reference leads its echo. */
    s_mic_line[s_mic_w & (AEC_MIC_CAP - 1)] = mic;
    int32_t mic_late = s_mic_line[(s_mic_w - s_mic_lead) & (AEC_MIC_CAP - 1)];
    s_mic_w++;
    if (head - tail < AEC_IN_CAP) {
      s_in[head & (AEC_IN_CAP - 1)] = (aec_pair_t){mic_late, ref};
      head++;
    } else {
      /* The task is starved: this input never produces an output, so
       * cancel one late output we're waiting for, or remember it. */
      s_in_overflow++;
      if (s_owed)
        s_owed--;
      else
        s_never++;
    }
    /* Raw channel: the same mic, delayed to line up with the cleaned one. */
    s_raw[s_raw_w & (AEC_RAW_CAP - 1)] = mic;
    int32_t raw = s_raw[(s_raw_w - s_delay) & (AEC_RAW_CAP - 1)];
    s_raw_w++;
    int32_t clean = 0;
    if (!s_started && s_in_count >= (uint64_t)AEC_START_BLOCKS * s_block)
      s_started = true;
    s_in_count++;
    if (s_started) {
      while (s_owed > 0 && oh != ot) {
        ot++; /* make up a late block: keep the alignment */
        s_owed--;
      }
      if (oh != ot) {
        clean = s_out[ot & (AEC_OUT_CAP - 1)];
        ot++;
      } else {
        s_out_underrun++;
        if (s_never)
          s_never--; /* the output of a dropped input: nothing to skip */
        else
          s_owed++;
      }
    }
    put_be24(cap, clean);
    put_be24(cap + 4, raw);
  }
  atomic_store_explicit(&s_in_head, head, memory_order_release);
  atomic_store_explicit(&s_out_tail, ot, memory_order_release);
  if (head - tail >= (uint32_t)s_block && s_task)
    xTaskNotifyGive(s_task);
}

void avb_aec_start(void) {
  aec_mode_t mode = AEC_MODE_FD_HIGH_PERF;
#if CONFIG_EXAMPLE_AVB_AEC_MODE_FD_LOW
  mode = AEC_MODE_FD_LOW_COST;
#elif CONFIG_EXAMPLE_AVB_AEC_MODE_VOIP
  mode = AEC_MODE_VOIP_HIGH_PERF;
#elif CONFIG_EXAMPLE_AVB_AEC_MODE_VOIP_LOW
  mode = AEC_MODE_VOIP_LOW_COST;
#endif
  /* ESP-SR's canceller takes ~290 KB and hard-codes PSRAM for part of
   * it (its NLP init faults on the NULL without PSRAM); in internal RAM it
   * leaves too little for AVB. So it runs only when PSRAM is present. */
  size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  ESP_LOGI(TAG, "before the canceller: internal %u free, PSRAM %u total",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)psram);
  if (psram == 0) {
    ESP_LOGE(TAG, "no PSRAM: canceller off, capture passes through");
    return;
  }
  s_aec = aec_make(mode, CONFIG_EXAMPLE_AVB_AEC_FILTER_LENGTH);
  if (!s_aec) {
    ESP_LOGE(TAG, "aec_create failed; capture passes through");
    return;
  }
  s_chunk = aec_get_chunksize(s_aec);
  s_block = s_chunk * AEC_DECIM;
  s_in = heap_caps_calloc(AEC_IN_CAP, sizeof(aec_pair_t), MALLOC_CAP_DEFAULT);
  s_out = heap_caps_calloc(AEC_OUT_CAP, sizeof(int32_t), MALLOC_CAP_DEFAULT);
  s_raw = heap_caps_calloc(AEC_RAW_CAP, sizeof(int32_t), MALLOC_CAP_DEFAULT);
  /* ESP-SR wants 16-byte aligned frames (vector loads). */
  const uint32_t frame_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  s_mic16 = heap_caps_aligned_alloc(16, AEC_FRAME_MAX * sizeof(int16_t), frame_caps);
  s_ref16 = heap_caps_aligned_alloc(16, AEC_FRAME_MAX * sizeof(int16_t), frame_caps);
  s_out16 = heap_caps_aligned_alloc(16, AEC_FRAME_MAX * sizeof(int16_t), frame_caps);
  if (!s_in || !s_out || !s_raw || !s_mic16 || !s_ref16 || !s_out16 ||
      s_chunk > AEC_FRAME_MAX || s_block * (AEC_START_BLOCKS + 2) > AEC_OUT_CAP) {
    ESP_LOGE(TAG, "no memory for the AEC buffers; capture passes through");
    return;
  }
  fir_design();
  s_mic_lead = CONFIG_EXAMPLE_AVB_AEC_MIC_LEAD_MS * (AEC_RATE_IN / 1000);
  if (s_mic_lead >= AEC_MIC_CAP)
    s_mic_lead = AEC_MIC_CAP - 1;
  int conv = converter_delay();
  /* ESP-SR's canceller returns each frame one frame late (measured: the
   * cleaned channel lagged the raw one by exactly 512 samples at 16 kHz,
   * cross-correlation on room noise), so the raw channel waits one more
   * frame. */
  s_delay = AEC_START_BLOCKS * s_block + conv + s_mic_lead + s_block;
  if (s_delay >= AEC_RAW_CAP)
    s_delay = AEC_RAW_CAP - 1;
  if (xTaskCreatePinnedToCore(aec_task, "avb_aec", 6144, NULL, 5, &s_task, 0) !=
      pdPASS) {
    ESP_LOGE(TAG, "no AEC task; capture passes through");
    return;
  }
  s_ready = true;
  ESP_LOGI(TAG, "after: internal %u free, PSRAM %u free",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  ESP_LOGI(TAG,
           "%s, frame %d samples at 16 kHz; ch0 cleaned, ch1 raw; "
           "both delayed %lu samples (start %d + converters %d + mic lead %lu "
           "+ canceller frame %d)",
           s_mode_name, s_chunk, (unsigned long)s_delay, AEC_START_BLOCKS * s_block, conv,
           (unsigned long)s_mic_lead, s_block);
}
