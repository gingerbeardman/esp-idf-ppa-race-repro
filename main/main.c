/* Minimal reproducer for espressif/esp-idf PR #19047:
 * esp_driver_ppa strands a transaction submitted from a core that does not
 * run the PPA/2D-DMA interrupt.
 *
 * Needs an ESP32-P4 with PSRAM. No display. Register the SRM client on core
 * 0 (the interrupt is allocated on the registering core), then:
 *   phase 1: a task submits single NON_BLOCKING SRM copies spaced at the
 *            transaction's own measured duration ±60 us, so that some
 *            submissions land inside the completion interrupt of the
 *            previous one. Each carries its index in user_data; the done
 *            callback counts completions that arrive out of index order and
 *            their latency. A stranded transaction completes only after the
 *            NEXT submission ran: out of order and ~3 durations late.
 *   phase 2: pairs — submit A, spin one duration, submit B, then wait for
 *            both with a timeout. If B was stranded nothing follows to
 *            rescue it: the wait times out. That is the application hang.
 * Both phases run from a core-1 task, then again from core 0 as the control.
 *
 * Stock IDF v5.5.5 / master, ESP32-P4 v1.3:
 *   core 1: ~130 out-of-order per 20000 in phase 1, several timeouts in phase 2
 *   core 0: 0 and 0
 * With the fix: 0 and 0 on both cores.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_cpu.h"
#include "driver/ppa.h"

static const char *TAG = "ppa_race";
#define W 480
#define H 800
#define BLOCK_H 40
#define N1 20000
#define N2 2000

static ppa_client_handle_t s_ppa;
static SemaphoreHandle_t s_done;
static uint16_t *s_in, *s_out;
static volatile int s_prev, s_inv, s_late, s_on;
static volatile int64_t s_thr, s_maxlat;
static int64_t *s_sub_us;   /* PSRAM, N1 entries */

static bool done_cb(ppa_client_handle_t h, ppa_event_data_t *e, void *user_data)
{
    BaseType_t woken = pdFALSE;
    if (s_on) {
        int idx = (int)(uintptr_t)user_data;
        if (idx >= 0 && idx < N1) {
            int64_t lat = esp_timer_get_time() - s_sub_us[idx];
            if (lat > s_maxlat) s_maxlat = lat;
            if (lat > s_thr) s_late++;
        }
        if (idx < s_prev) s_inv++;
        s_prev = idx;
    }
    xSemaphoreGiveFromISR(s_done, &woken);
    return woken == pdTRUE;
}

static void cfg(ppa_srm_oper_config_t *c, int idx)
{
    memset(c, 0, sizeof *c);
    c->in.buffer = s_in;  c->in.pic_w = W;  c->in.pic_h = H;
    c->in.block_w = W;    c->in.block_h = BLOCK_H;
    c->in.block_offset_x = 0; c->in.block_offset_y = 0;
    c->in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    c->out.buffer = s_out; c->out.buffer_size = W * H * 2;
    c->out.pic_w = W; c->out.pic_h = H;
    c->out.block_offset_x = 0; c->out.block_offset_y = 0;
    c->out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    c->rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    c->scale_x = 1.0f; c->scale_y = 1.0f;
    c->mode = PPA_TRANS_MODE_NON_BLOCKING;
    c->user_data = (void *)(uintptr_t)idx;
}

static void run_phases(const char *who)
{
    ppa_srm_oper_config_t c;
    while (xSemaphoreTake(s_done, 0) == pdTRUE) {}

    /* calibrate: sequential */
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < 64; i++) {
        cfg(&c, i);
        ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(s_ppa, &c));
        xSemaphoreTake(s_done, portMAX_DELAY);
    }
    int64_t dur = (esp_timer_get_time() - t0) / 64;

    /* phase 1: ordering */
    s_prev = -1; s_inv = 0; s_late = 0; s_maxlat = 0; s_thr = dur * 26 / 10; s_on = 1;
    uint32_t seed = (uint32_t)esp_timer_get_time();
    int taken = 0;
    for (int i = 0; i < N1; i++) {
        while (i - taken >= 3) { xSemaphoreTake(s_done, portMAX_DELAY); taken++; }
        cfg(&c, i);
        int64_t ts = esp_timer_get_time();
        s_sub_us[i] = ts;
        ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(s_ppa, &c));
        seed = seed * 1103515245u + 12345u;
        int64_t spin = dur - 60 + (int64_t)((seed >> 16) % 120);
        while (esp_timer_get_time() - ts < spin) {}
        while (xSemaphoreTake(s_done, 0) == pdTRUE) taken++;
        if ((i & 31) == 31) vTaskDelay(1);
    }
    while (taken < N1) { xSemaphoreTake(s_done, portMAX_DELAY); taken++; }
    s_on = 0;
    ESP_LOGI(TAG, "%s phase 1: transaction %lld us | %d of %d completed OUT OF ORDER | %d completed > 2.6x late | max latency %lld us",
             who, (long long)dur, s_inv, N1, s_late, (long long)s_maxlat);

    /* phase 2: pairs, the second one has nothing after it to rescue it */
    int hangs = 0;
    for (int i = 0; i < N2; i++) {
        cfg(&c, 0);
        int64_t ts = esp_timer_get_time();
        ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(s_ppa, &c));
        seed = seed * 1103515245u + 12345u;
        int64_t spin = dur - 60 + (int64_t)((seed >> 16) % 120);
        while (esp_timer_get_time() - ts < spin) {}
        cfg(&c, 1);
        ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(s_ppa, &c));
        int got = 0;
        for (int k = 0; k < 2; k++)
            if (xSemaphoreTake(s_done, pdMS_TO_TICKS(500)) == pdTRUE) got++;
        if (got < 2) {
            hangs++;
            /* stranded: an application would wait here forever. Rescue it
             * with one more submission so the demo can go on. */
            cfg(&c, 2);
            ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(s_ppa, &c));
            while (got < 3) { xSemaphoreTake(s_done, portMAX_DELAY); got++; }
        }
        if ((i & 15) == 15) vTaskDelay(1);
    }
    ESP_LOGI(TAG, "%s phase 2: %d of %d pairs where the second transaction NEVER completed within 500 ms (application hang)",
             who, hangs, N2);
}

static volatile int s_core1_done;
static void core1_task(void *arg)
{
    run_phases("core 1 submitter (PPA ISR on core 0):");
    s_core1_done = 1;
    vTaskDelete(NULL);
}

void app_main(void)
{
    s_in = heap_caps_aligned_calloc(64, 1, W * H * 2, MALLOC_CAP_SPIRAM);
    s_out = heap_caps_aligned_calloc(64, 1, W * H * 2, MALLOC_CAP_SPIRAM);
    s_sub_us = heap_caps_calloc(N1, sizeof(int64_t), MALLOC_CAP_SPIRAM);
    assert(s_in && s_out && s_sub_us);
    s_done = xSemaphoreCreateCounting(8, 0);
    /* registered on core 0: the 2D-DMA interrupts are allocated here */
    ppa_client_config_t pc = { .oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 4 };
    ESP_ERROR_CHECK(ppa_register_client(&pc, &s_ppa));
    ppa_event_callbacks_t cbs = { .on_trans_done = done_cb };
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(s_ppa, &cbs));
    ESP_LOGI(TAG, "app_main on core %d; SRM client registered here", (int)esp_cpu_get_core_id());

    xTaskCreatePinnedToCore(core1_task, "core1", 8192, NULL, 5, NULL, 1);
    while (!s_core1_done) vTaskDelay(pdMS_TO_TICKS(50));

    run_phases("core 0 submitter (control):        ");
    ESP_LOGI(TAG, "done");
}
