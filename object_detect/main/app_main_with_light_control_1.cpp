/**
 * @file app_main_with_light_control.cpp
 * @brief Person Detection + Brightness-based LED control + WS2812 RGB LED + Person trigger GPIO
 *
 * Detection logic:
 *   - Every FRAME_SAMPLE_RATE frames, enter a 2-frame sampling window.
 *   - Run the model on both consecutive frames in the window.
 *   - If BOTH frames detect a person  → GPIO_NUM_47 HIGH (stays HIGH).
 *   - If either frame misses a person → GPIO_NUM_47 LOW  (stays LOW).
 *   - No timer, no auto-reset — pin directly mirrors the confirmed result.
 *
 * False detection filter:
 *   - If the model returns a bounding box of exactly [0,0,239,239], it is
 *     treated as a false detection and ignored (person_now = false).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"

#include "frame_cap_pipeline.hpp"
#include "light_controller.hpp"
#include "who_frame_cap.hpp"
#include "img_converters.h"
#include "pedestrian_detect.hpp"

static const char *TAG = "MAIN";

// ── Feature enable / disable switches ────────────────────────────────────────
#define ENABLE_RGB_LED              1
#define ENABLE_SD_SAVE              1
#define ENABLE_FILL_LIGHT           1
#define ENABLE_BLINK_TEST           0
#define ENABLE_PERSON_TRIGGER_GPIO  1
// ─────────────────────────────────────────────────────────────────────────────

// ── Tuning parameters ────────────────────────────────────────────────────────
#define LIGHT_GPIO                  21
#define BRIGHTNESS_THRESHOLD        100
#define RGB_GPIO                    GPIO_NUM_48
#define PERSON_TRIGGER_GPIO         GPIO_NUM_47
#define FRAME_SAMPLE_RATE           7
#define SCORE_THRESHOLD             0.6f
#define SAVE_COOLDOWN_MS            3000
#define SD_MAX_USAGE_MB             12000
#define SD_DELETE_BATCH             10
#define MOUNT_POINT                 CONFIG_BSP_SD_MOUNT_POINT
#define SAVE_DIR                    MOUNT_POINT "/detections"
#define RESULT_CSV                  SAVE_DIR "/log.csv"
const int monitor_time = 3000; // ms to keep GPIO high after detection (extends on each new detection)
// ─────────────────────────────────────────────────────────────────────────────

// ── Start time ───────────────────────────────────────────────────────────────
#define START_YEAR   2026
#define START_MONTH  5
#define START_DAY    17
#define START_HOUR   10
#define START_MIN    0
#define START_SEC    0
// ─────────────────────────────────────────────────────────────────────────────

// ── Time feature ─────────────────────────────────────────────────────────────
static time_t s_boot_time = 0;

static void time_feature_init(void)
{
    struct tm t = {};
    t.tm_year = START_YEAR - 1900;
    t.tm_mon  = START_MONTH - 1;
    t.tm_mday = START_DAY;
    t.tm_hour = START_HOUR;
    t.tm_min  = START_MIN;
    t.tm_sec  = START_SEC;

    s_boot_time = mktime(&t);

    ESP_LOGI(TAG, "Boot time set: %04d-%02d-%02d %02d:%02d:%02d",
             START_YEAR, START_MONTH, START_DAY,
             START_HOUR, START_MIN, START_SEC);
}

static void get_timestamp_str(char *buf, size_t buf_size)
{
    uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
    time_t now = s_boot_time + (time_t)elapsed_sec;
    struct tm *t = localtime(&now);
    strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", t);
}

static void get_timestamp_filename(char *buf, size_t buf_size)
{
    uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
    time_t now = s_boot_time + (time_t)elapsed_sec;
    struct tm *t = localtime(&now);
    strftime(buf, buf_size, "%Y%m%d_%H%M%S", t);
}
// ─────────────────────────────────────────────────────────────────────────────

// ── RGB LED feature ──────────────────────────────────────────────────────────
#if ENABLE_RGB_LED

#define WS2812_T0H_NS  350
#define WS2812_T0L_NS  800
#define WS2812_T1H_NS  700
#define WS2812_T1L_NS  600

static rmt_channel_handle_t s_rmt_chan = nullptr;
static rmt_encoder_handle_t s_rmt_encoder = nullptr;

static void rgb_feature_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {};
    chan_cfg.gpio_num = RGB_GPIO;
    chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    chan_cfg.resolution_hz = 10 * 1000 * 1000;
    chan_cfg.mem_block_symbols = 64;
    chan_cfg.trans_queue_depth = 4;

    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_rmt_chan));

    rmt_bytes_encoder_config_t enc_cfg = {};
    enc_cfg.bit0.duration0 = (uint16_t)(WS2812_T0H_NS / 100);
    enc_cfg.bit0.level0 = 1;
    enc_cfg.bit0.duration1 = (uint16_t)(WS2812_T0L_NS / 100);
    enc_cfg.bit0.level1 = 0;
    enc_cfg.bit1.duration0 = (uint16_t)(WS2812_T1H_NS / 100);
    enc_cfg.bit1.level0 = 1;
    enc_cfg.bit1.duration1 = (uint16_t)(WS2812_T1L_NS / 100);
    enc_cfg.bit1.level1 = 0;
    enc_cfg.flags.msb_first = 1;

    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_rmt_encoder));
    ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
}

static void rgb_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = { g, r, b };

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;

    rmt_transmit(s_rmt_chan, s_rmt_encoder, grb, sizeof(grb), &tx_cfg);
    rmt_tx_wait_all_done(s_rmt_chan, pdMS_TO_TICKS(100));
}

static void rgb_no_person(void)
{
    rgb_set_color(0, 0, 0);
}

static void rgb_person_detected(void)
{
    rgb_set_color(0, 0, 255);
}

#else

static void rgb_feature_init(void) {}
static void rgb_no_person(void) {}
static void rgb_person_detected(void) {}

#endif
// ─────────────────────────────────────────────────────────────────────────────

// ── Person trigger GPIO feature ──────────────────────────────────────────────
// NOTE: No timer logic. The pin is set HIGH or LOW directly based on whether
//       both frames in the 2-frame window confirmed a person detection.
#if ENABLE_PERSON_TRIGGER_GPIO

static void person_trigger_gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PERSON_TRIGGER_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(PERSON_TRIGGER_GPIO, 0);

    ESP_LOGI(TAG, "Person trigger GPIO enabled: GPIO %d (direct HIGH/LOW, no timer)",
             (int)PERSON_TRIGGER_GPIO);
}

static void person_trigger_gpio_set(bool high)
{
    gpio_set_level(PERSON_TRIGGER_GPIO, high ? 1 : 0);
}

#else

static void person_trigger_gpio_init(void) {}
static void person_trigger_gpio_set(bool) {}

#endif
// ─────────────────────────────────────────────────────────────────────────────

// ── Fill light feature ───────────────────────────────────────────────────────
static LightController *light_feature_init(void)
{
#if ENABLE_FILL_LIGHT
    return new LightController((gpio_num_t)LIGHT_GPIO, BRIGHTNESS_THRESHOLD);
#else
    return nullptr;
#endif
}

static void light_feature_blink_test(LightController *light_ctrl)
{
#if ENABLE_FILL_LIGHT && ENABLE_BLINK_TEST
    if (!light_ctrl) return;

    for (int i = 0; i < 3; i++) {
        light_ctrl->set_light(true);
        vTaskDelay(pdMS_TO_TICKS(300));
        light_ctrl->set_light(false);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
#else
    (void)light_ctrl;
#endif
}

static void light_feature_update(LightController *light_ctrl, dl::image::img_t &img)
{
#if ENABLE_FILL_LIGHT
    if (light_ctrl) {
        light_ctrl->update_light(img);
    }
#else
    (void)light_ctrl;
    (void)img;
#endif
}

static int light_feature_get_brightness(LightController *light_ctrl)
{
#if ENABLE_FILL_LIGHT
    return light_ctrl ? light_ctrl->get_last_brightness() : -1;
#else
    (void)light_ctrl;
    return -1;
#endif
}

static const char *light_feature_get_state(LightController *light_ctrl)
{
#if ENABLE_FILL_LIGHT
    if (!light_ctrl) return "DISABLED";
    return light_ctrl->is_light_on() ? "ON" : "OFF";
#else
    (void)light_ctrl;
    return "DISABLED";
#endif
}
// ─────────────────────────────────────────────────────────────────────────────

// ── SD save feature ──────────────────────────────────────────────────────────
#if ENABLE_SD_SAVE

static bool s_sd_ok = false;
static int s_img_counter = 0;
static char s_last_timestamp[32] = {0};

static void load_state()
{
    FILE *f = fopen(SAVE_DIR "/state.txt", "r");
    if (!f) {
        ESP_LOGW(TAG, "No state file, starting fresh");
        return;
    }

    fscanf(f, "%31[^,],%d", s_last_timestamp, &s_img_counter);
    fclose(f);

    ESP_LOGI(TAG, "Loaded state: %s | %d", s_last_timestamp, s_img_counter);
}

static void save_state()
{
    FILE *f = fopen(SAVE_DIR "/state.txt", "w");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open state file!");
        return;
    }

    fprintf(f, "%s,%d", s_last_timestamp, s_img_counter);

    fflush(f);
    fsync(fileno(f));

    fclose(f);
}

static uint32_t get_save_dir_size_mb(void)
{
    DIR *dir = opendir(SAVE_DIR);
    if (!dir) return 0;

    uint64_t total_bytes = 0;
    struct dirent *entry;
    struct stat st;
    char path[1024];

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type != DT_REG) continue;

        snprintf(path, sizeof(path), "%s/%s", SAVE_DIR, entry->d_name);

        if (stat(path, &st) == 0) {
            total_bytes += st.st_size;
        }
    }

    closedir(dir);
    return (uint32_t)(total_bytes / (1024 * 1024));
}

static void delete_oldest_files(void)
{
    DIR *dir = opendir(SAVE_DIR);
    if (!dir) return;

    char files[100][256];
    int count = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL && count < 100) {
        if (entry->d_type != DT_REG) continue;

        size_t len = strlen(entry->d_name);
        if (len < 4) continue;
        if (strcasecmp(entry->d_name + len - 4, ".jpg") != 0) continue;

        snprintf(files[count], sizeof(files[count]), "%s", entry->d_name);
        count++;
    }

    closedir(dir);

    if (count == 0) return;

    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (strcmp(files[i], files[j]) > 0) {
                char tmp[256];
                memcpy(tmp, files[i], sizeof(tmp));
                memcpy(files[i], files[j], sizeof(files[i]));
                memcpy(files[j], tmp, sizeof(files[j]));
            }
        }
    }

    int to_delete = (count < SD_DELETE_BATCH) ? count : SD_DELETE_BATCH;
    char path[1024];

    for (int i = 0; i < to_delete; i++) {
        snprintf(path, sizeof(path), "%s/%.*s",
                 SAVE_DIR,
                 (int)sizeof(files[i]) - 1,
                 files[i]);

        if (remove(path) == 0) {
            ESP_LOGW(TAG, "Deleted old file: %s", files[i]);
        }
    }

    ESP_LOGW(TAG, "Deleted %d oldest files to free space", to_delete);
}

static void sd_feature_init(void)
{
    if (bsp_sdcard_mount() == ESP_OK) {
        s_sd_ok = true;

        struct stat st;
        if (stat(SAVE_DIR, &st) != 0) {
            mkdir(SAVE_DIR, 0755);
        }
        load_state();

        if (strlen(s_last_timestamp) == 0) {
            get_timestamp_filename(s_last_timestamp, sizeof(s_last_timestamp));
        }
        ESP_LOGI(TAG, "Restored image counter: %d", s_img_counter);

        FILE *f = fopen(RESULT_CSV, "r");
        if (!f) {
            f = fopen(RESULT_CSV, "w");
            if (f) {
                fprintf(f, "index,datetime,filename,score,x1,y1,x2,y2\n");
                fclose(f);
            }
        } else {
            fclose(f);
        }

        uint32_t used_mb = get_save_dir_size_mb();

        ESP_LOGI(TAG, "SD ready: %s | Used: %lu MB / %d MB max",
                 SAVE_DIR, used_mb, SD_MAX_USAGE_MB);
    } else {
        ESP_LOGW(TAG, "SD not available - saving disabled");
    }
}

static bool who_format_to_pixformat(who::cam::cam_fb_fmt_t in, pixformat_t *out)
{
    if (!out) return false;

    switch (in) {
    case who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565:
        *out = PIXFORMAT_RGB565;
        return true;

    case who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB888:
        *out = PIXFORMAT_RGB888;
        return true;

    case who::cam::cam_fb_fmt_t::CAM_FB_FMT_JPEG:
        *out = PIXFORMAT_JPEG;
        return true;

    default:
        return false;
    }
}

static void save_detection(who::cam::cam_fb_t *fb, float score, int x1, int y1, int x2, int y2)
{
    if (!s_sd_ok || !fb || !fb->buf) return;

    uint32_t used_mb = get_save_dir_size_mb();
    if (used_mb >= SD_MAX_USAGE_MB) {
        ESP_LOGW(TAG, "SD usage %lu MB >= %d MB limit - deleting oldest files...",
                 used_mb, SD_MAX_USAGE_MB);
        delete_oldest_files();
    }

    char ts_str[24];
    char ts_file[32];

    get_timestamp_str(ts_str, sizeof(ts_str));
    snprintf(ts_file, sizeof(ts_file), "%s", s_last_timestamp);

    char img_path[96];
    snprintf(img_path, sizeof(img_path), "%s/det_%s_%04d.jpg",
             SAVE_DIR, ts_file, s_img_counter);

    bool image_saved = false;

    FILE *f = fopen(img_path, "wb");
    if (f) {
        if (fb->format == who::cam::cam_fb_fmt_t::CAM_FB_FMT_JPEG) {
            size_t written = fwrite(fb->buf, 1, fb->len, f);
            image_saved = (written == fb->len);
        } else {
            pixformat_t pix_format;

            if (who_format_to_pixformat(fb->format, &pix_format)) {
                uint8_t *jpg_buf = NULL;
                size_t jpg_len = 0;

                bool converted = fmt2jpg(
                    (uint8_t *)fb->buf,
                    fb->len,
                    fb->width,
                    fb->height,
                    pix_format,
                    70,
                    &jpg_buf,
                    &jpg_len
                );

                if (converted && jpg_buf && jpg_len > 0) {
                    size_t written = fwrite(jpg_buf, 1, jpg_len, f);
                    image_saved = (written == jpg_len);
                    free(jpg_buf);
                } else {
                    ESP_LOGE(TAG, "JPEG conversion failed: %dx%d len=%u",
                             fb->width, fb->height, (unsigned)fb->len);
                }
            } else {
                ESP_LOGE(TAG, "Unsupported frame format");
            }
        }

        fclose(f);
    }

    if (!image_saved) {
        ESP_LOGE(TAG, "Failed to save image: %s", img_path);
        remove(img_path);
        return;
    }

    FILE *csv = fopen(RESULT_CSV, "a");
    if (csv) {
        fprintf(csv, "%d,%s,%s,%.4f,%d,%d,%d,%d\n",
                s_img_counter, ts_str, img_path,
                score, x1, y1, x2, y2);
        fclose(csv);
    }

    ESP_LOGI(TAG, "Saved: det_%s_%04d.jpg | score=%.3f | %s",
             ts_file, s_img_counter, score, ts_str);

    s_img_counter++;
    save_state();
}

#else

static void sd_feature_init(void) {}

static void save_detection(who::cam::cam_fb_t *fb, float score, int x1, int y1, int x2, int y2)
{
    (void)fb;
    (void)score;
    (void)x1;
    (void)y1;
    (void)x2;
    (void)y2;
}

#endif
// ─────────────────────────────────────────────────────────────────────────────

// ── Model feature ────────────────────────────────────────────────────────────
static PedestrianDetect *model_feature_init(void)
{
    ESP_LOGI(TAG, "Loading model...");

    PedestrianDetect *model = new PedestrianDetect(
        static_cast<PedestrianDetect::model_type_t>(CONFIG_DEFAULT_PEDESTRIAN_DETECT_MODEL),
        false
    );

    model->set_score_thr(SCORE_THRESHOLD);

    ESP_LOGI(TAG, "Model ready.");

    return model;
}
// ─────────────────────────────────────────────────────────────────────────────

// ── Main detection + light task ──────────────────────────────────────────────
class DetectLightTask : public who::task::WhoTask {
public:
    DetectLightTask(who::frame_cap::WhoFrameCapNode *frame_node,
                    LightController *light_ctrl,
                    PedestrianDetect *model)
        : who::task::WhoTask("DetectLightTask"),
          m_frame_node(frame_node),
          m_light_ctrl(light_ctrl),
          m_model(model),
          m_frame_count(0),
          m_last_save_ms(0),
          m_window_active(false),
          m_sample_frame(0),
          m_frame1_result(false)
    {}

    bool start()
    {
        m_frame_node->add_new_frame_signal_subscriber(this);
        return run(8192, 5, 1);
    }

private:
    void task() override
    {
        while (true) {
            EventBits_t bits = xEventGroupWaitBits(
                m_event_group,
                who::frame_cap::WhoFrameCapNode::NEW_FRAME | TASK_STOP,
                pdTRUE,
                pdFALSE,
                portMAX_DELAY
            );

            if (bits & TASK_STOP) break;
            if (!(bits & who::frame_cap::WhoFrameCapNode::NEW_FRAME)) continue;

            auto fb = m_frame_node->cam_fb_peek();
            if (!fb || !fb->buf) continue;
            uint64_t now_ms_global = esp_timer_get_time() / 1000;
            person_trigger_gpio_set(now_ms_global < m_gpio_high_until_ms);
            dl::image::img_t img = static_cast<dl::image::img_t>(*fb);

            // Always update the fill light on every frame
            light_feature_update(m_light_ctrl, img);

            // ── Waiting phase: skip until FRAME_SAMPLE_RATE frames have passed ──
            if (!m_window_active) {
                m_frame_count++;
                if (m_frame_count < FRAME_SAMPLE_RATE) continue;

                // Enter the 2-frame sampling window
                m_frame_count   = 0;
                m_window_active = true;
                m_sample_frame  = 0;
            }

            // ── 2-frame sampling window ──────────────────────────────────────
            auto &results = m_model->run(img);

            // Filter out false detections where the bounding box is exactly
            // [0, 0, 239, 239] — treat this as no person detected.
            bool person_now = false;
            if (!results.empty()) {
                auto &r = results.front();
                bool is_false_box = (r.box[0] == 0   && r.box[1] == 0 &&
                                     r.box[2] == 239  && r.box[3] == 239);
                if (is_false_box) {
                    ESP_LOGD(TAG, "False detection filtered: box=[0,0,239,239]");
                } else {
                    person_now = true;
                }
            }

            char ts[24];
            get_timestamp_str(ts, sizeof(ts));

            if (m_sample_frame == 0) {
                // Frame 1 of 2: store result and wait for the next frame
                m_frame1_result = person_now;
                m_sample_frame  = 1;

                ESP_LOGD(TAG, "[%s] Sample frame 1/2: %s",
                         ts, person_now ? "PERSON" : "no person");

            } else {
                // Frame 2 of 2: both results are in — make a decision
                bool both_detected = m_frame1_result && person_now;

                // Return to the waiting phase
                m_window_active = false;

                // Drive GPIO 47 directly — HIGH if confirmed, LOW otherwise
                uint64_t now_ms = esp_timer_get_time() / 1000;

                if (both_detected) {
                    m_gpio_high_until_ms = now_ms + monitor_time; // 3 sec extend
                }

                person_trigger_gpio_set(now_ms < m_gpio_high_until_ms);

                if (both_detected) {
                    rgb_person_detected();

                    auto &r = results.front();

                    ESP_LOGI(TAG,
                             "[%s] PERSON confirmed (2/2) | score=%.3f | "
                             "box=[%d,%d,%d,%d] | Brightness: %d | Light: %s",
                             ts,
                             r.score,
                             r.box[0], r.box[1], r.box[2], r.box[3],
                             light_feature_get_brightness(m_light_ctrl),
                             light_feature_get_state(m_light_ctrl));

                    uint64_t now_ms = esp_timer_get_time() / 1000;

                    if (now_ms - m_last_save_ms >= SAVE_COOLDOWN_MS) {
                        m_last_save_ms = now_ms;

                        save_detection(
                            fb,
                            r.score,
                            r.box[0], r.box[1], r.box[2], r.box[3]
                        );
                    }
                } else {
                    rgb_no_person();

                    ESP_LOGI(TAG,
                             "[%s] No person confirmed (2/2) | "
                             "f1=%s f2=%s | Brightness: %d | Light: %s",
                             ts,
                             m_frame1_result ? "Y" : "N",
                             person_now      ? "Y" : "N",
                             light_feature_get_brightness(m_light_ctrl),
                             light_feature_get_state(m_light_ctrl));
                }
            }
        }

        xEventGroupSetBits(m_event_group, TASK_STOPPED);
        vTaskDelete(NULL);
    }

    // ── Members ──────────────────────────────────────────────────────────────
    who::frame_cap::WhoFrameCapNode *m_frame_node;
    LightController                 *m_light_ctrl;
    PedestrianDetect                *m_model;
    uint64_t m_gpio_high_until_ms = 0;
    // Waiting-phase counter
    int      m_frame_count;
    uint64_t m_last_save_ms;

    // 2-frame window state
    bool     m_window_active;   // true while inside the sampling window
    int      m_sample_frame;    // 0 = waiting for frame 1, 1 = waiting for frame 2
    bool     m_frame1_result;   // detection result of the first sampled frame
};
// ─────────────────────────────────────────────────────────────────────────────

// ── App main ─────────────────────────────────────────────────────────────────
extern "C" void app_main(void)
{
    time_feature_init();

    ESP_LOGI(TAG, "=== Person Detection + Light Control ===");
    ESP_LOGI(TAG, "Frame sample rate  : every %d frames", FRAME_SAMPLE_RATE);
    ESP_LOGI(TAG, "Confirm window     : 2 consecutive frames");
    ESP_LOGI(TAG, "Score threshold    : %.2f", SCORE_THRESHOLD);
    ESP_LOGI(TAG, "Brightness thresh  : %d", BRIGHTNESS_THRESHOLD);
    ESP_LOGI(TAG, "SD max usage       : %d MB", SD_MAX_USAGE_MB);
    ESP_LOGI(TAG, "Person trigger GPIO: GPIO %d | direct HIGH/LOW (no timer)",
             (int)PERSON_TRIGGER_GPIO);

    rgb_feature_init();
    rgb_no_person();

    person_trigger_gpio_init();

    sd_feature_init();

    LightController *light_ctrl = light_feature_init();

    light_feature_blink_test(light_ctrl);

    PedestrianDetect *model = model_feature_init();

    auto frame_cap  = get_term_dvp_frame_cap_pipeline();
    auto frame_node = frame_cap->get_last_node();

    auto task = new DetectLightTask(frame_node, light_ctrl, model);

    if (!task->start()) {
        ESP_LOGE(TAG, "Failed to start task");
        return;
    }

    if (!frame_cap->run({{8192, 5, 1}})) {
        ESP_LOGE(TAG, "Failed to start camera");
        return;
    }

    char ts[24];
    get_timestamp_str(ts, sizeof(ts));

    ESP_LOGI(TAG, "Running from %s | Every %d frames | 2-frame confirm | Score >= %.2f",
             ts, FRAME_SAMPLE_RATE, SCORE_THRESHOLD);
}