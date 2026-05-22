/**
 * @file app_main.cpp
 * @brief Offline Pedestrian Detection from SD card images.
 *
 * Images read from: /sdcard/ (root)
 * Results saved to: /sdcard/results/results.csv
 */

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"

#include "dl_image_define.hpp"
#include "dl_image_jpeg.hpp"
#include "pedestrian_detect.hpp"

static const char *TAG = "SD_INFER";

#define MOUNT_POINT  CONFIG_BSP_SD_MOUNT_POINT
#define INPUT_DIR    MOUNT_POINT
#define OUTPUT_DIR   MOUNT_POINT "/results"
#define RESULT_CSV   OUTPUT_DIR  "/results.csv"

#define SCORE_THRESHOLD  0.6f   // ← adjust confidence here (0.5 - 0.95)
#define NMS_THRESHOLD    0.5f   // IoU overlap threshold

// ─────────────────────────────────────────────────────────────────────────────

static bool is_jpeg(const char *name)
{
    size_t len = strlen(name);
    if (len < 4) return false;
    if (strcasecmp(name + len - 4, ".jpg")  == 0) return true;
    if (len >= 5 && strcasecmp(name + len - 5, ".jpeg") == 0) return true;
    return false;
}

static void ensure_dir(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) mkdir(path, 0755);
}

// ─────────────────────────────────────────────────────────────────────────────

static void run_sd_inference(void *arg)
{
    // 1. Mount SD card
    if (bsp_sdcard_mount() != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "SD mounted. Scanning %s ...", INPUT_DIR);
    ensure_dir(OUTPUT_DIR);

    // 2. Open result CSV
    FILE *csv = fopen(RESULT_CSV, "w");
    if (!csv) {
        ESP_LOGE(TAG, "Cannot create CSV at %s", RESULT_CSV);
        vTaskDelete(NULL);
        return;
    }
    fprintf(csv, "filename,img_w,img_h,num_detections,score,x1,y1,x2,y2\n");
    fflush(csv);

    // 3. Load model and set thresholds
    ESP_LOGI(TAG, "Loading model (score_thr=%.2f, nms_thr=%.2f)...", SCORE_THRESHOLD, NMS_THRESHOLD);
    PedestrianDetect *model = new PedestrianDetect(
        static_cast<PedestrianDetect::model_type_t>(CONFIG_DEFAULT_PEDESTRIAN_DETECT_MODEL), false);
    model->set_score_thr(SCORE_THRESHOLD);
    model->set_nms_thr(NMS_THRESHOLD);
    ESP_LOGI(TAG, "Model ready. Starting inference...");

    // 4. Open input directory
    DIR *dir = opendir(INPUT_DIR);
    if (!dir) {
        ESP_LOGE(TAG, "Cannot open dir: %s", INPUT_DIR);
        fclose(csv);
        delete model;
        vTaskDelete(NULL);
        return;
    }

    int total = 0, detected = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR)   continue;
        if (!is_jpeg(entry->d_name))   continue;

        char img_path[128];
        snprintf(img_path, sizeof(img_path), "%s/%s", INPUT_DIR, entry->d_name);
        ESP_LOGI(TAG, "[%d] %s", total + 1, entry->d_name);

        // 5. Read JPEG from SD card
        dl::image::jpeg_img_t jpeg = dl::image::read_jpeg(img_path);
        if (!jpeg.data) {
            ESP_LOGW(TAG, "  Read failed");
            fprintf(csv, "%s,0,0,READ_FAIL,,,,,\n", entry->d_name);
            fflush(csv);
            total++;
            continue;
        }

        // 6. Decode JPEG → RGB565 Big Endian (required by pedestrian_detect model)
        dl::image::img_t img = dl::image::sw_decode_jpeg(
            jpeg,
            dl::image::DL_IMAGE_PIX_TYPE_RGB565,
            dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN
        );
        free(jpeg.data);

        if (!img.data) {
            ESP_LOGW(TAG, "  Decode failed");
            fprintf(csv, "%s,0,0,DECODE_FAIL,,,,,\n", entry->d_name);
            fflush(csv);
            total++;
            continue;
        }

        ESP_LOGI(TAG, "  Size: %dx%d", img.width, img.height);

        // 7. Run inference
        int64_t t1 = esp_timer_get_time();
        auto &results = model->run(img);
        int ms = (int)((esp_timer_get_time() - t1) / 1000);

        ESP_LOGI(TAG, "  %d ms | %d detection(s)", ms, (int)results.size());

        if (results.empty()) {
            fprintf(csv, "%s,%d,%d,0,,,,,\n", entry->d_name, img.width, img.height);
        } else {
            detected++;
            for (auto &r : results) {
                fprintf(csv, "%s,%d,%d,%d,%.4f,%d,%d,%d,%d\n",
                        entry->d_name, img.width, img.height,
                        (int)results.size(),
                        r.score,
                        r.box[0], r.box[1], r.box[2], r.box[3]);
                ESP_LOGI(TAG, "  → score=%.3f box=[%d,%d,%d,%d]",
                         r.score, r.box[0], r.box[1], r.box[2], r.box[3]);
            }
        }
        fflush(csv);
        free(img.data);
        total++;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    closedir(dir);
    fclose(csv);
    delete model;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Done!  Processed : %d images", total);
    ESP_LOGI(TAG, "       Detected  : %d images", detected);
    ESP_LOGI(TAG, "       Results @ : %s", RESULT_CSV);
    ESP_LOGI(TAG, "========================================");

    bsp_sdcard_unmount();
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(run_sd_inference, "sd_infer", 8192, NULL, 5, NULL, 1);
}














































// /**
//  * @file app_main_with_light_control.cpp
//  * @brief Person Detection + Brightness-based LED control + WS2812 RGB LED
//  *
//  * ── Tuning parameters ────────────────────────────────────────────────────────
//  *  FRAME_SAMPLE_RATE    : send every Nth frame to model
//  *  SCORE_THRESHOLD      : model confidence to count as person (0.0 - 1.0)
//  *  BRIGHTNESS_THRESHOLD : fill light ON when brightness < this (0 - 255)
//  *  SAVE_COOLDOWN_MS     : min ms between SD card saves
//  *  SD_MAX_USAGE_MB      : max SD usage in MB before old files are deleted
//  *  SD_DELETE_BATCH      : how many oldest files to delete when full
//  *
//  * ── Time ─────────────────────────────────────────────────────────────────────
//  *  Set START_YEAR/MONTH/DAY/HOUR/MIN/SEC to your current date & time.
//  *  All timestamps are calculated from this base.
//  * ─────────────────────────────────────────────────────────────────────────────
//  */

// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <dirent.h>
// #include <sys/stat.h>
// #include <time.h>
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "bsp/esp-bsp.h"
// #include "driver/rmt_tx.h"
// #include "driver/rmt_encoder.h"

// #include "frame_cap_pipeline.hpp"
// #include "light_controller.hpp"
// #include "who_frame_cap.hpp"
// #include "img_converters.h"
// #include "pedestrian_detect.hpp"
// static const char *TAG = "MAIN";

// // ── Tuning parameters — change these anytime ─────────────────────────────────
// #define LIGHT_GPIO            14
// #define BRIGHTNESS_THRESHOLD  100
// #define RGB_GPIO              GPIO_NUM_48
// #define FRAME_SAMPLE_RATE     10
// #define SCORE_THRESHOLD       0.8f
// #define SAVE_COOLDOWN_MS      3000
// #define SD_MAX_USAGE_MB       900          // start deleting when SD usage > this (MB)
// #define SD_DELETE_BATCH       10           // delete this many oldest files at once
// #define MOUNT_POINT           CONFIG_BSP_SD_MOUNT_POINT
// #define SAVE_DIR              MOUNT_POINT "/detections"
// #define RESULT_CSV            SAVE_DIR "/log.csv"

// // ── Start time — set this to current date & time before flashing ──────────────
// #define START_YEAR   2026
// #define START_MONTH  5        // 1-12
// #define START_DAY    8        // 1-31
// #define START_HOUR   10       // 0-23
// #define START_MIN    0        // 0-59
// #define START_SEC    0        // 0-59
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Time helpers ──────────────────────────────────────────────────────────────
// static time_t s_boot_time = 0;   // Unix timestamp at boot

// static void time_init(void)
// {
//     struct tm t = {};
//     t.tm_year = START_YEAR - 1900;
//     t.tm_mon  = START_MONTH - 1;
//     t.tm_mday = START_DAY;
//     t.tm_hour = START_HOUR;
//     t.tm_min  = START_MIN;
//     t.tm_sec  = START_SEC;
//     s_boot_time = mktime(&t);
//     ESP_LOGI(TAG, "Boot time set: %04d-%02d-%02d %02d:%02d:%02d",
//              START_YEAR, START_MONTH, START_DAY,
//              START_HOUR, START_MIN, START_SEC);
// }

// // Get current time string: "2026-05-08 10:05:23"
// static void get_timestamp_str(char *buf, size_t buf_size)
// {
//     uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
//     time_t   now         = s_boot_time + (time_t)elapsed_sec;
//     struct tm *t         = localtime(&now);
//     strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", t);
// }

// // Get filename-safe time string: "20260508_100523"
// static void get_timestamp_filename(char *buf, size_t buf_size)
// {
//     uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
//     time_t   now         = s_boot_time + (time_t)elapsed_sec;
//     struct tm *t         = localtime(&now);
//     strftime(buf, buf_size, "%Y%m%d_%H%M%S", t);
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── WS2812 NeoPixel driver ────────────────────────────────────────────────────
// #define WS2812_T0H_NS  350
// #define WS2812_T0L_NS  800
// #define WS2812_T1H_NS  700
// #define WS2812_T1L_NS  600

// static rmt_channel_handle_t s_rmt_chan    = nullptr;
// static rmt_encoder_handle_t s_rmt_encoder = nullptr;

// static void ws2812_init(void)
// {
//     rmt_tx_channel_config_t chan_cfg = {};
//     chan_cfg.gpio_num          = RGB_GPIO;
//     chan_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
//     chan_cfg.resolution_hz     = 10 * 1000 * 1000;
//     chan_cfg.mem_block_symbols = 64;
//     chan_cfg.trans_queue_depth = 4;
//     ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_rmt_chan));

//     rmt_bytes_encoder_config_t enc_cfg = {};
//     enc_cfg.bit0.duration0  = (uint16_t)(WS2812_T0H_NS / 100);
//     enc_cfg.bit0.level0     = 1;
//     enc_cfg.bit0.duration1  = (uint16_t)(WS2812_T0L_NS / 100);
//     enc_cfg.bit0.level1     = 0;
//     enc_cfg.bit1.duration0  = (uint16_t)(WS2812_T1H_NS / 100);
//     enc_cfg.bit1.level0     = 1;
//     enc_cfg.bit1.duration1  = (uint16_t)(WS2812_T1L_NS / 100);
//     enc_cfg.bit1.level1     = 0;
//     enc_cfg.flags.msb_first = 1;
//     ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_rmt_encoder));
//     ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
// }

// static void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b)
// {
//     uint8_t grb[3] = { g, r, b };
//     rmt_transmit_config_t tx_cfg = {};
//     tx_cfg.loop_count = 0;
//     rmt_transmit(s_rmt_chan, s_rmt_encoder, grb, sizeof(grb), &tx_cfg);
//     rmt_tx_wait_all_done(s_rmt_chan, pdMS_TO_TICKS(100));
// }

// #define RGB_RED()    ws2812_set_color(255, 0,   0)
// #define RGB_GREEN()  ws2812_set_color(0,   255, 0)
// // ─────────────────────────────────────────────────────────────────────────────

// // ── SD card storage management ────────────────────────────────────────────────
// static bool s_sd_ok       = false;
// static int  s_img_counter = 0;

// // Get total size of all files in SAVE_DIR in MB
// static uint32_t get_save_dir_size_mb(void)
// {
//     DIR *dir = opendir(SAVE_DIR);
//     if (!dir) return 0;

//     uint64_t total_bytes = 0;
//     struct dirent *entry;
//     struct stat st;
//     char path[1024];

//     while ((entry = readdir(dir)) != NULL) {
//         if (entry->d_type != DT_REG) continue;
//         snprintf(path, sizeof(path), "%s/%s", SAVE_DIR, entry->d_name);
//         if (stat(path, &st) == 0) {
//             total_bytes += st.st_size;
//         }
//     }
//     closedir(dir);
//     return (uint32_t)(total_bytes / (1024 * 1024));
// }

// // Delete oldest SD_DELETE_BATCH JPEG files (sorted by name = sorted by time)
// static void delete_oldest_files(void)
// {
//     DIR *dir = opendir(SAVE_DIR);
//     if (!dir) return;

//     // Collect all jpg filenames
//     char files[100][256];
//     int  count = 0;
//     struct dirent *entry;

//     while ((entry = readdir(dir)) != NULL && count < 100) {
//         if (entry->d_type != DT_REG) continue;
//         size_t len = strlen(entry->d_name);
//         if (len < 4) continue;
//         if (strcasecmp(entry->d_name + len - 4, ".jpg") != 0) continue;
//         snprintf(files[count], sizeof(files[count]), "%s", entry->d_name);
//         count++;
//     }
//     closedir(dir);

//     if (count == 0) return;

//     // Sort by name (oldest first since we name by timestamp)
//     for (int i = 0; i < count - 1; i++) {
//         for (int j = i + 1; j < count; j++) {
//             if (strcmp(files[i], files[j]) > 0) {
//                 char tmp[256];
//                 memcpy(tmp, files[i], sizeof(tmp));
//                 memcpy(files[i], files[j], sizeof(files[i]));
//                 memcpy(files[j], tmp, sizeof(files[j]));
//             }
//         }
//     }

//     // Delete oldest SD_DELETE_BATCH files
//     int to_delete = (count < SD_DELETE_BATCH) ? count : SD_DELETE_BATCH;
//     char path[1024];
//     for (int i = 0; i < to_delete; i++) {
//         strcpy(path, SAVE_DIR);
//         strcat(path, "/");
//         strcat(path, files[i]);
//         if (remove(path) == 0) {
//             ESP_LOGW(TAG, "Deleted old file: %s", files[i]);
//         }
//     }
//     ESP_LOGW(TAG, "Deleted %d oldest files to free space", to_delete);
// }

// static void sd_init(void)
// {
//     if (bsp_sdcard_mount() == ESP_OK) {
//         s_sd_ok = true;
//         struct stat st;
//         if (stat(SAVE_DIR, &st) != 0) mkdir(SAVE_DIR, 0755);

//         // Write CSV header if new file
//         FILE *f = fopen(RESULT_CSV, "r");
//         if (!f) {
//             f = fopen(RESULT_CSV, "w");
//             if (f) {
//                 fprintf(f, "index,datetime,filename,score,x1,y1,x2,y2\n");
//                 fclose(f);
//             }
//         } else {
//             fclose(f);
//         }

//         uint32_t used_mb = get_save_dir_size_mb();
//         ESP_LOGI(TAG, "SD ready: %s | Used: %lu MB / %d MB max",
//                  SAVE_DIR, used_mb, SD_MAX_USAGE_MB);
//     } else {
//         ESP_LOGW(TAG, "SD not available — saving disabled");
//     }
// }

// static void save_detection(camera_fb_t *fb, float score, int x1, int y1, int x2, int y2)
// {
//     if (!s_sd_ok || !fb) return;

//     // Check storage usage — delete old files if needed
//     uint32_t used_mb = get_save_dir_size_mb();
//     if (used_mb >= SD_MAX_USAGE_MB) {
//         ESP_LOGW(TAG, "SD usage %lu MB >= %d MB limit — deleting oldest files...",
//                  used_mb, SD_MAX_USAGE_MB);
//         delete_oldest_files();
//     }

//     // Get timestamp strings
//     char ts_str[24];       // "2026-05-08 10:05:23"
//     char ts_file[20];      // "20260508_100523"
//     get_timestamp_str(ts_str, sizeof(ts_str));
//     get_timestamp_filename(ts_file, sizeof(ts_file));

//     // Build image filename using timestamp
//     char img_path[96];
//     snprintf(img_path, sizeof(img_path), "%s/det_%s_%04d.jpg",
//              SAVE_DIR, ts_file, s_img_counter);

//     // Save JPEG
//         // Save framebuffer directly
//     FILE *f = fopen(img_path, "wb");
//     if (f) {
//         fwrite(fb->buf, 1, fb->len, f);
//         fclose(f);
//     }

//     // Append CSV row with datetime
//     FILE *csv = fopen(RESULT_CSV, "a");
//     if (csv) {
//         fprintf(csv, "%d,%s,%s,%.4f,%d,%d,%d,%d\n",
//                 s_img_counter, ts_str, img_path,
//                 score, x1, y1, x2, y2);
//         fclose(csv);
//     }

//     ESP_LOGI(TAG, "Saved: det_%s_%04d.jpg | score=%.3f | %s",
//              ts_file, s_img_counter, score, ts_str);
//     s_img_counter++;
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Main detection + light task ───────────────────────────────────────────────
// class DetectLightTask : public who::task::WhoTask {
// public:
//     DetectLightTask(who::frame_cap::WhoFrameCapNode *frame_node,
//                     LightController *light_ctrl,
//                     PedestrianDetect *model)
//         : who::task::WhoTask("DetectLightTask"),
//           m_frame_node(frame_node),
//           m_light_ctrl(light_ctrl),
//           m_model(model),
//           m_frame_count(0),
//           m_last_save_ms(0)
//     {}

//     bool start()
//     {
//         m_frame_node->add_new_frame_signal_subscriber(this);
//         return run(8192, 5, 1);
//     }

// private:
//     void task() override
//     {
//         while (true) {
//             EventBits_t bits = xEventGroupWaitBits(
//                 m_event_group,
//                 who::frame_cap::WhoFrameCapNode::NEW_FRAME | TASK_STOP,
//                 pdTRUE, pdFALSE, portMAX_DELAY);

//             if (bits & TASK_STOP) break;
//             if (!(bits & who::frame_cap::WhoFrameCapNode::NEW_FRAME)) continue;

//             auto fb = m_frame_node->cam_fb_peek();
//             if (!fb || !fb->buf) continue;

//             dl::image::img_t img = static_cast<dl::image::img_t>(*fb);

//             // 1. Brightness → fill light (every frame)
//             m_light_ctrl->update_light(img);

//             // 2. Count frames — only run model on every Nth frame
//             m_frame_count++;
//             if (m_frame_count < FRAME_SAMPLE_RATE) continue;
//             m_frame_count = 0;

//             // 3. Run model on sampled frame
//             auto &results = m_model->run(img);

//             // Get current time for log
//             char ts[24];
//             get_timestamp_str(ts, sizeof(ts));

//             if (results.empty()) {
//                 RGB_RED();
//                 ESP_LOGI(TAG, "[%s] No person | Brightness: %d | Light: %s",
//                          ts,
//                          m_light_ctrl->get_last_brightness(),
//                          m_light_ctrl->is_light_on() ? "OFF" : "ON");
//             } else {
//                 RGB_GREEN();
//                 auto &r = results.front();
//                 ESP_LOGI(TAG, "[%s] PERSON | score=%.3f | box=[%d,%d,%d,%d] | Brightness: %d | Light: %s",
//                          ts, r.score,
//                          r.box[0], r.box[1], r.box[2], r.box[3],
//                          m_light_ctrl->get_last_brightness(),
//                          m_light_ctrl->is_light_on() ? "OFF" : "ON");

//                 // Save with cooldown
//                 uint64_t now_ms = esp_timer_get_time() / 1000;
//                 if (now_ms - m_last_save_ms >= SAVE_COOLDOWN_MS) {
//                     m_last_save_ms = now_ms;
//                     save_detection(
//                         reinterpret_cast<camera_fb_t *>(fb),
//                         r.score,
//                         r.box[0], r.box[1], r.box[2], r.box[3]);
//                 }
//             }
//         }

//         xEventGroupSetBits(m_event_group, TASK_STOPPED);
//         vTaskDelete(NULL);
//     }

//     who::frame_cap::WhoFrameCapNode *m_frame_node;
//     LightController  *m_light_ctrl;
//     PedestrianDetect *m_model;
//     int      m_frame_count;
//     uint64_t m_last_save_ms;
// };
// // ─────────────────────────────────────────────────────────────────────────────

// extern "C" void app_main(void)
// {
//     // Init time first
//     time_init();

//     ESP_LOGI(TAG, "=== Person Detection + Light Control ===");
//     ESP_LOGI(TAG, "Frame sample rate  : every %d frames", FRAME_SAMPLE_RATE);
//     ESP_LOGI(TAG, "Score threshold    : %.2f", SCORE_THRESHOLD);
//     ESP_LOGI(TAG, "Brightness thresh  : %d", BRIGHTNESS_THRESHOLD);
//     ESP_LOGI(TAG, "SD max usage       : %d MB", SD_MAX_USAGE_MB);

//     // 1. RGB → RED (standby)
//     ws2812_init();
//     RGB_RED();

//     // 2. SD card
//     sd_init();

//     // 3. Fill light
//     LightController *light_ctrl = new LightController((gpio_num_t)LIGHT_GPIO, BRIGHTNESS_THRESHOLD);

//     // 4. Blink test
//     for (int i = 0; i < 3; i++) {
//         light_ctrl->set_light(true);
//         vTaskDelay(pdMS_TO_TICKS(300));
//         light_ctrl->set_light(false);
//         vTaskDelay(pdMS_TO_TICKS(300));
//     }

//     // 5. Load model
//     ESP_LOGI(TAG, "Loading model...");
//     PedestrianDetect *model = new PedestrianDetect(
//         static_cast<PedestrianDetect::model_type_t>(CONFIG_DEFAULT_PEDESTRIAN_DETECT_MODEL), false);
//     model->set_score_thr(SCORE_THRESHOLD);
//     ESP_LOGI(TAG, "Model ready.");

//     // 6. Camera + task
//     auto frame_cap  = get_term_dvp_frame_cap_pipeline();
//     auto frame_node = frame_cap->get_last_node();

//     auto task = new DetectLightTask(frame_node, light_ctrl, model);
//     if (!task->start()) {
//         ESP_LOGE(TAG, "Failed to start task");
//         return;
//     }

//     if (!frame_cap->run({{8192, 5, 1}})) {
//         ESP_LOGE(TAG, "Failed to start camera");
//         return;
//     }

//     char ts[24];
//     get_timestamp_str(ts, sizeof(ts));
//     ESP_LOGI(TAG, "Running from %s | Every %d frames | Score >= %.2f",
//              ts, FRAME_SAMPLE_RATE, SCORE_THRESHOLD);
// }


























//                                  working code with image save 



// /**
//  * @file app_main_with_light_control.cpp
//  * @brief Person Detection + Brightness-based LED control + WS2812 RGB LED
//  *
//  * ── Tuning parameters ────────────────────────────────────────────────────────
//  *  FRAME_SAMPLE_RATE    : send every Nth frame to model
//  *  SCORE_THRESHOLD      : model confidence to count as person (0.0 - 1.0)
//  *  BRIGHTNESS_THRESHOLD : fill light ON when brightness < this (0 - 255)
//  *  SAVE_COOLDOWN_MS     : min ms between SD card saves
//  *  SD_MAX_USAGE_MB      : max SD usage in MB before old files are deleted
//  *  SD_DELETE_BATCH      : how many oldest files to delete when full
//  *
//  * ── Time ─────────────────────────────────────────────────────────────────────
//  *  Set START_YEAR/MONTH/DAY/HOUR/MIN/SEC to your current date & time.
//  *  All timestamps are calculated from this base.
//  * ─────────────────────────────────────────────────────────────────────────────
//  */

// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <dirent.h>
// #include <sys/stat.h>
// #include <time.h>
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "bsp/esp-bsp.h"
// #include "driver/rmt_tx.h"
// #include "driver/rmt_encoder.h"

// #include "frame_cap_pipeline.hpp"
// #include "light_controller.hpp"
// #include "who_frame_cap.hpp"
// #include "img_converters.h"
// #include "pedestrian_detect.hpp"
// static const char *TAG = "MAIN";

// // ── Tuning parameters — change these anytime ─────────────────────────────────
// #define LIGHT_GPIO            14
// #define BRIGHTNESS_THRESHOLD  100
// #define RGB_GPIO              GPIO_NUM_48
// #define FRAME_SAMPLE_RATE     10
// #define SCORE_THRESHOLD       0.8f
// #define SAVE_COOLDOWN_MS      3000
// #define SD_MAX_USAGE_MB       900          // start deleting when SD usage > this (MB)
// #define SD_DELETE_BATCH       10           // delete this many oldest files at once
// #define MOUNT_POINT           CONFIG_BSP_SD_MOUNT_POINT
// #define SAVE_DIR              MOUNT_POINT "/detections"
// #define RESULT_CSV            SAVE_DIR "/log.csv"

// // ── Start time — set this to current date & time before flashing ──────────────
// #define START_YEAR   2026
// #define START_MONTH  5        // 1-12
// #define START_DAY    8        // 1-31
// #define START_HOUR   10       // 0-23
// #define START_MIN    0        // 0-59
// #define START_SEC    0        // 0-59
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Time helpers ──────────────────────────────────────────────────────────────
// static time_t s_boot_time = 0;   // Unix timestamp at boot

// static void time_init(void)
// {
//     struct tm t = {};
//     t.tm_year = START_YEAR - 1900;
//     t.tm_mon  = START_MONTH - 1;
//     t.tm_mday = START_DAY;
//     t.tm_hour = START_HOUR;
//     t.tm_min  = START_MIN;
//     t.tm_sec  = START_SEC;
//     s_boot_time = mktime(&t);
//     ESP_LOGI(TAG, "Boot time set: %04d-%02d-%02d %02d:%02d:%02d",
//              START_YEAR, START_MONTH, START_DAY,
//              START_HOUR, START_MIN, START_SEC);
// }

// // Get current time string: "2026-05-08 10:05:23"
// static void get_timestamp_str(char *buf, size_t buf_size)
// {
//     uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
//     time_t   now         = s_boot_time + (time_t)elapsed_sec;
//     struct tm *t         = localtime(&now);
//     strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", t);
// }

// // Get filename-safe time string: "20260508_100523"
// static void get_timestamp_filename(char *buf, size_t buf_size)
// {
//     uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
//     time_t   now         = s_boot_time + (time_t)elapsed_sec;
//     struct tm *t         = localtime(&now);
//     strftime(buf, buf_size, "%Y%m%d_%H%M%S", t);
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── WS2812 NeoPixel driver ────────────────────────────────────────────────────
// #define WS2812_T0H_NS  350
// #define WS2812_T0L_NS  800
// #define WS2812_T1H_NS  700
// #define WS2812_T1L_NS  600

// static rmt_channel_handle_t s_rmt_chan    = nullptr;
// static rmt_encoder_handle_t s_rmt_encoder = nullptr;

// static void ws2812_init(void)
// {
//     rmt_tx_channel_config_t chan_cfg = {};
//     chan_cfg.gpio_num          = RGB_GPIO;
//     chan_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
//     chan_cfg.resolution_hz     = 10 * 1000 * 1000;
//     chan_cfg.mem_block_symbols = 64;
//     chan_cfg.trans_queue_depth = 4;
//     ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_rmt_chan));

//     rmt_bytes_encoder_config_t enc_cfg = {};
//     enc_cfg.bit0.duration0  = (uint16_t)(WS2812_T0H_NS / 100);
//     enc_cfg.bit0.level0     = 1;
//     enc_cfg.bit0.duration1  = (uint16_t)(WS2812_T0L_NS / 100);
//     enc_cfg.bit0.level1     = 0;
//     enc_cfg.bit1.duration0  = (uint16_t)(WS2812_T1H_NS / 100);
//     enc_cfg.bit1.level0     = 1;
//     enc_cfg.bit1.duration1  = (uint16_t)(WS2812_T1L_NS / 100);
//     enc_cfg.bit1.level1     = 0;
//     enc_cfg.flags.msb_first = 1;
//     ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_rmt_encoder));
//     ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
// }

// static void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b)
// {
//     uint8_t grb[3] = { g, r, b };
//     rmt_transmit_config_t tx_cfg = {};
//     tx_cfg.loop_count = 0;
//     rmt_transmit(s_rmt_chan, s_rmt_encoder, grb, sizeof(grb), &tx_cfg);
//     rmt_tx_wait_all_done(s_rmt_chan, pdMS_TO_TICKS(100));
// }

// #define RGB_RED()    ws2812_set_color(255, 0,   0)
// #define RGB_GREEN()  ws2812_set_color(0,   255, 0)
// // ─────────────────────────────────────────────────────────────────────────────

// // ── SD card storage management ────────────────────────────────────────────────
// static bool s_sd_ok       = false;
// static int  s_img_counter = 0;

// // Get total size of all files in SAVE_DIR in MB
// static uint32_t get_save_dir_size_mb(void)
// {
//     DIR *dir = opendir(SAVE_DIR);
//     if (!dir) return 0;

//     uint64_t total_bytes = 0;
//     struct dirent *entry;
//     struct stat st;
//     char path[1024];

//     while ((entry = readdir(dir)) != NULL) {
//         if (entry->d_type != DT_REG) continue;
//         snprintf(path, sizeof(path), "%s/%s", SAVE_DIR, entry->d_name);
//         if (stat(path, &st) == 0) {
//             total_bytes += st.st_size;
//         }
//     }
//     closedir(dir);
//     return (uint32_t)(total_bytes / (1024 * 1024));
// }

// // Delete oldest SD_DELETE_BATCH JPEG files (sorted by name = sorted by time)
// static void delete_oldest_files(void)
// {
//     DIR *dir = opendir(SAVE_DIR);
//     if (!dir) return;

//     // Collect all jpg filenames
//     char files[100][256];
//     int  count = 0;
//     struct dirent *entry;

//     while ((entry = readdir(dir)) != NULL && count < 100) {
//         if (entry->d_type != DT_REG) continue;
//         size_t len = strlen(entry->d_name);
//         if (len < 4) continue;
//         if (strcasecmp(entry->d_name + len - 4, ".jpg") != 0) continue;
//         snprintf(files[count], sizeof(files[count]), "%s", entry->d_name);
//         count++;
//     }
//     closedir(dir);

//     if (count == 0) return;

//     // Sort by name (oldest first since we name by timestamp)
//     for (int i = 0; i < count - 1; i++) {
//         for (int j = i + 1; j < count; j++) {
//             if (strcmp(files[i], files[j]) > 0) {
//                 char tmp[256];
//                 memcpy(tmp, files[i], sizeof(tmp));
//                 memcpy(files[i], files[j], sizeof(files[i]));
//                 memcpy(files[j], tmp, sizeof(files[j]));
//             }
//         }
//     }

//     // Delete oldest SD_DELETE_BATCH files
//     int to_delete = (count < SD_DELETE_BATCH) ? count : SD_DELETE_BATCH;
//     char path[1024];
//     for (int i = 0; i < to_delete; i++) {
//         strcpy(path, SAVE_DIR);
//         strcat(path, "/");
//         strcat(path, files[i]);
//         if (remove(path) == 0) {
//             ESP_LOGW(TAG, "Deleted old file: %s", files[i]);
//         }
//     }
//     ESP_LOGW(TAG, "Deleted %d oldest files to free space", to_delete);
// }

// static void sd_init(void)
// {
//     if (bsp_sdcard_mount() == ESP_OK) {
//         s_sd_ok = true;
//         struct stat st;
//         if (stat(SAVE_DIR, &st) != 0) mkdir(SAVE_DIR, 0755);

//         // Write CSV header if new file
//         FILE *f = fopen(RESULT_CSV, "r");
//         if (!f) {
//             f = fopen(RESULT_CSV, "w");
//             if (f) {
//                 fprintf(f, "index,datetime,filename,score,x1,y1,x2,y2\n");
//                 fclose(f);
//             }
//         } else {
//             fclose(f);
//         }

//         uint32_t used_mb = get_save_dir_size_mb();
//         ESP_LOGI(TAG, "SD ready: %s | Used: %lu MB / %d MB max",
//                  SAVE_DIR, used_mb, SD_MAX_USAGE_MB);
//     } else {
//         ESP_LOGW(TAG, "SD not available — saving disabled");
//     }
// }

// typedef struct {
//     FILE *file;
//     bool ok;
//     size_t total;
// } jpg_file_writer_t;

// static size_t jpg_file_write_cb(void *arg, size_t index, const void *data, size_t len)
// {
//     jpg_file_writer_t *writer = (jpg_file_writer_t *)arg;

//     if (!writer || !writer->file || !data || len == 0) {
//         return 0;
//     }

//     size_t written = fwrite(data, 1, len, writer->file);

//     if (written != len) {
//         writer->ok = false;
//     }

//     writer->total += written;
//     return written;
// }

// static bool who_format_to_pixformat(who::cam::cam_fb_fmt_t in, pixformat_t *out)
// {
//     if (!out) return false;

//     switch (in) {
//     case who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565:
//         *out = PIXFORMAT_RGB565;
//         return true;

//     case who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB888:
//         *out = PIXFORMAT_RGB888;
//         return true;

//     case who::cam::cam_fb_fmt_t::CAM_FB_FMT_JPEG:
//         *out = PIXFORMAT_JPEG;
//         return true;

//     default:
//         return false;
//     }
// }


// static void save_detection(who::cam::cam_fb_t *fb, float score, int x1, int y1, int x2, int y2)

// {
//     if (!s_sd_ok || !fb || !fb->buf) return;

//     uint32_t used_mb = get_save_dir_size_mb();
//     if (used_mb >= SD_MAX_USAGE_MB) {
//         ESP_LOGW(TAG, "SD usage %lu MB >= %d MB limit — deleting oldest files...",
//                  used_mb, SD_MAX_USAGE_MB);
//         delete_oldest_files();
//     }

//     char ts_str[24];
//     char ts_file[20];
//     get_timestamp_str(ts_str, sizeof(ts_str));
//     get_timestamp_filename(ts_file, sizeof(ts_file));

//     char img_path[96];
//     snprintf(img_path, sizeof(img_path), "%s/det_%s_%04d.jpg",
//              SAVE_DIR, ts_file, s_img_counter);

// bool image_saved = false;

// FILE *f = fopen(img_path, "wb");
// if (f) {
//     if (fb->format == who::cam::cam_fb_fmt_t::CAM_FB_FMT_JPEG) {
//         size_t written = fwrite(fb->buf, 1, fb->len, f);
//         image_saved = (written == fb->len);
//     } else {
//         pixformat_t pix_format;

//         if (who_format_to_pixformat(fb->format, &pix_format)) {
//             uint8_t *jpg_buf = NULL;
//             size_t jpg_len = 0;

//             bool converted = fmt2jpg(
//                 (uint8_t *)fb->buf,
//                 fb->len,
//                 fb->width,
//                 fb->height,
//                 pix_format,
//                 70,
//                 &jpg_buf,
//                 &jpg_len
//             );

//             if (converted && jpg_buf && jpg_len > 0) {
//                 size_t written = fwrite(jpg_buf, 1, jpg_len, f);
//                 image_saved = (written == jpg_len);
//                 free(jpg_buf);
//             } else {
//                 ESP_LOGE(TAG, "JPEG conversion failed: %dx%d len=%u",
//                          fb->width, fb->height, (unsigned)fb->len);
//             }
//         } else {
//             ESP_LOGE(TAG, "Unsupported frame format");
//         }
//     }

//     fclose(f);
// }


//     if (!image_saved) {
//         ESP_LOGE(TAG, "Failed to save image: %s", img_path);
//         remove(img_path);
//         return;
//     }

//     FILE *csv = fopen(RESULT_CSV, "a");
//     if (csv) {
//         fprintf(csv, "%d,%s,%s,%.4f,%d,%d,%d,%d\n",
//                 s_img_counter, ts_str, img_path,
//                 score, x1, y1, x2, y2);
//         fclose(csv);
//     }

//     ESP_LOGI(TAG, "Saved: det_%s_%04d.jpg | score=%.3f | %s",
//              ts_file, s_img_counter, score, ts_str);

//     s_img_counter++;
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Main detection + light task ───────────────────────────────────────────────
// class DetectLightTask : public who::task::WhoTask {
// public:
//     DetectLightTask(who::frame_cap::WhoFrameCapNode *frame_node,
//                     LightController *light_ctrl,
//                     PedestrianDetect *model)
//         : who::task::WhoTask("DetectLightTask"),
//           m_frame_node(frame_node),
//           m_light_ctrl(light_ctrl),
//           m_model(model),
//           m_frame_count(0),
//           m_last_save_ms(0)
//     {}

//     bool start()
//     {
//         m_frame_node->add_new_frame_signal_subscriber(this);
//         return run(8192, 5, 1);
//     }

// private:
//     void task() override
//     {
//         while (true) {
//             EventBits_t bits = xEventGroupWaitBits(
//                 m_event_group,
//                 who::frame_cap::WhoFrameCapNode::NEW_FRAME | TASK_STOP,
//                 pdTRUE, pdFALSE, portMAX_DELAY);

//             if (bits & TASK_STOP) break;
//             if (!(bits & who::frame_cap::WhoFrameCapNode::NEW_FRAME)) continue;

//             auto fb = m_frame_node->cam_fb_peek();
//             if (!fb || !fb->buf) continue;

//             dl::image::img_t img = static_cast<dl::image::img_t>(*fb);

//             // 1. Brightness → fill light (every frame)
//             m_light_ctrl->update_light(img);

//             // 2. Count frames — only run model on every Nth frame
//             m_frame_count++;
//             if (m_frame_count < FRAME_SAMPLE_RATE) continue;
//             m_frame_count = 0;

//             // 3. Run model on sampled frame
//             auto &results = m_model->run(img);

//             // Get current time for log
//             char ts[24];
//             get_timestamp_str(ts, sizeof(ts));

//             if (results.empty()) {
//                 RGB_RED();
//                 ESP_LOGI(TAG, "[%s] No person | Brightness: %d | Light: %s",
//                          ts,
//                          m_light_ctrl->get_last_brightness(),
//                          m_light_ctrl->is_light_on() ? "OFF" : "ON");
//             } else {
//                 RGB_GREEN();
//                 auto &r = results.front();
//                 ESP_LOGI(TAG, "[%s] PERSON | score=%.3f | box=[%d,%d,%d,%d] | Brightness: %d | Light: %s",
//                          ts, r.score,
//                          r.box[0], r.box[1], r.box[2], r.box[3],
//                          m_light_ctrl->get_last_brightness(),
//                          m_light_ctrl->is_light_on() ? "OFF" : "ON");

//                 // Save with cooldown
//                 uint64_t now_ms = esp_timer_get_time() / 1000;
//                 if (now_ms - m_last_save_ms >= SAVE_COOLDOWN_MS) {
//                     m_last_save_ms = now_ms;
//                     save_detection(
//     fb,
//     r.score,
//     r.box[0], r.box[1], r.box[2], r.box[3]);

//                 }
//             }
//         }

//         xEventGroupSetBits(m_event_group, TASK_STOPPED);
//         vTaskDelete(NULL);
//     }

//     who::frame_cap::WhoFrameCapNode *m_frame_node;
//     LightController  *m_light_ctrl;
//     PedestrianDetect *m_model;
//     int      m_frame_count;
//     uint64_t m_last_save_ms;
// };
// // ─────────────────────────────────────────────────────────────────────────────

// extern "C" void app_main(void)
// {
//     // Init time first
//     time_init();

//     ESP_LOGI(TAG, "=== Person Detection + Light Control ===");
//     ESP_LOGI(TAG, "Frame sample rate  : every %d frames", FRAME_SAMPLE_RATE);
//     ESP_LOGI(TAG, "Score threshold    : %.2f", SCORE_THRESHOLD);
//     ESP_LOGI(TAG, "Brightness thresh  : %d", BRIGHTNESS_THRESHOLD);
//     ESP_LOGI(TAG, "SD max usage       : %d MB", SD_MAX_USAGE_MB);

//     // 1. RGB → RED (standby)
//     ws2812_init();
//     RGB_RED();

//     // 2. SD card
//     sd_init();

//     // 3. Fill light
//     LightController *light_ctrl = new LightController((gpio_num_t)LIGHT_GPIO, BRIGHTNESS_THRESHOLD);

//     // 4. Blink test
//     for (int i = 0; i < 3; i++) {
//         light_ctrl->set_light(true);
//         vTaskDelay(pdMS_TO_TICKS(300));
//         light_ctrl->set_light(false);
//         vTaskDelay(pdMS_TO_TICKS(300));
//     }

//     // 5. Load model
//     ESP_LOGI(TAG, "Loading model...");
//     PedestrianDetect *model = new PedestrianDetect(
//         static_cast<PedestrianDetect::model_type_t>(CONFIG_DEFAULT_PEDESTRIAN_DETECT_MODEL), false);
//     model->set_score_thr(SCORE_THRESHOLD);
//     ESP_LOGI(TAG, "Model ready.");

//     // 6. Camera + task
//     auto frame_cap  = get_term_dvp_frame_cap_pipeline();
//     auto frame_node = frame_cap->get_last_node();

//     auto task = new DetectLightTask(frame_node, light_ctrl, model);
//     if (!task->start()) {
//         ESP_LOGE(TAG, "Failed to start task");
//         return;
//     }

//     if (!frame_cap->run({{8192, 5, 1}})) {
//         ESP_LOGE(TAG, "Failed to start camera");
//         return;
//     }

//     char ts[24];
//     get_timestamp_str(ts, sizeof(ts));
//     ESP_LOGI(TAG, "Running from %s | Every %d frames | Score >= %.2f",
//              ts, FRAME_SAMPLE_RATE, SCORE_THRESHOLD);
// }








////////////////////////////////////////////////////   final working code  ////////////////////////////////////////////////////////////////////// 


// /**
//  * @file app_main_with_light_control.cpp
//  * @brief Person Detection + Brightness-based LED control + WS2812 RGB LED + Person trigger GPIO
//  */

// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <dirent.h>
// #include <sys/stat.h>
// #include <time.h>

// #include "esp_log.h"
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/timers.h"
// #include "bsp/esp-bsp.h"
// #include "driver/rmt_tx.h"
// #include "driver/rmt_encoder.h"
// #include "driver/gpio.h"

// #include "frame_cap_pipeline.hpp"
// #include "light_controller.hpp"
// #include "who_frame_cap.hpp"
// #include "img_converters.h"
// #include "pedestrian_detect.hpp"

// static const char *TAG = "MAIN";

// // ── Feature enable / disable switches ────────────────────────────────────────
// #define ENABLE_RGB_LED              0
// #define ENABLE_SD_SAVE              1
// #define ENABLE_FILL_LIGHT           1
// #define ENABLE_BLINK_TEST           0
// #define ENABLE_PERSON_TRIGGER_GPIO  1
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Tuning parameters ────────────────────────────────────────────────────────
// #define LIGHT_GPIO                  2
// #define BRIGHTNESS_THRESHOLD        100
// #define RGB_GPIO                    GPIO_NUM_48
// #define PERSON_TRIGGER_GPIO         GPIO_NUM_21
// #define PERSON_TRIGGER_HIGH_MS      3000
// #define FRAME_SAMPLE_RATE           10
// #define SCORE_THRESHOLD             0.8f
// #define SAVE_COOLDOWN_MS            3000
// #define SD_MAX_USAGE_MB             900
// #define SD_DELETE_BATCH             10
// #define MOUNT_POINT                 CONFIG_BSP_SD_MOUNT_POINT
// #define SAVE_DIR                    MOUNT_POINT "/detections"
// #define RESULT_CSV                  SAVE_DIR "/log.csv"
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Start time ───────────────────────────────────────────────────────────────
// #define START_YEAR   2026
// #define START_MONTH  5
// #define START_DAY    8
// #define START_HOUR   10
// #define START_MIN    0
// #define START_SEC    0
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Time feature ─────────────────────────────────────────────────────────────
// static time_t s_boot_time = 0;

// static void time_feature_init(void)
// {
//     struct tm t = {};
//     t.tm_year = START_YEAR - 1900;
//     t.tm_mon  = START_MONTH - 1;
//     t.tm_mday = START_DAY;
//     t.tm_hour = START_HOUR;
//     t.tm_min  = START_MIN;
//     t.tm_sec  = START_SEC;

//     s_boot_time = mktime(&t);

//     ESP_LOGI(TAG, "Boot time set: %04d-%02d-%02d %02d:%02d:%02d",
//              START_YEAR, START_MONTH, START_DAY,
//              START_HOUR, START_MIN, START_SEC);
// }

// static void get_timestamp_str(char *buf, size_t buf_size)
// {
//     uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
//     time_t now = s_boot_time + (time_t)elapsed_sec;
//     struct tm *t = localtime(&now);
//     strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", t);
// }

// static void get_timestamp_filename(char *buf, size_t buf_size)
// {
//     uint64_t elapsed_sec = esp_timer_get_time() / 1000000ULL;
//     time_t now = s_boot_time + (time_t)elapsed_sec;
//     struct tm *t = localtime(&now);
//     strftime(buf, buf_size, "%Y%m%d_%H%M%S", t);
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── RGB LED feature ──────────────────────────────────────────────────────────
// #if ENABLE_RGB_LED

// #define WS2812_T0H_NS  350
// #define WS2812_T0L_NS  800
// #define WS2812_T1H_NS  700
// #define WS2812_T1L_NS  600

// static rmt_channel_handle_t s_rmt_chan = nullptr;
// static rmt_encoder_handle_t s_rmt_encoder = nullptr;

// static void rgb_feature_init(void)
// {
//     rmt_tx_channel_config_t chan_cfg = {};
//     chan_cfg.gpio_num = RGB_GPIO;
//     chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
//     chan_cfg.resolution_hz = 10 * 1000 * 1000;
//     chan_cfg.mem_block_symbols = 64;
//     chan_cfg.trans_queue_depth = 4;

//     ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_rmt_chan));

//     rmt_bytes_encoder_config_t enc_cfg = {};
//     enc_cfg.bit0.duration0 = (uint16_t)(WS2812_T0H_NS / 100);
//     enc_cfg.bit0.level0 = 1;
//     enc_cfg.bit0.duration1 = (uint16_t)(WS2812_T0L_NS / 100);
//     enc_cfg.bit0.level1 = 0;
//     enc_cfg.bit1.duration0 = (uint16_t)(WS2812_T1H_NS / 100);
//     enc_cfg.bit1.level0 = 1;
//     enc_cfg.bit1.duration1 = (uint16_t)(WS2812_T1L_NS / 100);
//     enc_cfg.bit1.level1 = 0;
//     enc_cfg.flags.msb_first = 1;

//     ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_rmt_encoder));
//     ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
// }

// static void rgb_set_color(uint8_t r, uint8_t g, uint8_t b)
// {
//     uint8_t grb[3] = { g, r, b };

//     rmt_transmit_config_t tx_cfg = {};
//     tx_cfg.loop_count = 0;

//     rmt_transmit(s_rmt_chan, s_rmt_encoder, grb, sizeof(grb), &tx_cfg);
//     rmt_tx_wait_all_done(s_rmt_chan, pdMS_TO_TICKS(100));
// }

// static void rgb_no_person(void)
// {
//     rgb_set_color(0, 0, 0);
// }

// static void rgb_person_detected(void)
// {
//     rgb_set_color(0, 0, 0);
// }

// #else

// static void rgb_feature_init(void) {}
// static void rgb_no_person(void) {}
// static void rgb_person_detected(void) {}

// #endif
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Person trigger GPIO feature ──────────────────────────────────────────────
// #if ENABLE_PERSON_TRIGGER_GPIO

// static TimerHandle_t s_person_trigger_timer = nullptr;

// static void person_trigger_timer_cb(TimerHandle_t timer)
// {
//     (void)timer;
//     gpio_set_level(PERSON_TRIGGER_GPIO, 0);
// }

// static void person_trigger_gpio_init(void)
// {
//     gpio_config_t io_conf = {};
//     io_conf.pin_bit_mask = (1ULL << PERSON_TRIGGER_GPIO);
//     io_conf.mode = GPIO_MODE_OUTPUT;
//     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
//     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     io_conf.intr_type = GPIO_INTR_DISABLE;

//     ESP_ERROR_CHECK(gpio_config(&io_conf));
//     gpio_set_level(PERSON_TRIGGER_GPIO, 0);

//     s_person_trigger_timer = xTimerCreate(
//         "person_gpio_timer",
//         pdMS_TO_TICKS(PERSON_TRIGGER_HIGH_MS),
//         pdFALSE,
//         nullptr,
//         person_trigger_timer_cb
//     );

//     if (!s_person_trigger_timer) {
//         ESP_LOGE(TAG, "Failed to create person trigger timer");
//     } else {
//         ESP_LOGI(TAG, "Person trigger GPIO enabled: GPIO %d | High time: %d ms",
//                  (int)PERSON_TRIGGER_GPIO, PERSON_TRIGGER_HIGH_MS);
//     }
// }

// static void person_trigger_gpio_pulse(void)
// {
//     if (!s_person_trigger_timer) return;

//     gpio_set_level(PERSON_TRIGGER_GPIO, 1);

//     xTimerStop(s_person_trigger_timer, 0);
//     xTimerChangePeriod(
//         s_person_trigger_timer,
//         pdMS_TO_TICKS(PERSON_TRIGGER_HIGH_MS),
//         0
//     );
//     xTimerStart(s_person_trigger_timer, 0);
// }

// #else

// static void person_trigger_gpio_init(void) {}
// static void person_trigger_gpio_pulse(void) {}

// #endif
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Fill light feature ───────────────────────────────────────────────────────
// static LightController *light_feature_init(void)
// {
// #if ENABLE_FILL_LIGHT
//     return new LightController((gpio_num_t)LIGHT_GPIO, BRIGHTNESS_THRESHOLD);
// #else
//     return nullptr;
// #endif
// }

// static void light_feature_blink_test(LightController *light_ctrl)
// {
// #if ENABLE_FILL_LIGHT && ENABLE_BLINK_TEST
//     if (!light_ctrl) return;

//     for (int i = 0; i < 3; i++) {
//         light_ctrl->set_light(true);
//         vTaskDelay(pdMS_TO_TICKS(300));
//         light_ctrl->set_light(false);
//         vTaskDelay(pdMS_TO_TICKS(300));
//     }
// #else
//     (void)light_ctrl;
// #endif
// }

// static void light_feature_update(LightController *light_ctrl, dl::image::img_t &img)
// {
// #if ENABLE_FILL_LIGHT
//     if (light_ctrl) {
//         light_ctrl->update_light(img);
//     }
// #else
//     (void)light_ctrl;
//     (void)img;
// #endif
// }

// static int light_feature_get_brightness(LightController *light_ctrl)
// {
// #if ENABLE_FILL_LIGHT
//     return light_ctrl ? light_ctrl->get_last_brightness() : -1;
// #else
//     (void)light_ctrl;
//     return -1;
// #endif
// }

// static const char *light_feature_get_state(LightController *light_ctrl)
// {
// #if ENABLE_FILL_LIGHT
//     if (!light_ctrl) return "DISABLED";
//     return light_ctrl->is_light_on() ? "OFF" : "ON";
// #else
//     (void)light_ctrl;
//     return "DISABLED";
// #endif
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── SD save feature ──────────────────────────────────────────────────────────
// #if ENABLE_SD_SAVE

// static bool s_sd_ok = false;
// static int s_img_counter = 0;

// static uint32_t get_save_dir_size_mb(void)
// {
//     DIR *dir = opendir(SAVE_DIR);
//     if (!dir) return 0;

//     uint64_t total_bytes = 0;
//     struct dirent *entry;
//     struct stat st;
//     char path[1024];

//     while ((entry = readdir(dir)) != NULL) {
//         if (entry->d_type != DT_REG) continue;

//         snprintf(path, sizeof(path), "%s/%s", SAVE_DIR, entry->d_name);

//         if (stat(path, &st) == 0) {
//             total_bytes += st.st_size;
//         }
//     }

//     closedir(dir);
//     return (uint32_t)(total_bytes / (1024 * 1024));
// }

// static void delete_oldest_files(void)
// {
//     DIR *dir = opendir(SAVE_DIR);
//     if (!dir) return;

//     char files[100][256];
//     int count = 0;
//     struct dirent *entry;

//     while ((entry = readdir(dir)) != NULL && count < 100) {
//         if (entry->d_type != DT_REG) continue;

//         size_t len = strlen(entry->d_name);
//         if (len < 4) continue;
//         if (strcasecmp(entry->d_name + len - 4, ".jpg") != 0) continue;

//         snprintf(files[count], sizeof(files[count]), "%s", entry->d_name);
//         count++;
//     }

//     closedir(dir);

//     if (count == 0) return;

//     for (int i = 0; i < count - 1; i++) {
//         for (int j = i + 1; j < count; j++) {
//             if (strcmp(files[i], files[j]) > 0) {
//                 char tmp[256];
//                 memcpy(tmp, files[i], sizeof(tmp));
//                 memcpy(files[i], files[j], sizeof(files[i]));
//                 memcpy(files[j], tmp, sizeof(files[j]));
//             }
//         }
//     }

//     int to_delete = (count < SD_DELETE_BATCH) ? count : SD_DELETE_BATCH;
//     char path[1024];

//     for (int i = 0; i < to_delete; i++) {
//     snprintf(path, sizeof(path), "%s/%.*s",
//              SAVE_DIR,
//              (int)sizeof(files[i]) - 1,
//              files[i]);

//     if (remove(path) == 0) {
//         ESP_LOGW(TAG, "Deleted old file: %s", files[i]);
//     }
// }


//     ESP_LOGW(TAG, "Deleted %d oldest files to free space", to_delete);
// }

// static void sd_feature_init(void)
// {
//     if (bsp_sdcard_mount() == ESP_OK) {
//         s_sd_ok = true;

//         struct stat st;
//         if (stat(SAVE_DIR, &st) != 0) {
//             mkdir(SAVE_DIR, 0755);
//         }

//         FILE *f = fopen(RESULT_CSV, "r");
//         if (!f) {
//             f = fopen(RESULT_CSV, "w");
//             if (f) {
//                 fprintf(f, "index,datetime,filename,score,x1,y1,x2,y2\n");
//                 fclose(f);
//             }
//         } else {
//             fclose(f);
//         }

//         uint32_t used_mb = get_save_dir_size_mb();

//         ESP_LOGI(TAG, "SD ready: %s | Used: %lu MB / %d MB max",
//                  SAVE_DIR, used_mb, SD_MAX_USAGE_MB);
//     } else {
//         ESP_LOGW(TAG, "SD not available - saving disabled");
//     }
// }

// static bool who_format_to_pixformat(who::cam::cam_fb_fmt_t in, pixformat_t *out)
// {
//     if (!out) return false;

//     switch (in) {
//     case who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565:
//         *out = PIXFORMAT_RGB565;
//         return true;

//     case who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB888:
//         *out = PIXFORMAT_RGB888;
//         return true;

//     case who::cam::cam_fb_fmt_t::CAM_FB_FMT_JPEG:
//         *out = PIXFORMAT_JPEG;
//         return true;

//     default:
//         return false;
//     }
// }

// static void save_detection(who::cam::cam_fb_t *fb, float score, int x1, int y1, int x2, int y2)
// {
//     if (!s_sd_ok || !fb || !fb->buf) return;

//     uint32_t used_mb = get_save_dir_size_mb();
//     if (used_mb >= SD_MAX_USAGE_MB) {
//         ESP_LOGW(TAG, "SD usage %lu MB >= %d MB limit - deleting oldest files...",
//                  used_mb, SD_MAX_USAGE_MB);
//         delete_oldest_files();
//     }

//     char ts_str[24];
//     char ts_file[20];

//     get_timestamp_str(ts_str, sizeof(ts_str));
//     get_timestamp_filename(ts_file, sizeof(ts_file));

//     char img_path[96];
//     snprintf(img_path, sizeof(img_path), "%s/det_%s_%04d.jpg",
//              SAVE_DIR, ts_file, s_img_counter);

//     bool image_saved = false;

//     FILE *f = fopen(img_path, "wb");
//     if (f) {
//         if (fb->format == who::cam::cam_fb_fmt_t::CAM_FB_FMT_JPEG) {
//             size_t written = fwrite(fb->buf, 1, fb->len, f);
//             image_saved = (written == fb->len);
//         } else {
//             pixformat_t pix_format;

//             if (who_format_to_pixformat(fb->format, &pix_format)) {
//                 uint8_t *jpg_buf = NULL;
//                 size_t jpg_len = 0;

//                 bool converted = fmt2jpg(
//                     (uint8_t *)fb->buf,
//                     fb->len,
//                     fb->width,
//                     fb->height,
//                     pix_format,
//                     70,
//                     &jpg_buf,
//                     &jpg_len
//                 );

//                 if (converted && jpg_buf && jpg_len > 0) {
//                     size_t written = fwrite(jpg_buf, 1, jpg_len, f);
//                     image_saved = (written == jpg_len);
//                     free(jpg_buf);
//                 } else {
//                     ESP_LOGE(TAG, "JPEG conversion failed: %dx%d len=%u",
//                              fb->width, fb->height, (unsigned)fb->len);
//                 }
//             } else {
//                 ESP_LOGE(TAG, "Unsupported frame format");
//             }
//         }

//         fclose(f);
//     }

//     if (!image_saved) {
//         ESP_LOGE(TAG, "Failed to save image: %s", img_path);
//         remove(img_path);
//         return;
//     }

//     FILE *csv = fopen(RESULT_CSV, "a");
//     if (csv) {
//         fprintf(csv, "%d,%s,%s,%.4f,%d,%d,%d,%d\n",
//                 s_img_counter, ts_str, img_path,
//                 score, x1, y1, x2, y2);
//         fclose(csv);
//     }

//     ESP_LOGI(TAG, "Saved: det_%s_%04d.jpg | score=%.3f | %s",
//              ts_file, s_img_counter, score, ts_str);

//     s_img_counter++;
// }

// #else

// static void sd_feature_init(void) {}

// static void save_detection(who::cam::cam_fb_t *fb, float score, int x1, int y1, int x2, int y2)
// {
//     (void)fb;
//     (void)score;
//     (void)x1;
//     (void)y1;
//     (void)x2;
//     (void)y2;
// }

// #endif
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Model feature ────────────────────────────────────────────────────────────
// static PedestrianDetect *model_feature_init(void)
// {
//     ESP_LOGI(TAG, "Loading model...");

//     PedestrianDetect *model = new PedestrianDetect(
//         static_cast<PedestrianDetect::model_type_t>(CONFIG_DEFAULT_PEDESTRIAN_DETECT_MODEL),
//         false
//     );

//     model->set_score_thr(SCORE_THRESHOLD);

//     ESP_LOGI(TAG, "Model ready.");

//     return model;
// }
// // ─────────────────────────────────────────────────────────────────────────────

// // ── Main detection + light task ──────────────────────────────────────────────
// class DetectLightTask : public who::task::WhoTask {
// public:
//     DetectLightTask(who::frame_cap::WhoFrameCapNode *frame_node,
//                     LightController *light_ctrl,
//                     PedestrianDetect *model)
//         : who::task::WhoTask("DetectLightTask"),
//           m_frame_node(frame_node),
//           m_light_ctrl(light_ctrl),
//           m_model(model),
//           m_frame_count(0),
//           m_last_save_ms(0)
//     {}

//     bool start()
//     {
//         m_frame_node->add_new_frame_signal_subscriber(this);
//         return run(8192, 5, 1);
//     }

// private:
//     void task() override
//     {
//         while (true) {
//             EventBits_t bits = xEventGroupWaitBits(
//                 m_event_group,
//                 who::frame_cap::WhoFrameCapNode::NEW_FRAME | TASK_STOP,
//                 pdTRUE,
//                 pdFALSE,
//                 portMAX_DELAY
//             );

//             if (bits & TASK_STOP) break;
//             if (!(bits & who::frame_cap::WhoFrameCapNode::NEW_FRAME)) continue;

//             auto fb = m_frame_node->cam_fb_peek();
//             if (!fb || !fb->buf) continue;

//             dl::image::img_t img = static_cast<dl::image::img_t>(*fb);

//             light_feature_update(m_light_ctrl, img);

//             m_frame_count++;
//             if (m_frame_count < FRAME_SAMPLE_RATE) continue;
//             m_frame_count = 0;

//             auto &results = m_model->run(img);

//             char ts[24];
//             get_timestamp_str(ts, sizeof(ts));

//             if (results.empty()) {
//                 rgb_no_person();

//                 ESP_LOGI(TAG, "[%s] No person | Brightness: %d | Light: %s",
//                          ts,
//                          light_feature_get_brightness(m_light_ctrl),
//                          light_feature_get_state(m_light_ctrl));
//             } else {
//                 rgb_person_detected();
//                 person_trigger_gpio_pulse();

//                 auto &r = results.front();

//                 ESP_LOGI(TAG, "[%s] PERSON | score=%.3f | box=[%d,%d,%d,%d] | Brightness: %d | Light: %s",
//                          ts,
//                          r.score,
//                          r.box[0], r.box[1], r.box[2], r.box[3],
//                          light_feature_get_brightness(m_light_ctrl),
//                          light_feature_get_state(m_light_ctrl));

//                 uint64_t now_ms = esp_timer_get_time() / 1000;

//                 if (now_ms - m_last_save_ms >= SAVE_COOLDOWN_MS) {
//                     m_last_save_ms = now_ms;

//                     save_detection(
//                         fb,
//                         r.score,
//                         r.box[0], r.box[1], r.box[2], r.box[3]
//                     );
//                 }
//             }
//         }

//         xEventGroupSetBits(m_event_group, TASK_STOPPED);
//         vTaskDelete(NULL);
//     }

//     who::frame_cap::WhoFrameCapNode *m_frame_node;
//     LightController *m_light_ctrl;
//     PedestrianDetect *m_model;
//     int m_frame_count;
//     uint64_t m_last_save_ms;
// };
// // ─────────────────────────────────────────────────────────────────────────────

// // ── App main ─────────────────────────────────────────────────────────────────
// extern "C" void app_main(void)
// {
//     time_feature_init();

//     ESP_LOGI(TAG, "=== Person Detection + Light Control ===");
//     ESP_LOGI(TAG, "Frame sample rate  : every %d frames", FRAME_SAMPLE_RATE);
//     ESP_LOGI(TAG, "Score threshold    : %.2f", SCORE_THRESHOLD);
//     ESP_LOGI(TAG, "Brightness thresh  : %d", BRIGHTNESS_THRESHOLD);
//     ESP_LOGI(TAG, "SD max usage       : %d MB", SD_MAX_USAGE_MB);
//     ESP_LOGI(TAG, "Person trigger GPIO: GPIO %d | High time: %d ms",
//              (int)PERSON_TRIGGER_GPIO, PERSON_TRIGGER_HIGH_MS);

//     rgb_feature_init();
//     rgb_no_person();

//     person_trigger_gpio_init();

//     sd_feature_init();

//     LightController *light_ctrl = light_feature_init();

//     light_feature_blink_test(light_ctrl);

//     PedestrianDetect *model = model_feature_init();

//     auto frame_cap = get_term_dvp_frame_cap_pipeline();
//     auto frame_node = frame_cap->get_last_node();

//     auto task = new DetectLightTask(frame_node, light_ctrl, model);

//     if (!task->start()) {
//         ESP_LOGE(TAG, "Failed to start task");
//         return;
//     }

//     if (!frame_cap->run({{8192, 5, 1}})) {
//         ESP_LOGE(TAG, "Failed to start camera");
//         return;
//     }

//     char ts[24];
//     get_timestamp_str(ts, sizeof(ts));

//     ESP_LOGI(TAG, "Running from %s | Every %d frames | Score >= %.2f",
//              ts, FRAME_SAMPLE_RATE, SCORE_THRESHOLD);
// }













// //////////////////////////////   Sd card issue resolved code  //////////////////////////////////////////////////////////////////////////////////////


