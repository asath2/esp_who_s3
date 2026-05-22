#include "light_controller.hpp"

const char *LightController::TAG = "LightController";\


// ── Set to true if LED is active LOW (LED ON when GPIO = LOW) ──────────────
// Active LOW  = LED connected: 3.3V → resistor → LED → GPIO
// Active HIGH = LED connected: GPIO → resistor → LED → GND  (default)
#define LED_ACTIVE_LOW  false   // ← change to true if LED is ON by default
// ──────────────────────────────────────────────────────────────────────────

LightController::LightController(gpio_num_t gpio_pin, uint8_t brightness_threshold)
    : m_gpio_pin(gpio_pin),
      m_brightness_threshold(brightness_threshold),
      m_last_brightness(0),
      m_light_on(false)
{
    // Set GPIO level BEFORE configuring as output to avoid glitch
    // This ensures LED starts in OFF state immediately
    gpio_set_level(gpio_pin, LED_ACTIVE_LOW ? 1 : 0);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Confirm OFF state after config
    set_light(false);

    ESP_LOGI(TAG, "Initialized on GPIO%d | threshold=%d | active_%s",
             gpio_pin, brightness_threshold,
             LED_ACTIVE_LOW ? "LOW" : "HIGH");
}

LightController::~LightController()
{
    set_light(false);
}

uint8_t LightController::rgb565_to_brightness(uint16_t pixel)
{
    uint8_t r5 = (pixel >> 11) & 0x1F;
    uint8_t g6 = (pixel >> 5)  & 0x3F;
    uint8_t b5 =  pixel        & 0x1F;

    uint8_t r = (r5 << 3) | (r5 >> 2);
    uint8_t g = (g6 << 2) | (g6 >> 4);
    uint8_t b = (b5 << 3) | (b5 >> 2);

    return (uint8_t)((0.299f * r) + (0.587f * g) + (0.114f * b));
}

uint8_t LightController::calculate_brightness(const dl::image::img_t &img)
{
    if (!img.data || img.width == 0 || img.height == 0) return 0;

    const uint32_t sample_step = 4;
    uint32_t total_brightness  = 0;
    uint32_t sample_count      = 0;

    uint16_t *pixel_data  = (uint16_t *)img.data;
    uint32_t total_pixels = img.width * img.height;

    for (uint32_t i = 0; i < total_pixels; i += sample_step) {
        total_brightness += rgb565_to_brightness(pixel_data[i]);
        sample_count++;
    }

    m_last_brightness = (uint8_t)(total_brightness / sample_count);
    return m_last_brightness;
}

bool LightController::update_light(const dl::image::img_t &img)
{
    uint8_t brightness = calculate_brightness(img);

    // Detect darkness
    bool darkness_detected =
        (brightness < m_brightness_threshold);

    static uint32_t bright_start_time = 0;

    // DARKNESS PRESENT
    if (darkness_detected) {

        // Reset timer because still dark
        bright_start_time = 0;

        // Turn ON only once
        if (!m_light_on) {

            set_light(true);

            ESP_LOGI(TAG,
                     "Darkness detected -> Light ON | Brightness: %d",
                     brightness);
        }
    }
    else {

        // Brightness returned

        if (m_light_on) {

            // Start timer once
            if (bright_start_time == 0) {

                bright_start_time = esp_log_timestamp();

                ESP_LOGI(TAG,"Brightness normal -> " "starting OFF timer");
            }

            // Wait 5 seconds before OFF
            uint32_t elapsed = esp_log_timestamp() - bright_start_time;

            if (elapsed >= 5000) {

                set_light(false);

                bright_start_time = 0;

                ESP_LOGI(TAG, "5 seconds completed -> Light OFF");
            }
        }
    }

    return m_light_on;
}

void LightController::set_light(bool on)
{
    m_light_on = on;

#if LED_ACTIVE_LOW
    gpio_set_level(m_gpio_pin, on ? 0 : 1);
#else
    gpio_set_level(m_gpio_pin, on ? 1 : 0);
#endif
}
