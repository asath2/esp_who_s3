#pragma once
#include <cstdint>
#include <dl_image_define.hpp>
#include "esp_log.h"
#include "driver/gpio.h"

/**
 * @brief Light Controller for automatic lighting based on camera brightness
 * 
 * Analyzes frame brightness and controls external light via GPIO
 */
class LightController {
public:
    /**
     * @brief Initialize light controller
     * @param gpio_pin GPIO pin number for external light control (e.g., GPIO3 for ESP32-S3-EYE)
     * @param brightness_threshold Target brightness level (0-255)
     *                              Values above this = light OFF
     *                              Values below this = light ON
     */
    LightController(gpio_num_t gpio_pin, uint8_t brightness_threshold = 100);
    
    /**
     * @brief Cleanup resources
     */
    ~LightController();
    
    /**
     * @brief Calculate average brightness from RGB565 frame
     * @param img Image structure with RGB565 pixel data
     * @return Average brightness level (0-255)
     */
    uint8_t calculate_brightness(const dl::image::img_t &img);
    
    /**
     * @brief Update light state based on frame brightness
     * Automatically controls GPIO based on brightness threshold
     * @param img Image to analyze
     * @return true if light is ON, false if OFF
     */
    bool update_light(const dl::image::img_t &img);
    
    /**
     * @brief Manual light control
     * @param on true to turn ON, false to turn OFF
     */
    void set_light(bool on);
    
    /**
     * @brief Set brightness threshold
     * @param threshold Brightness level (0-255)
     */
    void set_threshold(uint8_t threshold) { m_brightness_threshold = threshold; }
    
    /**
     * @brief Get current brightness threshold
     */
    uint8_t get_threshold() const { return m_brightness_threshold; }
    
    /**
     * @brief Get last measured brightness
     */
    uint8_t get_last_brightness() const { return m_last_brightness; }
    
    /**
     * @brief Check if light is currently on
     */
    bool is_light_on() const { return m_light_on; }

private:
    gpio_num_t m_gpio_pin;
    uint8_t m_brightness_threshold;
    uint8_t m_last_brightness;
    bool m_light_on;
    static const char *TAG;
    
    /**
     * @brief Convert RGB565 pixel to brightness using luminance formula
     * Y = 0.299*R + 0.587*G + 0.114*B
     */
    uint8_t rgb565_to_brightness(uint16_t pixel);
};
