# Light Control Integration Guide

## Overview

This guide explains how to integrate automatic lighting control based on camera brightness with your ESP32-S3 object detection system.

## Files Added

- **light_controller.hpp**: Light controller header with API
- **light_controller.cpp**: Light controller implementation
- **app_main_with_light_control.cpp**: Example integration (reference)

## Quick Start

### 1. Hardware Setup

Connect your external light to an ESP32-S3 GPIO pin:

```
External Light (12V/24V)
  │
  ├─────[Relay/MOSFET Driver]─────[GPIO3/GPIO Number]
  │
  └─────[Ground]──────────────────[ESP32-S3 GND]
```

**Recommended GPIO pins for ESP32-S3-EYE:**
- **GPIO3**: Open-drain compatible, used in examples
- **GPIO4, GPIO5, etc**: Other available GPIOs (check your board schematic)

### 2. Integration Steps

#### Step 1: Add to your CMakeLists.txt

In `examples/object_detect/main/CMakeLists.txt`, add:

```cmake
set(src_files
    app_main.cpp
    light_controller.cpp  # Add this line
)
```

#### Step 2: Modify your app_main.cpp

Add these includes at the top:
```cpp
#include "light_controller.hpp"
#include "driver/gpio.h"
```

In your main function or detection loop:
```cpp
// Initialize light controller
// Parameters: (GPIO_pin, brightness_threshold)
LightController light_controller(GPIO_NUM_3, 80);  // 80 = threshold value

// In your detection/frame processing loop:
while (running) {
    // ... your frame capture code ...
    
    // Update light based on frame brightness
    bool light_on = light_controller.update_light(img);
    
    // Optionally log brightness
    uint8_t brightness = light_controller.get_last_brightness();
    printf("Brightness: %d, Light: %s\n", brightness, light_on ? "ON" : "OFF");
}
```

### 3. Configuration Parameters

#### Brightness Threshold (0-255)

- **Values < 50**: Very bright environment, light rarely needed
- **Values 50-100**: Normal indoor environment
- **Values 100-150**: Darker environment, light frequently on
- **Values 150+**: Very dark environment, light almost always on

**Example:**
```cpp
LightController light(GPIO_NUM_3, 100);  // Turn ON when brightness < 100
```

#### GPIO Pin Selection

```cpp
// ESP32-S3-EYE
LightController light(GPIO_NUM_3, 80);    // GPIO3 - Available for LED/light control

// ESP32-S3 (generic)
LightController light(GPIO_NUM_4, 80);    // GPIO4
LightController light(GPIO_NUM_5, 80);    // GPIO5
```

## API Reference

### Constructor
```cpp
LightController(gpio_num_t gpio_pin, uint8_t brightness_threshold);
```

### Main Methods

```cpp
// Calculate brightness from frame
uint8_t calculate_brightness(const dl::image::img_t &img);

// Automatically control light based on brightness
bool update_light(const dl::image::img_t &img);

// Manual light control
void set_light(bool on);

// Query current state
uint8_t get_last_brightness() const;
bool is_light_on() const;
uint8_t get_threshold() const;
void set_threshold(uint8_t threshold);
```

## Usage Examples

### Example 1: Basic Integration

```cpp
#include "light_controller.hpp"

LightController light(GPIO_NUM_3, 100);

while (true) {
    // ... capture frame to 'img' ...
    
    // Automatically turn light on/off based on brightness
    light.update_light(img);
    
    // Log brightness every 30 frames
    static int count = 0;
    if (++count % 30 == 0) {
        printf("Brightness: %d\n", light.get_last_brightness());
    }
}
```

### Example 2: Manual Threshold Adjustment

```cpp
// Start with initial threshold
LightController light(GPIO_NUM_3, 100);

// ... some time later, adjust based on feedback ...
light.set_threshold(120);  // Require darker environment to turn light on

// Later: check current settings
printf("Current threshold: %d\n", light.get_threshold());
```

### Example 3: Debug Mode

```cpp
LightController light(GPIO_NUM_3, 100);

while (true) {
    light.update_light(img);
    
    // Print detailed info for debugging
    uint8_t brightness = light.get_last_brightness();
    bool is_on = light.is_light_on();
    uint8_t threshold = light.get_threshold();
    
    printf("Brightness: %3d | Threshold: %3d | Light: %s\n",
           brightness, threshold, is_on ? "ON " : "OFF");
}
```

## Brightness Values Guide

The brightness value (0-255) represents the average luminance of the camera frame:

| Brightness | Condition | Light Status |
|-----------|-----------|--------------|
| 0-30 | Complete darkness | Light ON |
| 31-80 | Very dark/dim | Light ON |
| 81-150 | Normal lighting | Depends on threshold |
| 151-200 | Bright | Light OFF (likely) |
| 201-255 | Very bright | Light OFF |

## Troubleshooting

### Light doesn't turn on/off
1. Check GPIO pin is correctly specified
2. Verify GPIO is not already used by other components
3. Check external light circuit (relay/MOSFET)
4. Lower the brightness threshold to test

### Brightness values too low/high
- Adjust `BRIGHTNESS_THRESHOLD` in your code
- Lower threshold = light turns on more easily
- Higher threshold = light turns off more easily

### Light flickers
- The code has hysteresis built in (light state only changes when threshold is crossed)
- If still flickering, increase threshold range or add temporal filtering

### GPIO conflicts on ESP32-S3-EYE
- GPIO3: Module Power LED (open-drain mode)
- Other options: Check your board schematic for unused GPIOs
- Some boards may need level shifter for 3.3V control signal

## Building and Running

### With standard app_main.cpp:
```bash
idf.py -DSDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_s3_eye set-target esp32s3
idf.py build
idf.py flash
```

### With example app_main_with_light_control.cpp:
Replace `app_main.cpp` content or compile the reference example

## Notes

- Brightness calculation samples every 4th pixel for performance
- RGB565 frame format is automatically handled
- Luminance formula: Y = 0.299*R + 0.587*G + 0.114*B (standard)
- GPIO output is active-HIGH (1=ON, 0=OFF) - adjust in `set_light()` if needed
