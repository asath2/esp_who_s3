set(src_files
    app_main_with_light_control_1
    light_controller.cpp
    frame_cap_pipeline.cpp
)

set(include_dirs ./)

set(requires
    log
    esp_timer
    fatfs
    sdmmc
    vfs
    esp-dl
    esp_lcd
    driver
    who_frame_cap
    esp_driver_rmt
)

set(detect_component ${DETECT_MODEL} espressif__${DETECT_MODEL})

idf_component_register(
    SRCS         ${src_files}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES     ${requires}
)

idf_component_optional_requires(PUBLIC ${detect_component})
