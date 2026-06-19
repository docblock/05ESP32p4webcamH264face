idf_component_register(SRCS "main.c"
                             "wifi_manager.c"
                             "camera_controller.c"
                             "image_processor.c"
                             "mjpeg_server.c"
                             "face_detect_task.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_mm esp_driver_isp esp_driver_cam esp_driver_i2c sensor_init esp_h264 esp_wifi nvs_flash esp_http_server esp_driver_ppa esp_timer esp_new_jpeg human_face_detect
                       )

# Force model binary data into the final binary.
# --gc-sections eliminates .rodata.embedded sections because the reference chain
# through virtual dispatch is not visible to the linker GC. Adding --undefined
# makes these symbols GC roots so their sections are preserved.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--undefined=_binary_human_face_detect_espdl_start"
)
