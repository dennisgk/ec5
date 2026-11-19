idf_component_register(
    SRCS "app.c" "motor.c" "emg.c" "capture_bridge.c" "signal_processing.c"
         "command_dispatch.c" "telemetry.c" "serial_frame.c" "serial_transport.c" "control.c" "ble_control.c" "autoconfig.c" "autoconfig_schema.c"
    INCLUDE_DIRS "."
    REQUIRES driver ulp esp_hw_support nvs_flash esp_timer esp_rom esp_driver_usb_serial_jtag bt esp_driver_sdspi fatfs sdmmc json
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE SERIAL_ASSEMBLY_MAX=1024)

ulp_embed_binary(lp_capture "lp_core/main.c" "capture_bridge.c")
