idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_usb_serial_jtag esp_hw_support esp_timer
)

# The fixed-rate capture loop must stay short and deterministic regardless of
# the optimization level selected for the rest of the project.
target_compile_options(${COMPONENT_LIB} PRIVATE -O3)
