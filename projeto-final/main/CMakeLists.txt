idf_component_register(
    SRCS
        "main.c"
        "joystick.c"
        "mpu6050.c"
        "servo.c"
        "status.c"
        "system_common.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_adc
)
