idf_component_register(
    SRCS 
        "main.cpp"
    
    INCLUDE_DIRS
        "../lib"
    
    REQUIRES
        freertos
        driver
        esp_driver_i2c
        esp_timer
        esp_driver_sdspi
        fatfs
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -fstack-usage
    -Wstack-usage=2048
)
