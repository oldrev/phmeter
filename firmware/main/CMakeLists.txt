idf_component_register(
    SRCS "main.c" "mcp3201.c"
    INCLUDE_DIRS ""
    REQUIRES
        freertos
        driver
        libq       
)
