file(GLOB srcsCOMP "q/q.c")


idf_component_register(
    SRCS "${srcsCOMP}"
    INCLUDE_DIRS "."
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE "-DQVERSION=0x000900")