idf_component_register(SRCS 
    lora.c
    INCLUDE_DIRS 
        include
        .
)

# Add additional source files here if needed

# If you have any additional libraries to link with, you can add them using target_link_libraries
# For example, if you need to link with the SPI library, uncomment the following line:
# target_link_libraries(${COMPONENT_LIB} PRIVATE "-lspi")

# If you have any additional compiler flags to set, you can add them using target_compile_options
# For example, if you need to set a specific optimization level, uncomment the following line:
# target_compile_options(${COMPONENT_LIB} PRIVATE "-O2")

# If you have any additional dependencies, you can add them using target_link_libraries
# For example, if you need to link with a specific library, uncomment the following line:
# target_link_libraries(${COMPONENT_LIB} PRIVATE your-library)

