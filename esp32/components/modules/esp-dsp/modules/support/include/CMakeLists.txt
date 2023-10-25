file(GLOB_RECURSE srcs 
    "*.cpp"
    "*.c"
    "*.S"
    "*.h"
)

if(target STREQUAL "esp32s3")
    file(GLOB_RECURSE extra_srcs 
        "../modules/support/mem/*"
    )
    list(APPEND srcs ${extra_srcs})
endif()

set(COMPONENT_SRCDIRS ".")

set(COMPONENT_REQUIRES unity esp-dsp)

set(COMPONENT_ADD_LDFRAGMENTS linker.lf)

register_component()
