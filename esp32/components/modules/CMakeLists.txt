set(src_dirs
                camera
                lcd
                imu
                esp-dsp
                model
                algorithm
                utils)

set(include_dirs
                    camera
                    lcd
                    imu
                    esp-dsp
                    model
                    algorithm
                    utils)

set(requires    
                esp32-camera
                bus
                screen
                fb_gfx
                tflite-lib
                esp_timer
           
                nvs_flash

                )

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3)
project(NYC_pedestrian_counter)