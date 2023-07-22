#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>
#include "TheThingsNetwork.h"

#include "config.h"

/* Camera and YOLO setup */

#include "app_lcd.h"
#include "app_camera.h"
#include "algo_yolo.hpp"
static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueLCDFrame = NULL;

// void runCameraYOLO(void *pvParameter)
// {
//     static QueueHandle_t xQueueAIFrame = NULL;
//     static QueueHandle_t xQueueLCDFrame = NULL;

//     while (1)
//     { // Added while loop
//         if (xQueueAIFrame == NULL && xQueueLCDFrame == NULL)
//         {
//             xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
//             xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

//             register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
//             int result = register_algo_yolo(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
//             // register_lcd(xQueueLCDFrame, NULL, true);
//             // return;
//         }

//         // Process camera and YOLO code here...
//         // vTaskDelay(10 / portTICK_PERIOD_MS); // Add a delay to allow other tasks to run
//     }
// }

extern "C"

{

    void app_main(void)
    {

        // xTaskCreatePinnedToCore(runCameraYOLO, "camera_yolo", 1024 * 4, NULL, 4, NULL, 0); /* Camera and YOLO */

        xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
        xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

        register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
        register_algo_yolo(xQueueAIFrame, NULL, NULL, NULL, true);
    }
}