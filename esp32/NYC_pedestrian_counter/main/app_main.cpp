
// #include "app_lcd.h"
// #include "app_camera.h"
// #include "algo_yolo.hpp"

// static QueueHandle_t xQueueAIFrame = NULL;
// static QueueHandle_t xQueueLCDFrame = NULL;

// extern "C" void app_main()
// {

//   xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
//   xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

//   register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
//   register_algo_yolo(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
//   register_lcd(xQueueLCDFrame, NULL, true);

// }


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

void task_tx(void *p)
{
   for(;;) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      lora_send_packet((uint8_t*)"Hello", 5);
      printf("packet sent...\n");
   }
}

void app_main()
{
   lora_init();
   lora_set_frequency(915e6);
   lora_enable_crc();
   xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
}
