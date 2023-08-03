
#include "app_lcd.h"
#include "app_camera.h"
#include "algo_yolo.hpp"
#include "dsp_platform.h"
#include "esp_dsp.h"
#include "ekf.h"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueLCDFrame = NULL;

extern "C" void app_main()
{

  xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
  xQueueLCDFrame = xQueueCreate(1, sizeof(camera_fb_t *));

  register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
  register_algo_yolo(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
  register_lcd(xQueueLCDFrame, NULL, true);
}
