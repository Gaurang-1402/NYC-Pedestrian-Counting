

#include "app_lcd.h"
#include "app_camera.h"
#include "algo_yolo.hpp"

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

/*
DevEUI - 70B3D57ED005F4F0
Device Address - 260CD2E3
AppSKey - 45480AE3382FFD0AB45ADE069C7C973E
FNwkSIntKey - 5269AC96E82FC699B5673F09734CB1C8
SNwkSIntKey - FB75FDB16BEA2F16B6203BEC42E6B865
End device ID - eui-70b3d57ed005f4f0
*/

extern "C" {

	static QueueHandle_t xQueueAIFrame = NULL;
	static QueueHandle_t xQueueLCDFrame = NULL;

	#if CONFIG_SENDER
	void task_tx(void *pvParameters)
	{
		ESP_LOGI(pcTaskGetName(NULL), "Start");
		uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255
		while(1) {
			TickType_t nowTick = xTaskGetTickCount();
			int send_len = sprintf((char *)buf,"Hello World!! %"PRIu32, nowTick);
			lora_send_packet(buf, send_len);
			ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
			vTaskDelay(pdMS_TO_TICKS(5000));
		} // end while
	}
	#endif // CONFIG_SENDER

	#if CONFIG_RECEIVER
	void task_rx(void *pvParameters)
	{
		ESP_LOGI(pcTaskGetName(NULL), "Start");
		uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255
		while(1) {
			lora_receive(); // put into receive mode
			if (lora_received()) {
				int receive_len = lora_receive_packet(buf, sizeof(buf));
				ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", receive_len, receive_len, buf);
			} // end if
			vTaskDelay(1); // Avoid WatchDog alerts
		} // end while
	}
	#endif // CONFIG_RECEIVER

	void app_main()
	{
		if (lora_init() == 0) {
			ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
			while(1) {
				vTaskDelay(1);
			}

		}

		xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
		xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));

		register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
		register_algo_yolo(xQueueAIFrame, NULL, NULL, xQueueLCDFrame, false);
		register_lcd(xQueueLCDFrame, NULL, true);

		ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
		lora_set_frequency(915e6); // 915MHz

		lora_enable_crc();

		int cr = 1;
		int bw = 7;
		int sf = 7;
	#if CONFIF_ADVANCED
		cr = CONFIG_CODING_RATE
		bw = CONFIG_BANDWIDTH;
		sf = CONFIG_SF_RATE;
	#endif

		lora_set_coding_rate(cr);
		//lora_set_coding_rate(CONFIG_CODING_RATE);
		//cr = lora_get_coding_rate();
		ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

		lora_set_bandwidth(bw);
		//lora_set_bandwidth(CONFIG_BANDWIDTH);
		//int bw = lora_get_bandwidth();
		ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

		lora_set_spreading_factor(sf);
		//lora_set_spreading_factor(CONFIG_SF_RATE);
		//int sf = lora_get_spreading_factor();
		ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

	#if CONFIG_SENDER
		xTaskCreate(&task_tx, "task_tx", 1024*3, NULL, 5, NULL);
	#endif
	#if CONFIG_RECEIVER
		xTaskCreate(&task_rx, "task_rx", 1024*3, NULL, 5, NULL);
	#endif
	}
} // extern "C"
