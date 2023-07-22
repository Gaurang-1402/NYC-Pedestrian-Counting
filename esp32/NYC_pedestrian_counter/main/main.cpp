#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>
#include "TheThingsNetwork.h"
#include "esp_wifi.h"
#include "esp_system.h"


#include "config.h"

/* Camera and YOLO setup */

#include "app_lcd.h"
#include "app_camera.h"
#include "algo_yolo.hpp"


/* LoRa setup */

// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'idf.py menuconfig'.
// Please read the wiki on Lora to learn how to do this.

// AppEUI (sometimes called JoinEUI)
const char *appEui = APPEUI;
// DevEUI
const char *devEui = DEVEUI;
// AppKey
const char *appKey = APPKEY;

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
int newCount = 0;

uint8_t mac[6];
char boardId[18];  // MAC address will be 17 characters long

void sendMessages(void* pvParameter)
{
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    while (1) {
        printf("Sending message...\n");

        /* TODO: this is an example, replace these with actual values */
        int newCount = 42;

        /* Prepare the payload */

        /*
        In this code, we're sending a payload that is 7 bytes long: the first 6 bytes are the MAC address and the last byte is the count.
        This is a very compact representation that fits well within the constraints of LoRaWAN.
        On the receiving side (TTN), we receive a 7-byte array.
        We need to parse this back into a MAC address and count.
        Note: This example assumes that that count is in the range 0-255.
        If it can be larger, we may need to use more bytes to represent it, and adjust the code accordingly.
        */
        uint8_t payload[7];
        memcpy(payload, mac, 6);
        payload[6] = newCount;

        /* Send the payload */
        TTNResponseCode res = ttn.transmitMessage(payload, sizeof(payload));

        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

        vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
    }
}



void messageReceived(const uint8_t* message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

extern "C"

{

    void app_main(void) {
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    memset(&spi_bus_config, 0, sizeof(spi_bus_config));
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    // Register callback for received messages
    ttn.onMessage(messageReceived);

    /* TODO: set these parameters according to requirements */
    //    ttn.setAdrEnabled(false);
    //    ttn.setDataRate(kTTNDataRate_US915_SF7);
    //    ttn.setMaxTxPower(14);

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }

}

}