# TTN LoRaWAN Communication

It is recommended that you read from and understand the following links:
1) https://github.com/manuelbl/ttn-esp32/wiki
2) https://circuitdigest.com/microcontroller-projects/esp32-lora-communication-with-the-things-network

# Lora module hardware configuration

The recommended lora extension module is the Adafruit RFM915W shown here: https://www.adafruit.com/product/3072?gclid=EAIaIQobChMI-NWq5qHz_wIVGkRHAR1iHQ4sEAQYAyABEgLJr_D_BwE

Here is the schematic of XIAO ESP32S3
![image](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/4fc59de3-7e9c-491b-b2c7-6e44613497d4)

Here is the schematic of the RFM915W
![3072-14 (1)](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/311e4f96-4bf4-4c81-a29a-653d9583510b)


The "G" in "G0", "G1", "G2" etc stand for "GPIO"

Note: You will have to attach an antenna to the Lora module.

Here is the recommended pin configurations

Pin | Signal
--- | ------
CS | GPIO1
RST | GPIO2
MISO | GPIO8 
MOSI | GPIO9
SCK | GPIO7
G0 | GPIO4
G1 | GPIO3

We will also have to set the pin configuration in the software to correspond to the hardware


## Set the target

Set the target to the correct variant of the ESP32 chip (*esp32*, *esp32s2*, *esp32s3* etc.):

```
idf.py set-target esp32s2
```


## Configure the frequency plan (region)

The region and frequency plan must be configured according to your country and device. We need to select US915.

From the NYC_pedestian_counter directory, run:

```
idf.py menuconfig
```

In the menu, select `Component config --->` and then select `The Things Network --->` (at the very bottom). Now select `TTN LoRa frequency / region (LoRa disabled) --->` and choose the correct frequency plan. Then select `Exit` several times and confirm that the new configuration should be saved.

This should be the final setup in your menu:

![Screenshot from 2023-07-12 23-23-17](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/f9332487-8f0e-4971-ab0f-43c4fd4a292f)



## Register your device with TTN

In the [TTN console](https://console.cloud.thethings.network/) go to your application (or create one if needed), select the *Devices* tab and click *register device*:

![Register new device](images/register_device.png)

First select the brand, model etc. of your device in the section *Enter end device specifics manually*.

Select the following options

![Screenshot from 2023-07-12 23-16-36](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/2e17768c-d758-4195-81c7-8c1ae4a7358c)


The AppEUI (also called JoinEUI), the DevEUI and the AppKey should be provided by the device manufacturer. If so, enter them. If not, kill *Fill with random numbers* for the AppEUI and click *Generate* for the DevEUI and AppKey.

Finally finish by clicking *Register end device*.

The device will be registered and the overiew page will be displayed:

![Copy device EUI](images/copy_dev_eui.png)

For each of AppEUI, DevEUI and AppKey, copy the hexadecimal value by clicking the copy icon after the value and paste it into `config_template.h` to replace the question marks in the below line:

```cpp
// AppEUI (sometimes called JoinEUI)
#define APPEUI "????????????????"
// DevEUI
#define DEVEUI "????????????????"
// AppKey
#define APPKEY "????????????????????????????????"
```

Rename the file to `config.h`

*Please copy your own values and not the example values above.*


## Configure the pins

Go to `config.h` and change the pin numbers where needed:

```cpp
// Pins and other resources
#define TTN_SPI_HOST      SPI2_HOST
#define TTN_SPI_DMA_CHAN  SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK  7
#define TTN_PIN_SPI_MOSI  9
#define TTN_PIN_SPI_MISO  8
#define TTN_PIN_NSS       1
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       2
#define TTN_PIN_DIO0      4
#define TTN_PIN_DIO1      3
```

See the above diagram and table for the pin configuration.


## Build and flash the app

Connect your device to the USB port and run:

```
idf.py -p /dev/cu.usbserial build flash monitor
```

Instead of _/dev/cu.usbserial_, insert your serial port (on Windows it will look like _COM3_ on Linux it will be /dev/tty**).

The app should be built, uploaded to your device and the app's output should be shown in your terminal window:

```
me@MacBook hello_world % idf.py -p /dev/cu.usbserial-0001 build flash monitor
Executing action: all (aliases: build)
Running ninja in directory /Users/me/Documents/hello_world/build
Executing "ninja all"...
[0/1] Re-running CMake...
-- Building ESP-IDF components for target esp32
-- Project sdkconfig file /Users/me/Documents/hello_world/sdkconfig
-- App "hello_world" version: v3.2.0-22-g5c1db03

...

Writing at 0x0003e23b... (77 %)
Writing at 0x00043b92... (88 %)
Writing at 0x0004997c... (100 %)
Wrote 239392 bytes (133071 compressed) at 0x00010000 in 3.4 seconds (effective 562.8 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Executing action: monitor
Running idf_monitor in directory /Users/me/Documents/hello_world
--- idf_monitor on /dev/cu.usbserial-0001 115200 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2

...

I (309) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (355) ttn_hal: IO initialized
I (355) ttn_hal: SPI initialized
I (355) ttn_hal: Timer initialized
I (385) ttn_prov: DevEUI, AppEUI/JoinEUI and AppKey saved in NVS storage
Joining...
I (385) ttn: event EV_JOINING
I (6675) ttn: event EV_TXSTART
I (11705) ttn: event EV_RXSTART
I (11815) ttn: event EV_JOINED
Joined.
Sending message...
I (11815) ttn: event EV_TXSTART
I (16845) ttn: event EV_RXSTART
I (17835) ttn: event EV_RXSTART
I (17925) ttn: event EV_TXCOMPLETE
Message sent.
```

## Check the received message in the TTN console

Go to the [TTN console](https://console.cloud.thethings.network/), select your *region*, click *Applications*, select your application, click *End devices*, select your device and click on the *Live data* tab. After at most 30 seconds, the first message should appear and further messages should arrive about twice a minute:

![Received messages](images/recv_messages.png)

The payload should read the pedestrian count.


