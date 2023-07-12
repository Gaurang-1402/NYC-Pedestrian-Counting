# P2P LoRaWAN Communication

Simple send or receive.   


# Lora module hardware configuration

The recommended lora extension module is the RFM915W shown here: https://www.amazon.com/Lubeby-Smart-Original-915Mhz-Transceiver/dp/B0BHLK9FC9


Here is the schematic of XIAO ESP32S3
![image](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/4fc59de3-7e9c-491b-b2c7-6e44613497d4)

Here is the schematic of the RFM915W

![image](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/42063aad-0dc3-48f7-91b5-c163840614e0)

Note: You will have to attach an antenna to the Lora module.

Here is the recommended pin configurations

Pin | Signal
--- | ------
CS | GPIO1
RST | GPIO2
MISO | GPIO8 
MOSI | GPIO9
SCK | GPIO7

We will also have to set the pin configuration in the software to correspond to the hardware

Run

```
idf.py menuconfig
```

Now, pick "LoRa Configuration" option

![Screenshot from 2023-07-04 12-32-44](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/6b3f1fde-7462-482e-a148-9ad49241c606)

Here is what it should look like if you follow the above hardware configuration

![Screenshot from 2023-07-04 12-33-33](https://github.com/Gaurang-1402/NYC-Pedestrian-Detection/assets/71042887/6a6d49f2-4469-44d5-9b69-185086f252d2)


# Software Configuration  

You have to make some changes to the menuconfig. Run

```
idf.py menuconfig
```

Then pick "Application Configuration" as an option

![config-top](https://user-images.githubusercontent.com/6020549/152315401-0a3ea44f-d045-4b93-b988-b9ce9451f935.jpg)


Set 


![config-basic-1](https://user-images.githubusercontent.com/6020549/153731778-d7c3e699-5afa-4673-8de0-da9ec8ea6808.jpg)

Set transceiver frequency to 915MHz (open frequency band in North America):   
![config-basic-2](https://user-images.githubusercontent.com/6020549/153731782-dd217bd5-b37c-4d31-b6e8-297d4372ce3c.jpg)


When using advanced settings:   
![config-basic-3](https://user-images.githubusercontent.com/6020549/153731783-da342804-9be9-4b16-9908-4c7aa69419d4.jpg)

LoRa mode has the following three communication parameters.   
1.Signal Bandwidth (= BW)   
2.Error Cording Rate (= CR)   
3.Spreading Factor (= SF)   
The communication speed is faster when BW is large, CR is small, and SF is small.   
However, as the communication speed increases, the reception sensitivity deteriorates, so select the one that best suits your needs.   

- Error coding rate   
001:4/5(Default)   
010:4/6   
011:4/7   
100:4/8   

- Signal Bandwidth   
0001:10.4 kHz   
0010:15.6 kHz   
0011:20.8kHz   
0100:31.25 kHz   
0101:41.7 kHz   
0110:62.5 kHz   
0111:125 kHz(Default)   
1000:250 kHz   
1001:500 kHz   
In the lower band (169MHz), signal bandwidths 8&9 are not supported.   

- SF rate (expressed as a base-2 logarithm)   
6:64 chips / symbol   
7:128 chips / symbol(Default)   
8:256 chips / symbol   
9:512 chips / symbol   
10:1024 chips / symbol   
11:2048 chips / symbol   
12:4096 chips / symbol   


# Throughput (Unit=Bytes/Sec)   
It depends on the payload size.   
|Payload Size|BW=62.5KHz|BW=125KHz|BW=250KHz|BW=500KHz|
|:-:|:-:|:-:|:-:|:-:|
|31Bytes|183.11|345.98|618.76|621.24|
|63Bytes|253.52|422.54|698.87|1254.93|
|127Bytes|284.44|556.53|964.06|1809.06|
|255Bytes|306.34|579.15|1072.33|1835.85|

# Packet Lost Rate (Unit=Percent)   
It depends on the payload size.   
0 is all packet received.   
2 indicates that the two packets did not arrive.   
|Payload Size|BW=62.5KHz|BW=125KHz|BW=250KHz|BW=500KHz|
|:-:|:-:|:-:|:-:|:-:|
|31Bytes|0|0|0|0|
|63Bytes|0|0|1|1|
|127Bytes|0|0|2|1|
|255Bytes|0|1|0|0|

