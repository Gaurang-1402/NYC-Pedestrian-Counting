# NY-Pedestrian-Detection

Some of the code is adapted and modified from: https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera 


Important commands

```
get_idf
```

Windows

```
.\esptool.exe --chip esp32s3  --no-stub --port COM6 --baud 460800  --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 yolo.bin
```

Linux
```
esptool --chip auto --no-stub --port /dev/ttyACM0 --baud 460800  --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 yolo.bin


```
