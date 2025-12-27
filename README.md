Code initially used to create a video game style mini map on a Waveshare ESP32-P4 3.4" as featured in the Garage Tinkering video at https://youtu.be/sAp7oCB939c

Ported to Waveshare ESP32-S3 1.75", with integrated GPS (LC76G GNSS Module with external ceramic antenna)
https://www.waveshare.com/esp32-s3-touch-amoled-1.75.htm?sku=31264

Requires offline maps provided as a tiled grid of LZ4 compressed .bin images on an SDCard - recommended zoom level 18
Default structure: `tiles1/z/x/y.bin`

Code is free to use and modify as you like, but not for commercial
CC BY-NC (Attribution-NonCommercial)
