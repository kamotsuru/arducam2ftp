# ArduCAM to FTP
## 0. Motivation
I just wanted to copy an image taken by my ArduCAM-Mini-2MP with ESP32 to my server whenever two successive images significantly differs from each other, but most examples only show how to store the images to an SD card...

## 1. Take an ArduCAM image as jpg format
The following example includes taking a jpg image by ArduCAM-Mini-2MP and storing it to a buffer.
https://github.com/Dasch0/esp32-arducam-edge-impulse/blob/main/src/main.cpp


## 2. Decode a jpg image and calculate the difference between two images
Tiny JPEG Decompressor can decode a jpg image with a small footprint. Then, each pixel in the decoded jpg image is compared between two successive images.
http://elm-chan.org/fsw/tjpgd/en/appnote.html


## 3. Put a jpg image to an FTP server
The following is to copy a particular object to an FTP server.
https://playground.arduino.cc/Code/FTP/

I combined these three examples to one which sends a taken jpg image to an FTP server as follows.