# Arduino-Manchester-Encoder
Low level Manchester encoder for Arduino boards and simple RF modules

Sample code for 
Arduino Manchester encoding in FSK for RFM69 on Adafruit Feather M0
Requires Radiohead library
http://www.airspayce.com/mikem/arduino/RadioHead/

Based very loosely on the (more elegant)
https://github.com/mchr3k/arduino-libs-manchester

Whilst the RFM69 family has built-in packet handler for Manchester encoding, this sample code uses the continuous mode and pin modulation to create manchester encoded FSK.

It should therefore be straightforward to adapt to other radio modules with pin modulation in either FSK, ASK or OOK
