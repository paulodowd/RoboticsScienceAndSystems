# Stamp C3 Template

This is some work in progress to provide a class to operate this M5Stack C3 extension board.  It currently handles i2c transactions to a Pololu 3Pi+.  

## Freenove_WS2812_Lib_for_ESP32

Within this code template I've extracted and used core parts of the Freenove_WS2912_Lib_for_ESP32 library. I've done this to avoid the necessity of asking students to install a library when they first approach using this extension board.  The StampC3 device only has 1 RGB LED, so the full Freenove_ library is a bit excessive.  I've included the original license supplied with Freenove_, and the original attributions and source can be found at https://github.com/Freenove/Freenove_WS2812_Lib_for_ESP32/
