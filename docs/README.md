# Honda Radio Code Finder
## Overview
This is an Arduino sketch which interfaces with a Honda radio where normally the LCD face plate would plug in. It mimics pressing the buttons at the screen to enter the code and brute force guesses all the codes until it finds the correct one. It has only been used for 03-07 Honda Accords, and among those only certain radio models (7BK0). As far as hardware I have only tested it on an Arduino Mega2560. 

The original goal was to be able to test the radios before reselling them, and to test them I needed the radio code. While you can just [contact Honda](https://radio-navicode.honda.com) for this, I didn't want to be repeatedly requesting codes. Since I've created this I've found out that there are [other websites](https://hondaradiocodes.com/) that will just tell you the code from the serial number, so this project doesn't have much practical use.  

## Hardware Interface 
On the radios I've looked at there are 2 single row blade pin connectors. I wasn't able to figure out what mating connector would work and didn't want to desolder the mating connector to the front panel, so I made jumper wires using these [Molex pins.](https://www.molex.com/molex/products/part-detail/crimp_terminals/1053002200) I used those since i needed something small to fit between the 1.27(ish) mm blade connectors.
### Pinout
The interface between a Honda radio and the face plate is basically a SPI interface with some notable differences.
The serial connections are labeled on the silkscreen as well as the ground and power button, all of which are needed for the sketch. Format is connector#.pin# on the radio side where pin 1 is marked on the silkscreen but atleast on 7BK0. Pin 1 connector 1 is the left most pin when staring at the connectors with the radio label side up.  

|Signal  |LCD-DI  |LCD-DO  |LCD-CK  |LCD-CE  |PWD-SW |GND  
|---     |---     |---     |---     |---     |---    |---   
|Arduino |51      |50      |52      |21      |22     |GND        
|7BK0    |1.2     |1.14    |1.18    |1.16    |2.7    |1.10 (one of several)

## Protocol
While the SPI peripheral on the Arduino can be used, there are 2 main differences that require some extra logic.
### CE signal 
The CE signal which normally encompases an entire block of data is used to separate the command type from the data. 
