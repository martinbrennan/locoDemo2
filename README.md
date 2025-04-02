This is a minimal example program to show off the capabilities of the Loco Software Library
It runs on the Loco V4 PCB

Find out more at www.brennan.co.uk/loco

The program is built using Espressifs ESP-IDF framework.

Instructions to install ESP-IDF and build the program can be found here

https://brennan.co.uk/pages/esp-idf-installation-and-build

Before building you need to set MYSSID and MYPASSWORD for your Wifi network

When Loco is connected to the Wifi network you can see it if you tap the devices icon on the Spotify app.
Loco has the name "Beauty" - like the quark - you can change this in main.c

You can play Spotify through Loco using its Spotify Connect capability.
Spotify Connect is available to the 260 million Spotify subscribers. It is not available on the free tier.

There is a simple command line interface that responds to lines typed in the ESP-IDF monitor.

for example 

tt followed by enter will turn the test tone generator on and off

You can see the all the commands in the function exCli in main.c
exCli is a good place to add your own functions and experiment with the loco Software library










