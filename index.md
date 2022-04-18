# BumBiter Mk 2
-BB Mk 2

### Acknowledgements. 
At the core of this build is a task scheduler called libtask which was shamelessly stolen from
"Test of LMX multi-tasking on ARDUINO" by David P. Anderson https://github.com/dprg/lmx

### The Name 
The nickname for the bot was given by my daughter Alyssa who said she wanted a robot that would patrol the house and bite bad guys in the bum but also be fun and bring you stuff. 

The initial build, the ***BumBiter Mk1***, was initially built on Arduino Uno (later Nano) hardware. Lack of multitasking lead to the search that eventually rested on libtask/LMX and I upgraded to the Arduino Mega 2560 because that's I already had.

This represents a WORK IN PROGRESS and a fun learning project.

**Current status** is that Tele-operation is possible with the Dabble Bluetooth game pad app. Bluetooth terminal commands work as well so making something respond to a voice typed command could be simple. however that has not been done.

ROS2 Integration is being developed by having this hardware exchange messages with a WiFi enabled board running micro-ROS. In this case an Arduino Nano RP2040 Connect.

The griper arm is not assembled.
  
Many more enhancements are in the dream pipe.
