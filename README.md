# BB Mk 2

Right now I'm updating the [BB-Mk2 Site](https://redstone-rm.github.io/BB-Mk2/

 ### Acknowledgements. 
libtask is the task manager at core of this build. This was code is shamelessly stolen from

"Test of LMX multi-tasking on ARDUINO" by David P. Anderson https://github.com/dprg/lmx

The nickname was given by my daughter Alyssa who said she wanted a robot that would patrol the house and bite bad guys in the bum but also be fun and bring you stuff.

The initial build, the ***BumBiter Mk1***, was initially built on Arduino Uno (later Nano) hardware. Lack of multitasking lead to the search that eventually rested on libtask/LMX and I upgraded to the Arduino Mega 2560 because that's what I already had.

This represents a WORK IN PROGRESS and a fun learning project.

Current status is that Tele-operation is possible with the Dabble Bluetooth game pad app. Bluetooth terminal commands work as well so making something respond to a voice typed command could be simple. however that has not been done.

ROS2 Integration is being developed by having this hardware exchange messages with a [WiFi enabled board running micro-ROS](https://github.com/Redstone-RM/NanoNode). In this case an Arduino Nano RP2040 Connect. [Read here about how to set that up](https://gist.github.com/Redstone-RM/0ca459c32ec5ead8700284ff56a136f7).
  
Many more enhancements are in the dream pipe.
The build documents, pictures and videos are coming together as work is completed.


 ### Hardware: 
                
- Arduino Mega 2560. 
- TB6612fng Motor Driver with 2x Encoder TT motors (wired in a fliped/mirrored configuration because why suffer?.)
- Custom built power distriubtion board.
- HC-SRO4 Ultrasonic distance Sensor fwd and rear
- Sharp GP2Y0A41SK0F IR distance sensor 4 - 30CM  - x 2 located left and right forward of wheels. 
- Sharp GP20D21 21CM Digital IR Distance sensor mounted fwd as a backup to Ultrasonic
- Not yet moved from mk1 is the Passive Piezo speaker and tone lib for 8bit jams
- HM-10 V3 BT 4.0 on Serial3
- Adding soon 4DOF gripper.   

Software:
- libtask LMX from D.P. Anderson as a scheduler 
- Stole this IR sensor lib https://github.com/qub1750ul/Arduino_SharpIR 
- Currently Porting from Mk1 hardware. Unamed custom written TB6612fng Motor Driver functions that should probably move to a library later.
- Blue Tooth interface using libraries from Dabble https://thestempedia.com/docs/dabble/game-pad-module/
- SerialTransfer https://github.com/PowerBroker2/SerialTransfer
    For passing ROS2 messages between the control boards.(In development.)

A platformio.ini file is included with this repo, Mine looked like this.


```
 [env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
upload_port = /dev/ttyACM1
monitor_speed = 57600

lib_deps = 
    https://github.com/PowerBroker2/SerialTransfer
    https://github.com/STEMpedia/Dabble
    https://github.com/qub1750ul/Arduino_SharpIR
    Wire
```
         