Bum Biter Bot MK 2.0
*   09 Apr 2022 RM - Installed SerialTranserfer
                   - Code and comments clean up. 
                   - Migrated code to github.
                   - changed default serial port to ttyACM1 in platformio.ini

*   15 Mar 2022 RM - Implemented new encoder counter functions to increse the resoultion of the counts. I'm getting 4 times more! 8D
                      |-> Made new backup before began. 
                      |-> https://github.com/vinay-lanka/navbot_hardware/blob/master/encodertest/encodertest.ino
                    
*   13 Mar 2022 RM - Continued Troubleshooting of new shield. Success. 
*   11 Mar 2022 RM - changing PINS to accomodate for new board shield configuration 

*   06 Mar 2022 RM - Add ability Turn PID on and off with controler Select and Start for now.
                      - Begin setup BlueTooth Terminal mobile for feedback with pid tuning etc.

*   05 Mar 2022 RM - Added clip limiting function and installed PID libaray.
                    - working on PID implementation

*   04 Mar 2022 RM - Motor function changes
                      -> the 3 functions with control of the motor set the global variables for rotation. 
                   -Fixed up the motor debug info output.
                   
 *  03 Mar 2022 RM - Moved wheel encoder reading to a service used an atomic_block for less erratic results from reading the interrupt driven variables.
                      - Working on PID control.
                        - Got RPM count working. It could probably be better.
                        - Added distance and speed calc using wheel measurments                        

 *  02 Mar 2022 RM - Added wheel encoder counters and related functions.
                   - Noticed DEBUG printing to conlog was getting close to maxstack. increased it to 128 (maxstack*2 )
 *  28 Feb 2022 RM - Adding Second array of Rear facing sensors. Sharp Digital IR 20CM  and HC-SR04. Pins. 
                   - Moved analog IR sensors to left and right.
                   - Expanded power distribution board to accomodate more sensors.
                   
 *  27 Feb 2022 RM - Adding HM-10 V3 BT 4.0 on Serial3 using Dabble BT library
 *  24 Feb 2022 RM - Adding Analog sharp IR sensor to Analog pin 0. note the alias for A0 is PIN_A0 and not simple 
 *
 *  17 -23 Feb 2022 RM  -  created ping and cruise/drive tasks with associated functions plus a terrible first POC avoid function.
 *  
 *  15 Feb 2022 RM Created. 
 *      - Acknowledgements. This was code is shamelssly stolen from "Test of LMX multi-tasking on ARDUINO" by David P. Anderson https://github.com/dprg/lmx
 *        The nickname was given by my daughter Alyssa who said she wanted a robot that would patrol the house and bite bad guys in the bum but also be fun and bring you stuff.
 *      Hardware: 
 *            - Arduino Mega 2560. 
 *            - TB6612fng Motor Driver with 2x Encoder TT motors (wired in a fliped/mirrored configuration because why suffer?.)
 *            - Custom built power distriubtion board.
 *            - HC-SRO4 Ultrasonic distance Sensor fwd and rear
 *            - Sharp GP2Y0A41SK0F IR distance sensor 4 - 30CM  - x 2 located left and right forward of wheels. 
 *            - Sharp GP20D21 21CM Digital IR Distance sensor mounted fwd as a backup to Ultrasonic as some materials and situations are blind to one or ther other. 
 *            - Not yet moved from mk1 is the Passive Piezo speaker for 8bit jams
 *            - HM-10 V3 BT 4.0 on Serial3   

 *     Software:
 *            - libtask LMX from D.P. Anderson as a scheduler 
 *            - Stole this IR sensor lib https://github.com/qub1750ul/Arduino_SharpIR 
 *            - Currently Porting from Mk1 hardware. Unamed custom written TB6612fng Motor Driver functions that should probably move to a inc or lib later.
 *            - Blue Tooth interface using libraries from Dabble https://thestempedia.com/docs/dabble/game-pad-module/
 *            - SerialTransfer https://github.com/PowerBroker2/SerialTransfer
 *          
 * 
 *  TO-DO List: 
 *            - Convert all dependencies to PlatformIO libdeps.
 *            - Move documentation and activit log out of source code.
 *            - Add/fix ability to send some console DEBUG log info back over serial BT console.
 *            - Add visual feedback RGB LEDs
 *            - Add Speaker. 
 *            - BUG. When Arduino reboots BT connection is problematic.
 *              |-> Connect the appropriate reset ping to the BT module and have it restart when the sketch does.
 *                |-->Make sure above is run time configurable option ALSO a tirggerable event.
 *            - BUG. Time out functioning required for HC-SRO4. Default of 1000ms causes issues if the conection faults.  
 *  
 */
/* -------------------------------------- */
