/* -------------------------------------- */
/* 
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

/* -------------------------------------- */
#define VERSION "Bum Biter MK-2.1.1"
#define DEBUG true

/* -------------------------------------- */
#include <Arduino.h> // Only Needed for PlatformIO.
#include <util/atomic.h> // Using ATOMIC_BLOCK macro for wheel encoder reading of volitile ints.

// LMX 
#define BAUDRATE 57600
#define PRINTF Serial.println
#define SPRINTF sprintf
#include <stdio.h> // LMX
#include <task.h> // LMX
#include <log.h> // LMX
#include <sysclock.h> // LMX

// Sensors
#include <SharpIR.h> // IR Distance Sensors
#include <PID_v1.h> // PID https://playground.arduino.cc/Code/PIDLibrary/

// BlueTooth Config
#include <Dabble.h> // Dabble Bluetooth Contoller. https://thestempedia.com/docs/dabble
#define CUSTOM_SETTINGS // You can reduce the size of compiled library by enabling only modules you want to use. First define CUSTOM_SETTINGS followed by #define INCLUDE_modulename
#define INCLUDE_GAMEPAD_MODULE // Include Dabble Gamepad module https://thestempedia.com/docs/dabble/game-pad-module
 
// I2C Datum TRANSFER from ROS2 Controler. see https://github.com/PowerBroker2/SerialTransfer
#include <I2CTransfer.h> 
#define I2C_ADDR 9  // I2C Slave Address when in slave mode. IMPORTANT. SerailTransfer.h defaults addr 0 not configurable AFIAK here ATM .
  // Begin I2C control message structs.
I2CTransfer  myTransfer; // create I2C Transfer Obj

// create crtlmsg and statmsg struct to hold I2c Transfer "datum" Obj 
// ROS2 Controler> ctrlmsg> I2C"datum"> I2C Callback> statmsg> I2C> ROS2> Topic
struct ctrlmsg { 
  float x;
  float z;
  char  debug[8];
} botmsg; 

struct statmsg { 
  float x; // Confirm current requested X value. Feedback 
  float z; // Confirm current requested Z value. Feedback 
  int   mtr_pos_right; // right motor encoder position
  int   mtr_pos_left; // left motor encoder position
  int   mtr_speed_right;  // motor a speed
  int   mtr_speed_left;  // motor b speed
  int   sen_sonar_fwd; // forward sonar value
  int   sen_sonar_rear; // rear sonar value
  int   sen_ir_right; // right IR value
  int   sen_ir_left; // left IR value
  char  debug[8]; // short logging message
} statmsg; // create a status message struct



/* ------------ <dpa> -------------------------- */
void printkbuf(char *s) {  // dpa
   PRINTF(s);
}
/* Choose one of the below:  */ 
// #define WAIT(d)  { d *= 10; cnt = 0; while (cnt++ < d) defer(); }
// #define WAIT(d)  { msleep(d); }
#define WAIT(d)  { wake_after(d); }
/* ------------</dpa> -------------------------- */


/* -------------Definitions ------------------------- */

/*   Pin Definitions   */

#define LED_HEARTBEAT_PIN 13

/*  MOTOR CONTROL  
      Motor setup notes
      MOTOR A = RIGHT
      MOTOR B = LEFT
      FWD = HIGH/LOW
      REV = LOW/HIGH
      BRAKE = HIGH/HIGH
*/ 
// motor A
#define MTRA_ENCA 2 // orange Motor A encoder output A must be on interrupt pin.
#define MTRA_ENCB 3 // red Motor A encoder output on B on an interrupt pin.
#define MTRA_CLICKS_PER 540 // encoder clicks per bot wheel rotation.
#define MTRA_WHEELD 60.3 // wheel D in mm


// motor B
#define MTRB_ENCA 19 // red. Motor B encoder output A must be on interrupt pin. // remember the motors are flipped here and in polarity. Affected encoder counting rotation.
#define MTRB_ENCB 18 // orange.  Motor B encoder output B must be on interrupt pin. // remember the motors are flipped here and in polarity.
#define MTRB_CLICKS_PER  540// encoder clicks per bot wheel rotation.
#define MTRB_WHEELD 60.3 // wheel D in mm

// These Interface with Motor Driver H-Bridge.
// keep in mind the PWM defines must be on PWM pins
              // new pin#    // Old pin#
#define MOTOR_BIN2 6  // 34
#define MOTOR_BIN1 7  // 36
#define MOTOR_STBY 8  // 38
#define MOTOR_AIN1 9  // 40                                       
#define MOTOR_AIN2 10 // 42
#define MOTOR_PWMB 5  // 44 // PWM pin
#define MOTOR_PWMA 11 // 46 // PWM Pin 


// sound control
// #define BEEP xx // hooked to passive piezo buzzer.

// SENSONR PINS 
  //  HC-SR04 Ultra-sonic Distance Sensors
      // FWD 
#define trigPinFwd 46// yellow 
#define echoPinFwd 48 // green. Note that 46 is also PWM capable. 
    // REAR
#define trigPinRear 26 // yellow
#define echoPinRear 28 // green

  // Sharp Analog IR sensor 4-30CM  gp2y0a41sk0f 
#define IRPin_1  PIN_A0 // right
#define IRPin_2  PIN_A1 // left
#define IR_SENSOR_1 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR
#define IR_SENSOR_2 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR
  
// Sharp Digital IR distance Sensors 2YD021 - Disabled Mar 12 2022 RM.
// #define IRPin_FWD 4 
// #define IRPin_Rear 5

/* Layer 1 function alias definitions    */
//  heartbeat LED .
#define LED1_ON  digitalWrite(LED_HEARTBEAT_PIN, HIGH) // onboard LED used for heartbeat.
#define LED1_OFF digitalWrite(LED_HEARTBEAT_PIN, LOW)  // onboard LED used for heartbeat.

// Wake up motor driver from low power standby
#define MOTOR_WAKE digitalWrite(MOTOR_STBY, HIGH) 

/* 
STATE VARIABLES 
scope_type_name
   scope classes :   bot - global to entire bot
                     mtr - motors
                      -  
   
   types :           init  - Initializaion variables
                     ctl   - Controls. Variable inputs set by user or other nodes.
                     sen   - Sensor Data. Readings from various sensor processing tasks.
                     cal   - Calculating variables. Raw vars used in other calcuations
                     cfg   - Configureable stuff.
                     

*/
int bot_init_runlvl  = 5; // Default init level.
char bot_sys_debug[] = "Hello World"; // string to hold debug output. For Serial or logging

String Serialdata = ""; // Output String object for BlueTooth Serial Terminal output.
bool dataflag = 0;

/********** MOTOR GLOBALS **********/
// Motor CONTORL
int mtr_ctl_speed = 150; // CFG > default drive speed. tweak these as req either here or runtime
int mtr_ctl_pivot_speed = 150; // CFG > default pivot turn speed. 
int mtr_ctl_speed_a = 0;  // to set motor a speed
int mtr_ctl_speed_b = 0;  // set motor b speed

// Motor Virtual Sensors
volatile long mtr_cal_a_encoder0Pos = 0;    // MTR A encoder 
volatile long mtr_cal_b_encoder1Pos = 0;    // MTR B encoder 

int mtr_sen_pos_a = 0; // SAFE readable versions of above motor postional counts.
int mtr_sen_pos_b = 0; // SAFE readable versions of above motor postional counts.

double mtr_sen_rpm_a = 0.00; // Holds Rough RPM calc per motor
double mtr_sen_rpm_b = 0.00; // Holds Rough RPM calc per motor

double mtr_sen_speed_a =  0.00; // Estimated Ideal Speed. Wheel Circumference * RPM expressed in Meters per minute MPM
double mtr_sen_speed_b =  0.00; // Estimated Ideal Speed. Wheel Circumference * RPM expressed in Meters per minute MPM

int mtr_sen_clicks_per_a = 0, mtr_sen_clicks_per_b =0;

int mtr_sen_stat_a = 0; // Default Stopped = 0 rotation  (0 = stopped, 1 = fwd, -1 = rev )
int mtr_sen_stat_b = 0; // Default Stopped = 0 rotation  (0 = stopped, 1 = fwd, -1 = rev )


// Motor Command PID Control
bool bot_ctl_Motor_PID_Enable = 0;

double bot_ctl_velocity, bot_ctl_rotation;  // PID motor control vars
double mtr_cmd_a_Input, mtr_cmd_a_Output, mtr_cmd_a_Setpoint;
double mtr_cmd_b_Input, mtr_cmd_b_Output, mtr_cmd_b_Setpoint;

double aKp=0.35, aKi=10, aKd= 0.1;
double bKp=0.35, bKi=10, bKd= 0.1;

 PID mtr_cmd_a_PID(&mtr_cmd_a_Input, &mtr_cmd_a_Output, &mtr_cmd_a_Setpoint,aKp,aKi,aKd,P_ON_M, DIRECT); // Motor A PID Object
    // mtr_cmd_a_PID.SetSampleTime(100); // match sample time for RPM
 PID mtr_cmd_b_PID(&mtr_cmd_b_Input, &mtr_cmd_b_Output, &mtr_cmd_b_Setpoint,bKp,bKi,bKd,P_ON_M, DIRECT); // Motor B PID Object
    //mtr_cmd_b_PID.SetSampleTime(100); // 10Hz sample time to match RPM calc.



/********** SENSOR GLOBALS **********/
int bot_sen_sonar_fwd_ping = 1; // Distance reported from fwd Ultra-Sonic Sensor rounded up to whole CM. Starts at 1 because <2 returns 0 as an out of bounds error cond. 1 is an Unread cond.
int bot_sen_sonar_rear_ping = 1; // Distance reported from rear Ultra-Sonic Sensor rounded up CM. Starts at 1 because <2 returns 0 as an out of bounds error cond. 1 is an Unread cond.
int bot_sen_sonar_ping_cnt = 5; // how many pings to sample for avg 


int bot_sen_ir_right_ping = 0 ; // Distance reported from Right Analog IR 
int bot_sen_ir_left_ping = 0 ; // Distance reported from Left Analog IR 
bool bot_sen_ir_fwd_ping = 0 ; // Fwd Digital IR 
bool bot_sen_ir_rear_ping = 0 ; // Rear Digital IR

SharpIR IRsensorRight( SharpIR::GP2Y0A41SK0F,IRPin_1);  // RIGHT IR sensor object. 
SharpIR IRsensorLeft( SharpIR::GP2Y0A41SK0F,IRPin_2);  // LEFT IR sensor object. 

// set some reaction distances in CM
//int bot_ctl_ir_1_ping_lowval = 15; // halt at this distance
int bot_ctl_sonar_ping_minval = 15; // halt at this distance
int bot_ctl_sonar_ping_lowval = 45; // turn until at least this much fwd space observed.

// BEHAVIOR CONTROL FLAGS


/* ----------- Functions --------------------------- */
bool randomBool() {
   return rand() > (RAND_MAX / 2);
}
// BlueTooth Terminal Output
void bot_sys_bt_conlog(String inString = ""){
  Dabble.processInput(); //Refresh data obtained from BT Mod. Calling this function is mandatory in order to get data properly from the mobile.
  if(inString.length() > 0 ){  
    Terminal.println(inString);
    }
}



/* ------------ <dpa> -------------------------- */
/* Count idle cycles per second dpa original function */ 
unsigned long idle_cnt;

void cpu_idle(ASIZE ignored)
{
    unsigned long t;
    unsigned long cnt;
    
    t = sysclock + 1000;
    while (1) {
        idle_cnt = proc_counter;
        proc_counter = 0;
        // WAIT(1000);
        PERIOD(&t,1000);
    }
}

/* -------------------------------------- */
void console_log(ASIZE delay)
{
  unsigned long cnt;
  String debug_msg;
  
  while (1) {

    if(DEBUG){
    debug_msg = "";
    debug_msg += ("======================== STATUS: ONLINE  ========================\n"); // TBD ADD SOME MEANINGFULL OUTPUT <=== HERE
    debug_msg +=  ("FWD SONAR :\t") + String (bot_sen_sonar_fwd_ping) + "\t";    
    debug_msg +=  ("REAR SONAR:\t") + String (bot_sen_sonar_rear_ping) +"\n" ; 
    debug_msg +=  ("RIGHT IR:\t") + String (bot_sen_ir_right_ping) +"\t";
    debug_msg +=  ("LEFT IR:\t") + String (bot_sen_ir_left_ping) +"\t\n";  
    debug_msg += ("\n==== MOTORS ====\n#MOTOR\t\tCPS\t\tROT\t\tCNT\n");
    debug_msg +=  "Motor A\t\t" + String( mtr_sen_clicks_per_a * 4) + "\t\t" + String( mtr_sen_stat_a) +"\t\t" + String(mtr_sen_pos_a)+"\n"; 
    debug_msg +=  "Motor B\t\t" + String( mtr_sen_clicks_per_b * 4 )  + "\t\t" + String( mtr_sen_stat_b) +"\t\t" + String(mtr_sen_pos_b)+"\n"; 
    debug_msg +=  "CTRL X:"+ String(botmsg.x) + " Z: " + String(botmsg.z) + " DeBug: " + String(botmsg.debug) + "\n"; 
    
    PRINTF(debug_msg);
    //bot_sys_bt_conlog("Motor A\nRPM:" + String(mtr_sen_rpm_a) + "\nSPD: " + String (mtr_sen_speed_a) + "\nROT: " + String( mtr_sen_stat_a) +"\nPOS: " + String(mtr_sen_pos_a)+"\n");
    //bot_sys_bt_conlog("Motor B\nRPM:" + String(mtr_sen_rpm_b) + "\nSPD: " + String (mtr_sen_speed_b) + "\nROT: " + String( mtr_sen_stat_b) +"\nPOS: " + String(mtr_sen_pos_b)+"\n");
 
    } 
    
    WAIT(delay);   
  }  
}

/* -------------------------------------- */
/* Heartbeat onbard LED. flash on and hang on a semaphore */
int flash_sem;

void led(ASIZE delay)
{
    int i;   
    while (1) {
        semaphore_obtain(&flash_sem);
        for (i = 0; i < 2; i++) {
          LED1_ON;
          WAIT(delay);
          LED1_OFF;
          WAIT(delay);
        }
    }
}

/* Flash led periodically */

void flash(ASIZE delay)
{
    TSIZE t;
    t = sysclock + delay;

    while (1) {
      // WAIT(delay);
      PERIOD(&t,delay);
      semaphore_release(&flash_sem);
    }
}
/* ----------------------------------------- */

void stats_task(ASIZE delay) /*  <dpa>  */
{
    TSIZE t;
    t = sysclock + delay;

    while (1) {
      // WAIT(delay);
      PERIOD(&t,delay);
      PRINTF("");
      SPRINTF(sbuf,"# Sysclock\t%ld\tSampleclock\t%ld\tIdleHz\t%ld",
              sysclock,sampleclock,idle_cnt);
      PRINTF(sbuf);
      PRINTF("");
      print_llist(1);
    }
}

float clip(float value, float min, float max ){ // dpa inspired cliping function
  if (value > max ) return max;
  if (value < min ) return min;
  return value;
}

/* 
-----------------------------------------  INIT Section ------------------------------------------
 TBD maybe? - Emulate a sysV type init function system. 
// init functions are named init_runlevel_class_name
// runlevels 
// 0 - Reserved        - unused currently
// 1 - Primary Systems - init_1_led_heartbeat() , 
// 2 - Sensors Systems -  
// 3 - Motor Controls  - init_3_motors_setup() , 
--------------------------------------------------------------------------------------------------
*/

/* LVL 1 INIT Functions */

// led heartbeat init 
void init_1_led_heartbeat(void){
     pinMode(LED_HEARTBEAT_PIN, OUTPUT); 
}

void I2C_callback()
{
  myTransfer.rxObj(botmsg);  // what to do when we get an I2C msg
  // Serial.println("I2C Callback");
}
const functionPtr callbackArr[] = { I2C_callback }; // Call back function pointer array. Orig lib Demo code says "persistent allocation required"

/* LVL 2 INIT Functions */ 

void init_2_sonar_setup(){
    pinMode(trigPinFwd, OUTPUT);
    pinMode(echoPinFwd, INPUT);

    pinMode(trigPinRear, OUTPUT);
    pinMode(echoPinRear, INPUT);
}
/* LVL 3 INIT Functions */

void doEncoderA(){  // ************** Encoder MTR A *********************

  // look for a low-to-high on channel A
  if (digitalRead( MTRA_ENCA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead( MTRA_ENCB) == LOW) {  
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos - 1;         // CCW
    } 
    else {
      
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos + 1;         // CW     
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MTRA_ENCB) == HIGH) {   
      
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos - 1;          // CCW
    } 
    else {
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos + 1;          // CW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(MTRA_ENCB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(MTRA_ENCA) == HIGH) {  
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos - 1;         // CCW
    } 
    else {
      
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos + 1;         // CW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MTRA_ENCA) == LOW) {   
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos - 1;          // CCW
    } 
    else {
      mtr_cal_a_encoder0Pos = mtr_cal_a_encoder0Pos + 1;          // CW
      
    }
  }
  
}

void doEncoderC(){  // ************** Encoder MTR B *********************

  // look for a low-to-high on channel A
  if (digitalRead(MTRB_ENCA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(MTRB_ENCB) == LOW) {  
       mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos - 1;         // CW
    } 
    else {
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MTRB_ENCB) == HIGH) {   
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos - 1;          // CW
    } 
    else {
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos + 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(MTRB_ENCB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(MTRB_ENCA) == HIGH) {  
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos - 1;         // CW
    } 
    else {
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(MTRB_ENCA) == LOW) {   
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos - 1;          // CW
    } 
    else {
      mtr_cal_b_encoder1Pos = mtr_cal_b_encoder1Pos + 1;          // CCW
    }
  }
  

}

void init_3_motors_setup() {
  // H-Bridge
  pinMode(MOTOR_STBY, OUTPUT); 
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT); 
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  
  // Encoders must be Interrupt capable pins. ex. Mega 2560 2,3, 18,19
  pinMode(MTRA_ENCA, INPUT); 
  pinMode(MTRA_ENCB, INPUT); 
  pinMode(MTRB_ENCA, INPUT);  
  pinMode(MTRB_ENCB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(MTRA_ENCA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MTRA_ENCB ), doEncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(MTRB_ENCA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MTRB_ENCB), doEncoderD, CHANGE); 

  // Enable Motor CMD PID
  mtr_cmd_b_PID.SetMode(AUTOMATIC);
  mtr_cmd_a_PID.SetMode(AUTOMATIC);

  if (DEBUG){PRINTF("<INIT>\tinit_3_motors_setup"); } 
  
}

int sonar_ping(int num = 0 ){  // take a sample number of pings and return rounded avg in CM. Sum adds 1cm to reserve 0 as an error situation. since this is only used as bumper and not measuerment we dont care much atm.
    float duration = 0, distance; 
    int sum = 1, cnt = 0; 

  while (cnt < bot_sen_sonar_ping_cnt)
  { 
    if (num == 0){ //default ping fwd sonar
   // Write 10 MicroSec pulse to trigger pin.
        digitalWrite(trigPinFwd,LOW);// ensure set LOW to start. 
        delayMicroseconds(2); 
        digitalWrite(trigPinFwd,HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPinFwd,LOW);  
       // measure response    
        duration = pulseIn(echoPinFwd,HIGH);

    } else if (num == 1) // ping rear sonar
    {
         // Write 10 MicroSec pulse to trigger pin.
        digitalWrite(trigPinRear,LOW);// ensure set LOW to start. 
        delayMicroseconds(2); 
        digitalWrite(trigPinRear,HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPinRear,LOW);  
       // measure response
    duration = pulseIn(echoPinRear,HIGH);
    }
    

    // calculate distance in CM
    distance = (duration / 2) * 0.0343; // speed of sound at sea level 20C 343 m/s  adjust for cond?

   if (distance >= 400 || distance <= 2 ){
      // Serial.println("Out Of Range");
      distance = 0;
   } else {
      sum += round(distance); // add measurment
      cnt++; // increment ping counter that is tested against global config var int bot_sen_ping_cnt
   }
    WAIT(1);   
  }
  int avg =  sum / cnt;
    //PRINTF(avg);
    return avg;

}

void ir_ping(){
  bot_sen_ir_right_ping = IRsensorRight.getDistance();
  bot_sen_ir_left_ping = IRsensorLeft.getDistance();
  WAIT(1);
}
/* Level 5 Control Functions */

void mtr_ctl_a(bool rev = false, int speed = 0 ) {
  MOTOR_WAKE;
  if (rev )
  {
    // reverse
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      analogWrite(MOTOR_PWMA, speed);
      mtr_sen_stat_a = -1;
  } else{ // FWD mtr A
      digitalWrite(MOTOR_AIN1, HIGH);
      digitalWrite(MOTOR_AIN2, LOW);
      analogWrite(MOTOR_PWMA, speed);
      mtr_sen_stat_a = 1;
  }
}

void mtr_ctl_b(bool rev = false, int speed = 0 ) {
  MOTOR_WAKE;
  if (rev)
  {
    // reverse
      digitalWrite(MOTOR_BIN1, LOW);
      digitalWrite(MOTOR_BIN2, HIGH);
      analogWrite(MOTOR_PWMB, speed);
      mtr_sen_stat_b = -1;
  } else{ // FWD mtr B
      digitalWrite(MOTOR_BIN1, HIGH);
      digitalWrite(MOTOR_BIN2, LOW);
      analogWrite(MOTOR_PWMB, speed);
      mtr_sen_stat_b = 1;
  }
}

void mtr_cmd_a(int speed){
  MOTOR_WAKE;
  if (speed == 0){
    analogWrite(MOTOR_PWMA, speed);
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, LOW);
    mtr_sen_stat_a = 0;
    return;
  }

  if (speed < 0 )
  {
    // reverse
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      analogWrite(MOTOR_PWMA, speed);
      mtr_sen_stat_a = -1;
  } else 
  { // FWD mtr A
      digitalWrite(MOTOR_AIN1, HIGH);
      digitalWrite(MOTOR_AIN2, LOW);
      analogWrite(MOTOR_PWMA, speed);
      mtr_sen_stat_a = 1;
  }

}

void mtr_cmd_b(int speed){
  MOTOR_WAKE;
    if (speed == 0){
    analogWrite(MOTOR_PWMB, speed);
    digitalWrite(MOTOR_BIN1, LOW);
    digitalWrite(MOTOR_BIN2, LOW);
    mtr_sen_stat_b = 0;
    return;
  }

  if (speed < 0)
  {
    // reverse
      digitalWrite(MOTOR_BIN1, LOW);
      digitalWrite(MOTOR_BIN2, HIGH);
      analogWrite(MOTOR_PWMB, speed);
      mtr_sen_stat_b = -1;
  } else{ // FWD mtr B
      digitalWrite(MOTOR_BIN1, HIGH);
      digitalWrite(MOTOR_BIN2, LOW);
      analogWrite(MOTOR_PWMB, speed);
      mtr_sen_stat_b = 1;
  }

}

void bot_motor_command(){
  // !!!THIS IS BROEKN!!!  :(  Scrap this maybe.
    return;

    // Calc right and left motor inputs based on global  Velocity and rotation factors
    int left_mtr = bot_ctl_velocity + bot_ctl_rotation; 
    int right_mtr = bot_ctl_velocity - bot_ctl_rotation;
    
    mtr_cmd_a_Input = map((mtr_sen_clicks_per_a * mtr_sen_stat_a) ,1,90,1,100); // map clicks per 250msec as percentage 330 cps = MAX  = 100  
    mtr_cmd_b_Input = map((mtr_sen_clicks_per_b * mtr_sen_stat_b ),1,90,1,100); // map clicks per 250msec as percentage 330 cps = MAX  = 100  

    mtr_cmd_b_Setpoint = clip(left_mtr,-100,100);
    mtr_cmd_a_Setpoint = clip(right_mtr,-100,100);
    
    // compute each motor PID 
    mtr_cmd_a_PID.Compute();
    mtr_cmd_b_PID.Compute();
      
    mtr_cmd_a( (right_mtr/abs(right_mtr) * mtr_cmd_a_Output)); // send +/-  mtr_cmd_a_Output for rotation
//    PRINTF((right_mtr/abs(right_mtr) * mtr_cmd_a_Output));
    mtr_cmd_b( (left_mtr/abs(left_mtr) * mtr_cmd_b_Output));  
//    PRINTF((left_mtr/abs(left_mtr) * mtr_cmd_b_Output));

}

void stop() {
  MOTOR_WAKE;
  //Aye Capt! FULL STOP!
  // Brakes //
   // Work on migrating this to a decellerate funct in future.

  // bot_ctl_velocity = 0; // Set global bot velocity factor to 0.
  // bot_ctl_rotation = 0; ;// Set global rotation factor to FALSE
  // mtr_cmd_a(0);
  // mtr_cmd_b(0);
/*
  mtr_ctl_a(0,0); // set motors to zero
  mtr_sen_stat_a = 0;

  mtr_ctl_b(0,0);
   mtr_sen_stat_b = 0;
*/
   /*
   digitalWrite(MOTOR_AIN1, LOW);
   digitalWrite(MOTOR_AIN2, LOW);
   analogWrite(MOTOR_PWMA,0);
   
   digitalWrite(MOTOR_BIN1, LOW);
   digitalWrite(MOTOR_BIN2, LOW);
   analogWrite(MOTOR_PWMB,0);  
*/
  mtr_ctl_a(0,0); // set motors to zero
  mtr_sen_stat_a = 0;

  mtr_ctl_b(0,0);
  mtr_sen_stat_b = 0;

  WAIT(10);
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA,0);
   
  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, LOW);
  analogWrite(MOTOR_PWMB,0); 

   //WAIT(1);
}

void bot_ctl_forward(int speed){
  MOTOR_WAKE;
  mtr_ctl_speed_a = mtr_ctl_speed ;
  mtr_ctl_a(false, mtr_ctl_speed_a);

  mtr_ctl_speed_b = mtr_ctl_speed ;
  mtr_ctl_b(false, mtr_ctl_speed_b);


}

void bot_ctl_backward(int speed){
  MOTOR_WAKE;
  mtr_ctl_speed_a = mtr_ctl_speed ;
  mtr_ctl_a(true, mtr_ctl_speed_a);
  
  mtr_ctl_speed_b = mtr_ctl_speed ;
  mtr_ctl_b(true, mtr_ctl_speed_b);
}

void bot_ctl_backoffturn_left(int speed){
    MOTOR_WAKE;
      mtr_ctl_b(true, speed);// back motor b up by global val
      mtr_ctl_a(0, 0);// put motor A to idle
}

void bot_ctl_backoffturn_right(int speed){
    MOTOR_WAKE;
      // back motor A up by global val
      mtr_ctl_a(true, speed);
      // put motor B to idle
      mtr_ctl_b(0,0);
}

void bot_ctl_pivot( bool rotation){ // 0 = neg rotation (ie. CCW)  1 = pos rotation (ie. CW)
    MOTOR_WAKE;
    if(DEBUG){
     
    }
    if (rotation) { // Positive rotation. Clock wise pivot to the right.
      // back motor A up by global val
      mtr_ctl_a(1, mtr_ctl_pivot_speed);
      // fwd B
      mtr_ctl_b(0, mtr_ctl_pivot_speed);          
    } else {
       // FWD motor A up by global val
      mtr_ctl_a(0, mtr_ctl_pivot_speed);
      // BACK  MTR B
      mtr_ctl_b(1, mtr_ctl_pivot_speed);
    }
}

void svc_I2C_conn (ASIZE delay){ // Internal I2C Data Exchange Service
  while(1){
    statmsg.x               = botmsg.x; // Float. Currently demanded X. Feedback for ctrlmsg
    statmsg.z               = botmsg.z; // Float. Currently demanded Z. Feedback for ctrlmsg
    statmsg.mtr_pos_right   = mtr_sen_pos_a; // right motor encoder position
    statmsg.mtr_pos_left    = mtr_sen_pos_b;  // left motor encoder position
    statmsg.mtr_speed_right = mtr_ctl_speed_a;  // motor a speed
    statmsg.mtr_speed_left  = mtr_ctl_speed_b;  // motor b speed
    statmsg.sen_sonar_fwd   = bot_sen_sonar_fwd_ping; // forward sonar value
    statmsg.sen_sonar_rear  = bot_sen_sonar_rear_ping; // rear sonar value
    statmsg.sen_ir_right    = bot_sen_ir_right_ping; // right IR value
    statmsg.sen_ir_left     = bot_sen_ir_left_ping; // left IR value 
    strcpy (statmsg.debug, "LOG"); // short logging message
    // rxSerialTransfer.sendDatum(statmsg);
  WAIT(delay);
  }
}

void svc_ping( ASIZE delay){ // PING service function.
    
  while (1)
  {
    ir_ping(); // Analog IR side sensors update global vars
    bot_sen_sonar_fwd_ping = sonar_ping(0); // 0 = FWD 1 = Rear
    WAIT(1); // stagger sonar pings by 1 ms
    bot_sen_sonar_rear_ping = sonar_ping(1);
    
    //PRINTF("DEBUG!");
    WAIT(delay);
  }  
}

void motor_test (ASIZE delay)

{
  while (1)
  {
    
   MOTOR_WAKE;
   int speed = 150;
   digitalWrite(MOTOR_AIN1, HIGH);
   digitalWrite(MOTOR_AIN2, LOW);
   analogWrite(MOTOR_PWMA, speed);
   PRINTF("FWD A ");
   WAIT(delay);
   
   stop();
   WAIT(1000);
   
   digitalWrite(MOTOR_AIN1, LOW);
   digitalWrite(MOTOR_AIN2, HIGH);
   analogWrite(MOTOR_PWMA, speed);
   PRINTF("REV A ");
   WAIT(delay);


   stop();
  WAIT(1000);

   digitalWrite(MOTOR_BIN1, HIGH);
   digitalWrite(MOTOR_BIN2, LOW);
   analogWrite(MOTOR_PWMB, speed);
   PRINTF("FWD B ");
  
   WAIT(delay);
   stop();
  WAIT(1000);
   digitalWrite(MOTOR_BIN1, LOW);
   digitalWrite(MOTOR_BIN2, HIGH);
   analogWrite(MOTOR_PWMB, speed);
   PRINTF("REV B ");
   WAIT(delay);
   stop();
   WAIT(5000);
   
   }

}

void svc_encoders(ASIZE ignored){ // 10Hz Wheel encoder update function
     unsigned long x = sysclock;
     int interval = 250;
     int posPrev_a, posPrev_b, deltaA, deltaB;
     posPrev_a = mtr_sen_pos_a;
     posPrev_b = mtr_sen_pos_b;

    while(1){
     // This ATOMIC_BLOCK reads the encoders moves the data to globals that we can use. "Safely" 
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
       mtr_sen_pos_a = mtr_cal_a_encoder0Pos;    // MTR A encoder 
       mtr_sen_pos_b = mtr_cal_b_encoder1Pos ; // MTRB encoder
        
      }
      if (sysclock >= (x + interval) ){ // if we have passed interval time           
        // calc delta pos
            deltaA = abs(abs(mtr_sen_pos_a) - abs(posPrev_a));
            if(deltaA){
              mtr_sen_clicks_per_a = deltaA;
              } else{
                mtr_sen_clicks_per_a = 0;
              }
            deltaB = abs(abs(mtr_sen_pos_b) - abs(posPrev_b));
            if (deltaB)  {  
              mtr_sen_clicks_per_b = deltaB;    
              } else{
                mtr_sen_clicks_per_b = 0;
              }
          // reset for next run
        
        posPrev_a = mtr_sen_pos_a;
        posPrev_b = mtr_sen_pos_b;  
        x = sysclock;
        }

      WAIT(1);
      }
  
}

void bot_svc_bt_pin_monitor(ASIZE delay){
  while (1)
  {
    Dabble.processInput(); //Refresh data obtained from BT Mod. Calling this function is mandatory in order to get data properly from the mobile.
    PinMonitor.sendDigitalData(); // DABBLE : This function sends all the digital pins state to the app
    PinMonitor.sendAnalogData() ; // DABBLE : This function sends all the analog  pins state to the app  /* code */
    WAIT(delay);
  }
  
  
}

void bot_motor_cmd_svc ( ASIZE delay){ // Motor Command Service Function
  while (1)
  {
    // DISABLING THIS PID FOR NOW DUE TO "Hardware Failure. Mar 07"
    bot_ctl_Motor_PID_Enable = false;

    if (bot_ctl_Motor_PID_Enable)
    {
    bot_motor_command();  
    }  

   WAIT(1);
 
  }
         
}

void bot_bt_input(ASIZE delay){ // user input motion control from BT app
  while (1)
  {
    Dabble.processInput(); //Refresh data obtained from BT Mod. Calling this function is mandatory in order to get data properly from the mobile.
    if (DEBUG){
      //PinMonitor.sendDigitalData(); // DABBLE : This function sends all the digital pins state to the app. Currently causes hang. BUG TBD.
      //PinMonitor.sendAnalogData() ; // DABBLE : This function sends all the analog  pins state to the app
    }

    /* BEGIN Game Pad Code. *** TBD ** Move to seperate function */
    // cmd aliases for clarity
    bool up = GamePad.isUpPressed();
    bool down = GamePad.isDownPressed();
    bool left = GamePad.isLeftPressed();
    bool right = GamePad.isRightPressed();
    bool square = GamePad.isSquarePressed();
    bool circle = GamePad.isCirclePressed();
    bool triangle = GamePad.isTrianglePressed();
    bool cross = GamePad.isCrossPressed();

  if (GamePad.isStartPressed()){
    //bot_ctl_Motor_PID_Enable = true;
    WAIT(20);
    continue;
    
  }
  if (GamePad.isSelectPressed()){
    // make a selction.
    //bot_ctl_Motor_PID_Enable = false;
    WAIT(20);
    continue;
  }

  if ( up or down or left or right or circle or square or triangle or cross){ // a movement key was pressed 

    if( up)  {
      bot_ctl_forward(mtr_ctl_speed);     
      WAIT(delay);
    } 

    if (down)   {
      bot_ctl_backward (mtr_ctl_speed);
      WAIT(delay);
    } 

    if (left)
    {
      mtr_ctl_a(0, int (mtr_ctl_speed * 1.25) );
      mtr_ctl_b(0, int (mtr_ctl_speed /1.4));
      WAIT(delay);
    } 

    if (right)
    {
      mtr_ctl_b(0, int (mtr_ctl_speed * 1.25));
      mtr_ctl_a(0, int (mtr_ctl_speed /1.25));
      WAIT(delay);
    } 
    if (GamePad.isSquarePressed())
    {
      bot_ctl_backoffturn_left(mtr_ctl_pivot_speed);
      WAIT(delay);
    }

    if (circle)
      {
      bot_ctl_backoffturn_right(mtr_ctl_pivot_speed ) ;
      WAIT(delay);
      }

    if (triangle)
      {
      bot_ctl_pivot(1);
      WAIT(delay);
      }

    if (cross)
      {
      bot_ctl_pivot(0);
      WAIT(delay);
      }

  } else{
    stop(); // otherwise brake motors
    WAIT(delay);
  }



/*



  if (GamePad.isStartPressed())
  {
    
  }

  if (GamePad.isSelectPressed())
  {
    
  }
  */    

    WAIT(delay);
  }
  
  
}

/* --------------------------------------------------------------------------------------------------------------------------- 
 system_init  - Add all other runlevel init functions  
 --------------------------------------------------------------------------------------------------------------------------- */
void system_init(void)
{
    /* AVR & ARM Teesy3.1  */
    sysclock_init(); //  <dpa>. clock.
    Serial.begin(BAUDRATE); 
    Dabble.begin(9600);  //Enter baudrate of your bluetooth module 
    init_1_led_heartbeat(); // LED heartbeat
    init_2_sonar_setup(); // Initalize sonar
    init_3_motors_setup(); // Motor Init    

}

/* ------------ <dpa> -------------------------- */
/* Create signon and terminate task */
void signon(ASIZE version)
{
  PRINTF(VERSION);
  wake_after(2000);
  PRINTF("# SIGNON Messages signing off!\n");
  DELAY(1000);
  terminate();
}

/*-----------------------------------------  MAIN ------------------------------------------

------------------------------------------------------------------------------------------*/

void setup()

{
    system_init();

    Wire.begin(I2C_ADDR);// 
    configST I2C_myConfig; 
    I2C_myConfig.debug        = true;
    I2C_myConfig.callbacks    = callbackArr;
    I2C_myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
    myTransfer.begin(Wire, I2C_myConfig);


    printv = printkbuf;

    PRINTF("Howdy Console!\n");
        
    #if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */ // <dpa> libtask set in task.h
    delay(1500);   /* hardware delay, sysclock not running yet */
    #endif

    pid_count = 0; current = 0;


   // create_task((char *)"MTRTST",motor_test,5000, MINSTACK );   
   
    /************** TASKS *******************/
    // EX 2000ms:  create_task((char *)"TSKNAM",function_name,2000, MINSTACK );
    
    // LMX Tasks
    create_task((char *)"IDLE",cpu_idle,0,MINSTACK);
    create_task((char *)"STATS",stats_task,15000,MINSTACK*4);
    create_task((char *)"SIGNON",signon,1,MINSTACK*4);
    
    // Level 1 System Tasks
    
    create_task((char *)"I2C",svc_I2C_conn,1, MINSTACK*4); // update UART connection with ROS2 Controler.
    create_task((char *)"LED",led,200, MINSTACK); // heatbeat. kept as example of how to use semaphore setting and fetching with LMX
    create_task((char *)"FLASH",flash,800,MINSTACK); // Timing part of heatbeat. kept as example of how to use semaphore setting 
    create_task((char *)"CONLOG",console_log,5000,MINSTACK*2); // Dev logging to USB Serial output

    // Level 2 Services     
    create_task((char *)"ENCDR",svc_encoders,1, MINSTACK ); // Motor Encoder Reading Service. Delay is ignored.
    create_task((char *)"PING",svc_ping,10, MINSTACK*2); // IR and Sonar Ping service
    // create_task((char *)"MENTOR",bot_svc_bt_pin_monitor,10, MINSTACK); // Pin Monitor. ???not wrking 8()
    

    // Level 3 Controls
    create_task((char *)"BTCTL",bot_bt_input,5, MINSTACK); // BlueTooth User Input Controler

    scheduler(); // Main LMX task scheduler
    PRINTF("Should never get here."); // Leave this alone.

    while (1);
    #if ((MACHINE != MACH_AVR) && (MACHINE != MACH_ARM))
    return 0;
    #endif
}

void loop() 
{
  /* Noting to see here. Move along. Not the droid you're looking for. */
  asm("nop");
}

/* EOF */