/* -------------------------------------- */
/* 
Bum Biter Bot MK 2.0
 *  03 Mar 2022 RM - Moved wheel encoder reading to a service used an atomic_block for less erratic results from reading the interrupt diriven variables.
                      - Working on PID control.
                        - Got RPM conunt working. It could probably be better.
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
 *            
 *  
 *     Software:
 *        
 *            - libtask LMX from D.P. Anderson  as a scheduler 
 *            - Stole this IR sensor lib https://github.com/qub1750ul/Arduino_SharpIR 
 *            - Currently Porting from Mk1. Unamed custom written TB6612fng Motor Driver functions that should probably move to a inc or lib later.
 *            - Blue Tooth interface using libraries from Dabble https://thestempedia.com/docs/dabble/game-pad-module/
 *          
 * 
 *  TO-DO List: 
 *            - Add ability to send some console DEBUG log info back over serial BT console.
 *            - Add visual feedback RGB LEDs
 *            - Add Speaker. 
 *            - Add IMU
 *            - BUG. When Arduino reboots BT connection is problematic.
 *              |-> Connect the appropriate reset ping to the BT module and have it restart when the sketch does.
 *                |-->Make sure above is run time configurable option ALSO a tirggerable event.
 * 
 *            - Too long for now.
 *         
 *               
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
#include <stdio.h> 
#include <task.h> // LMX
#include <log.h> // LMX
#include <sysclock.h> // LMX

// Sensors
#include <SharpIR.h> // IR Distance Sensors

// BlueTooth
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer. 
   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.
   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#include <Dabble.h> // Dabble Bluetooth Contoller


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
#define MTRA_ENCA 2 // Motor A encoder output A must be on interrupt pin.
#define MTRA_ENCB 3 // Motor A encoder output on B on an interrupt pin.
#define MTRA_CLICKS_PER 132 // encoder clicks per bot wheel rotation.
#define MTRA_WHEELD 60.3 // wheel D in mm


// motor B
#define MTRB_ENCA 19 // Motor B encoder output A must be on interrupt pin. // remember the motors are flipped
#define MTRB_ENCB 18 // Motor B encoder output B must be on interrupt pin.
#define MTRB_CLICKS_PER  132 // encoder clicks per bot wheel rotation.
#define MTRB_WHEELD 60.3 // wheel D in mm

// These Interface with Motor Driver. TB6612fng H-Bridge
// keep in mind the PWM defines must be on PWM pins
#define MOTOR_BIN2 34
#define MOTOR_BIN1 36
#define MOTOR_STBY 38
#define MOTOR_AIN1 40                                       
#define MOTOR_AIN2 42
#define MOTOR_PWMB 44 // PWM pin
#define MOTOR_PWMA 46 // PWM Pin 


// sound control
// #define BEEP xx // hooked to passive piezo buzzer.

// SENSONR PINS 
  //  HC-SR04 Ultra-sonic Distance Sensors
      // FWD 
#define trigPinFwd 32 // blue
#define echoPinFwd 30 // green
    // REAR
#define trigPinRear 26
#define echoPinRear 28

  // Sharp Analog IR sensor 4-30CM  gp2y0a41sk0f 
#define IRPin_1  PIN_A0
#define IRPin_2  PIN_A1
#define IR_SENSOR_1 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR
#define IR_SENSOR_2 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR
  // Sharp Digital IR distance Sensors 2YD021
#define IRPin_FWD 4 
#define IRPin_Rear 5

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
char bot_sys_debug[] = ""; // string to hold debug output.

/********** MOTOR GLOBALS **********/

int mtr_ctl_speed = 150; // CFG > default drive speed. tweak these as req either here or runtime
int mtr_ctl_pivot_speed = 150; // CFG > default pivot turn speed. 

int mtr_ctl_speed_a = 0;  // to set motor a speed
int mtr_ctl_speed_b = 0;  // used set motor b speed

volatile int mtr_cal_pos_a = 0; // motor A positional counts +/- // volatiles are read in an ATOMIC_BLOCK
volatile int mtr_cal_pos_b = 0; // motor B positional counts +/- volatiles are read in an ATOMIC_BLOCK
float mtr_sen_rpm_a = 0; // Rought RPM calc per motor
float mtr_sen_rpm_b = 0; // Rought RPM calc per motor

float mtr_sen_speed_a =  0; // cm per minute ?
float mtr_sen_speed_b =  0; // cm per minute ?

int mtr_sen_pos_a = 0; // SAFE readable versions of above motor postional counts.
int mtr_sen_pos_b = 0; // SAFE readable versions of above motor postional counts.



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
  
  while (1) {

    if(DEBUG){
    PRINTF ("STATUS: ONLINE \n======"); // TBD ADD SOME MEANINGFULL OUTPUT <=== HERE
    PRINTF("FWD PING:\t");
    PRINTF (bot_sen_sonar_fwd_ping); 
    
    PRINTF("REAR PING:\t");
    PRINTF (bot_sen_sonar_rear_ping); 
    
    PRINTF("Wheels: A  B\t");
    PRINTF (mtr_sen_pos_a);
    PRINTF (mtr_sen_pos_b);
    PRINTF("RPM Velocity A  B :\t");
    PRINTF (mtr_sen_rpm_a );
    PRINTF (mtr_sen_rpm_b);

    PRINTF("SPEED  Meters per min");
    PRINTF(mtr_sen_speed_a);
    PRINTF(mtr_sen_speed_b);

      


/*    
    PRINTF("IR Right:\t");
    PRINTF (bot_sen_ir_right_ping); 
    
    PRINTF("IR Left:\t");
    PRINTF (bot_sen_ir_left_ping); 

    PRINTF("IR FWD:\t");
    PRINTF (bot_sen_ir_fwd_ping); 

    PRINTF("IR Rear:\t");
    PRINTF (bot_sen_ir_rear_ping); 
*/
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
    unsigned long cnt;
    
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
  unsigned long cnt;
    while (1) {
      semaphore_release(&flash_sem);
      WAIT(delay);
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

/* 
-----------------------------------------  INIT Section ------------------------------------------
 TBD - Emulate a sysV type init function system. 

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
void init_1_led_heartbeat(void)
{
     pinMode(LED_HEARTBEAT_PIN, OUTPUT); 
}

/* ----------------------------------------- */

/* LVL 2 INIT Functions */

void init_2_sonar_setup(){
    pinMode(trigPinFwd, OUTPUT);
    pinMode(echoPinFwd, INPUT);

    pinMode(trigPinRear, OUTPUT);
    pinMode(echoPinRear, INPUT);
}

void init_2_IR_setup(){
  pinMode(IRPin_FWD, INPUT); //  the 2 digital IR sensors
  pinMode(IRPin_Rear, INPUT);
}

/* LVL 3 INIT Functions */

void bot_mtr_a_readEncoder(){  // run on INT and increment or decrement the wheel counter. Each wheel is as independant as possible.
  // Read encoder B when encoder A rises
  int mtra_encb = digitalRead(MTRA_ENCB);
  int increment = 0;
  if(mtra_encb>0){
    increment++;
    mtr_cal_pos_a++; // these globals are volatile. They are read in an ATOMIC_BLOCK    
  }
  else {
    increment--;
    mtr_cal_pos_a--; // these globals are volatile. They are read in an ATOMIC_BLOCK 

  }

   /*
      // Compute velocity with method 2
      long currT = micros();
      float deltaT = ((float) (currT - mtr_cal_prevT_a ))/1.0e6;
      mtr_cal_vel_a  = increment/deltaT;
      mtr_cal_prevT_a = currT;
    */

}

void bot_mtr_b_readEncoder(){
  int increment = 0;
  int mtrb_encb = digitalRead(MTRB_ENCB);
  if(mtrb_encb>0){
    increment++;
    mtr_cal_pos_b++;// these globals are volatile. They are read in an ATOMIC_BLOCK 
  }
  else {
    increment--;
    mtr_cal_pos_b--; // these globals are volatile. They are read in an ATOMIC_BLOCK 

  }
/*
    // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - mtr_cal_prevT_b ))/1.0e6;
  mtr_cal_vel_b  = increment/deltaT;
  mtr_cal_prevT_b = currT;
  */
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
  
  attachInterrupt(digitalPinToInterrupt(MTRA_ENCA), bot_mtr_a_readEncoder, RISING); // set interupts
  attachInterrupt(digitalPinToInterrupt(MTRB_ENCA), bot_mtr_b_readEncoder, RISING); // set interups

  if (DEBUG){PRINTF("<INIT>\tinit_3_motors_setup"); } 
  
}


/* Level 5 Control Functions */

void mtr_ctl_a(bool rev = false, int speed = 0 ) {
  MOTOR_WAKE;
  if (rev)
  {
    // reverse
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      analogWrite(MOTOR_PWMA, speed);
  } else{ // FWD mtr A
      digitalWrite(MOTOR_AIN1, HIGH);
      digitalWrite(MOTOR_AIN2, LOW);
      analogWrite(MOTOR_PWMA, speed);
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
  } else{ // FWD mtr B
      digitalWrite(MOTOR_BIN1, HIGH);
      digitalWrite(MOTOR_BIN2, LOW);
      analogWrite(MOTOR_PWMB, speed);
  }
}

void stop() {

  mtr_ctl_a(0,0); // set motors to zero
  mtr_ctl_b(0,0);

   // Brakes //
   // Full stop Cap't. Work on migrating this to a decellerate funct in future.
   digitalWrite(MOTOR_AIN1, HIGH);
   digitalWrite(MOTOR_AIN2, HIGH);
   analogWrite(MOTOR_PWMA,0);
   
   digitalWrite(MOTOR_BIN1, HIGH);
   digitalWrite(MOTOR_BIN2, HIGH);
   analogWrite(MOTOR_PWMB,0);  
   //WAIT(1);
}

void motor_test_a (ASIZE delay) // run motor  fwd and backward 
{
   MOTOR_WAKE;
   int speed = 150;

   while (1)
   {
   digitalWrite(MOTOR_AIN1, HIGH);
   digitalWrite(MOTOR_AIN2, LOW);
   analogWrite(MOTOR_PWMA, speed);
   WAIT(delay);
   stop();
   WAIT(delay);
   digitalWrite(MOTOR_AIN1, LOW);
   digitalWrite(MOTOR_AIN2, HIGH);
   analogWrite(MOTOR_PWMA, speed);
   WAIT(delay);
   stop();
   WAIT(5000);   
   }

}

void motor_test_b (ASIZE delay)// run motor  fwd and backward 
{
   MOTOR_WAKE;
   
   int speed = 150;

   while (1)
   {
   digitalWrite(MOTOR_BIN1, HIGH);
   digitalWrite(MOTOR_BIN2, LOW);
   analogWrite(MOTOR_PWMB, speed);
   WAIT(delay);
   stop();
   WAIT(delay);
   digitalWrite(MOTOR_BIN1, LOW);
   digitalWrite(MOTOR_BIN2, HIGH);
   analogWrite(MOTOR_PWMB, speed);
   WAIT(delay);
   stop();
   WAIT(5000);   
   }

}

void bot_ctl_forward(int speed){
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
      // back motor b up by global val
      mtr_ctl_b(true, speed);
      // put motor A to idle
      mtr_ctl_a(0, 0);
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
    //PRINTF("PIVOT TURN REQESTED ");
    //PRINTF(rotation);
    }
    //int test = rotation;
    //PRINTF(test);
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

int sonar_ping(int num = 0 ){  // take a sample number of pings and return rounded avg in CM. Sum adds 1cm to reserve 0 as an error situation. since this is only used as bumper and not measuerment we dont care much atm.
    float duration, distance; 
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
    // send results to serial montior
    //Serial.print ("Distance = ");

   if (distance >= 400 || distance <= 2 ){
      // Serial.println("Out Of Range");
      distance = 0;
      // sum += round(distance);    
      
   } else {
      // Serial.println(round(distance));
      // Serial.print(" cm\n");
      // WAIT(1000);
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

  if( digitalRead(IRPin_FWD)  == HIGH ){
    bot_sen_ir_fwd_ping = true;
  } else bot_sen_ir_fwd_ping = false;
  // Rear IR disconnected pending replacement. Mar 1 2022 probably not required in final build anyway
  if( digitalRead(IRPin_Rear)  == HIGH ){
    bot_sen_ir_rear_ping = true;
  } else bot_sen_ir_rear_ping = false;
  WAIT(1);
}

void svc_ping( ASIZE delay){ // PING service function.
    
  while (1)
  {
    bot_sen_sonar_fwd_ping = sonar_ping(0);
    WAIT(1); // stager sonar pings by 1 ms
    bot_sen_sonar_rear_ping = sonar_ping(1);
    ir_ping(); // Analog IR side sensors update global vars

    WAIT(delay);
  }  
}

void svc_encoders(ASIZE ignored){ // Wheel encoder update function
    while(1){
     int posPrev_a = mtr_sen_pos_a;
     int posPrev_b = mtr_sen_pos_b;
     float mtr_a_roations = 0;
     float mtr_b_roations = 0;

      // This ATOMIC_BLOCK reads the encoders and calculates the RPM for each motor during the interrupts then moves the data to globals that we can use. 
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        mtr_sen_pos_a = mtr_cal_pos_a; 
        mtr_sen_pos_b = mtr_cal_pos_b; 
      
        // mtr_sen_vel_a = mtr_cal_vel_a;
        // mtr_sen_vel_b = mtr_cal_vel_b;
      mtr_a_roations =  (abs( mtr_sen_pos_a - posPrev_a   )  * 600) / MTRA_CLICKS_PER ; // 600 (60sec * 10 times per sec for RPM) | encoder clicks per wheel rotation
      mtr_b_roations =  (abs( mtr_sen_pos_b - posPrev_b )  * 600) / MTRB_CLICKS_PER ; // 240 (60sec * 10 times per sec for RPM) | encoder clicks per wheel rotation
      mtr_sen_rpm_a = mtr_a_roations;
      mtr_sen_rpm_b = mtr_b_roations;

      mtr_sen_speed_a = mtr_a_roations * ( (MTRA_WHEELD * PI) / 1000 ); 
      mtr_sen_speed_b = mtr_b_roations * ( (MTRB_WHEELD * PI) / 1000 );
    
      }

      WAIT(100); // Samples encoder counts 10 times per sec. much faster than 100ms loop seems to give erratic results. Not enough clicks for good math.
      

    

  }
 
}

void bot_cruise ( ASIZE delay){ // Cruise Behavior
  while (1)
  {
    /*
    <<< Wake >>>
    If(Need){
      Seek
      Do-Until

    <<< THE END >>>
    */

   WAIT(1);
 
  }
         
}

void bot_bt_input(ASIZE delay){ // user input motion control from BT app
  while (1)
  {
    Dabble.processInput(); //Refresh data obtained from BT Mod. Calling this function is mandatory in order to get data properly from the mobile.
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
    // make a selction.
    WAIT(20);
    continue; 
  }
  if (GamePad.isStartPressed()){
    // make a selction.
    WAIT(20);
    continue;
  }

  if ( up or down or left or right or circle or square or triangle or cross){ // a movement key was pressed 

    if( up)  {
      bot_ctl_forward(mtr_ctl_speed);
      WAIT(delay);
    } 

    if (down)   {
      bot_ctl_backward(mtr_ctl_speed);
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


/* --------------------------------------------------------------------------------------------------------------------------- */

/* system_init  - Add all other runlevel init functions  */
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
/* ----------------------------------------- */

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
/* ----------------------------------------- */

/* 
-----------------------------------------  MAIN ------------------------------------------

 this is for the Arduino IDE "sketch" set up 
------------------------------------------------------------------------------------------
*/

void setup()

{
 
    system_init();
    printv = printkbuf;


    PRINTF("Howdy Console!\n");

    #if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */ // <dpa> libtask set in task.h
    delay(1500);   /* hardware delay, sysclock not running yet */
    #endif

    pid_count = 0; current = 0;
   // create_task((char *)"MOTOR_TEST_A",motor_test_a,2000, MINSTACK );
   //  create_task((char *)"MOTOR_TEST_B",motor_test_b,2000, MINSTACK );
   // create_task((char *)"CRUISE",bot_cruise,300, MINSTACK);
    //create_task((char *)"ENCDR",svc_read_encoders,1, MINSTACK ); // Motor Encoder Reading Service. Delay is ignored.

    /**************TASKS *******************/
    
    // LMX Tasks
    create_task((char *)"IDLE",cpu_idle,0,MINSTACK);
    create_task((char *)"STATS",stats_task,10000,MINSTACK*4);
    create_task((char *)"SIGNON",signon,1,MINSTACK*4);
    
    // Level 1 System Tasks
    create_task((char *)"LED",led,200, MINSTACK); // heatbeat. kept as example of how to use semaphore setting and getting with LMX
    create_task((char *)"FLASH",flash,800,MINSTACK); // heatbeat. kept as example of how to use semaphore setting
    create_task((char *)"CONLOG",console_log,2000,MINSTACK*2); // logging to serial output

    // Level 2 Services     
    create_task((char *)"ENCDR",svc_encoders,1, MINSTACK ); // Motor Encoder Reading Service.  250ms. Delay is ignored.
    create_task((char *)"PING",svc_ping,10, MINSTACK); // IR and Sonar Ping service

    // Level 3 Controls
    create_task((char *)"BTCTL",bot_bt_input,5, MINSTACK); // BlueTooth User Input Contorler

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