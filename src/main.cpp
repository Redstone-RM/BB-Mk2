/* -------------------------------------- */
/* 
Bum Biter Bot MK 2.0
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
 *            - Mega2560. 
 *            - TB6612fng Motor Driver with 2x TT motors (wired in a fliped/mirrored configuration because why suffer?.)
 *            - HC-SRO4 Ultrasonic distance Sensor
 *            - Sharp GP2Y0A41SK0F IR distance sensor 4 - 30CM 
 *            - Not yet moved from mk1 is the Passive Piezo speaker for 8bit jams
 *            - HM-10 V3 BT 4.0 on Serial3   
 *            
 *  
 *     Software:
 *        
 *            - libtask LMX from D.P. Anderson  as a scheduler 
 *            - Stole this IR sensor lib https://github.com/qub1750ul/Arduino_SharpIR 
 *            - Currently Porting from Mk1. Unamed custom written TB6612fng Motor Driver functions that should probably move to a lib later.
 *            - Blue Tooth interface using libraries from Dabble https://thestempedia.com/docs/dabble/game-pad-module/
 *          
 * 
 *  TO-DO List - Too long for now.
 *            
 *               
 *  
 */
/* -------------------------------------- */

/* -------------------------------------- */
#define VERSION "Bum Biter MK-2.0.1"
#define DEBUG true

/* -------------------------------------- */
#include <Arduino.h> // Only Needed for PlatformIO.


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
/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer. 
   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.
   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h> // Bluetooth


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

// motor control pins. 
// Motor setup notes
// MOTOR A = RIGHT
// MOTOR B = LEFT
// FWD = HIGH/LOW
// REV = LOW/HIGH
// BRAKE = HIGH/HIGH

// motor A
#define MTRA_ENCA 2 // Motor A encoder output A must be on interrupt pin.
#define MTRA_ENCB 3 // Motor A encoder output on B on an interrupt pin.
// motor B
#define MTRB_ENCA 18 // Motor B encoder output A must be on interrupt pin.
#define MTRB_ENCB 19 // Motor B encoder output B must be on interrupt pin.

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

// Share Analog IR sensor 4-30CM  gp2y0a41sk0f 
#define IRPin_1  PIN_A0
#define IRPin_2  PIN_A1
#define IR_SENSOR_1 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR
#define IR_SENSOR_2 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR

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
                     cal   - Calculated variables
                     

*/

int bot_init_runlvl  = 5; // Current init level.
char bot_sys_debug[] = ""; // string to hold debug output.

// MOTOR GLOBALS
int mtr_ctl_speed_a = 150;  // set motor a speed
int mtr_ctl_speed_b = 150;  // set motor b speed
int mtr_cal_pos_a = 0; // motor A positional counts +/-
int mtr_cal_pos_b = 0; // motor B positional counts +/-
int mtr_ctl_speed = 150;
int mtr_ctl_pivot_speed = 150;


// SENSOR GLOBALS
int bot_sen_sonar_fwd_ping = 1; // Distance reported from fwd Ultra-Sonic Sensor rounded up to whole CM. Starts at 1 because <2 returns 0 as an out of bounds error cond. 1 is an Unread cond.
int bot_sen_sonar_rear_ping = 1; // Distance reported from rear Ultra-Sonic Sensor rounded up CM. Starts at 1 because <2 returns 0 as an out of bounds error cond. 1 is an Unread cond.
int bot_sen_sonar_ping_cnt = 5; // how many pings to sample for avg 

int bot_sen_ir_right_ping = 0 ; // Distance reported from Right Analog IR 
int bot_sen_ir_left_ping = 0 ; // Distance reported from left Analog IR 

SharpIR IRsensorFWD( SharpIR::GP2Y0A41SK0F,IRPin_1);  // FWD IR sensor object. 
SharpIR IRsensorRear( SharpIR::GP2Y0A41SK0F,IRPin_2);  // FWD IR sensor object. 

// set some reaction distances in CM
int bot_ctl_ir_1_ping_lowval = 15; // halt at this distance
int bot_ctl_sonar_ping_minval = 15; // halt at this distance
int bot_ctl_sonar_ping_lowval = 45; // turn until at least this much fwd space observed.

// BEHAVIOR CONTROL FLAGS
int bot_ctl_cruise = 1; // enable cruise mode
int bot_ctl_turn_attempts = 10 ;

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
    PRINTF ("STATUS: ONLINE \n======"); // TBD ADD SOME MEANINGFULL OUTPUT <=== HERE
    PRINTF("PING:\t");
    PRINTF (bot_sen_sonar_fwd_ping); 
    PRINTF ("\n");
    
    if(DEBUG){
      PRINTF ("");
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

/* LVL 3 INIT Functions */

void init_3_motors_setup() {
   pinMode(MOTOR_STBY, OUTPUT); 

   pinMode(MOTOR_AIN1, OUTPUT);
   pinMode(MOTOR_AIN2, OUTPUT);
   pinMode(MOTOR_PWMA, OUTPUT);
   
   pinMode(MOTOR_BIN1, OUTPUT);
   pinMode(MOTOR_BIN2, OUTPUT);
   pinMode(MOTOR_PWMB, OUTPUT);
      // Wake up motor driver
   PRINTF("<INIT>\tinit_3_motors_setup");
}

/* Level 5 Control Functions */

void stop() {
   // Full stop Cap't. Work on migrating this to a decellerate funct in future.
   digitalWrite(MOTOR_AIN1, HIGH);
   digitalWrite(MOTOR_AIN2, HIGH);
   analogWrite(MOTOR_PWMA,0);
   
   digitalWrite(MOTOR_BIN1, HIGH);
   digitalWrite(MOTOR_BIN2, HIGH);
   analogWrite(MOTOR_PWMB,0);

   WAIT(1000);
   digitalWrite(MOTOR_AIN1, LOW);
   digitalWrite(MOTOR_AIN2, LOW);
   analogWrite(MOTOR_PWMA,0);
   
   digitalWrite(MOTOR_BIN1, LOW);
   digitalWrite(MOTOR_BIN2, LOW);
   analogWrite(MOTOR_PWMB,0);
}

void motor_test_a (ASIZE delay)
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

void motor_test_b (ASIZE delay)
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
  MOTOR_WAKE;

   digitalWrite(MOTOR_BIN1, HIGH);
   digitalWrite(MOTOR_BIN2, LOW);
   analogWrite(MOTOR_PWMB, speed);
   
   digitalWrite(MOTOR_AIN1, HIGH);
   digitalWrite(MOTOR_AIN2, LOW);
   analogWrite(MOTOR_PWMA, speed);

}

void bot_ctl_backward(int speed){
  MOTOR_WAKE;
   digitalWrite(MOTOR_BIN1, LOW);
   digitalWrite(MOTOR_BIN2, HIGH);
   analogWrite(MOTOR_PWMB, speed);
   
   digitalWrite(MOTOR_AIN1, LOW);
   digitalWrite(MOTOR_AIN2, HIGH);
   analogWrite(MOTOR_PWMA, speed);

}

void bot_ctl_backoffturn_left(int speed){
    MOTOR_WAKE;
      // back motor b up by global val
      digitalWrite(MOTOR_BIN1, LOW);
      digitalWrite(MOTOR_BIN2, HIGH);
      analogWrite(MOTOR_PWMB, speed);
      // put motor A to idle
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, LOW);
      analogWrite(MOTOR_PWMA, 0);
}

void bot_ctl_backoffturn_right(int speed){
    MOTOR_WAKE;
      // back motor A up by global val
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      analogWrite(MOTOR_PWMA, speed);
      // put motor B to idle
      digitalWrite(MOTOR_BIN1, LOW);
      digitalWrite(MOTOR_BIN2, LOW);
      analogWrite(MOTOR_PWMB, 0);
}

void bot_ctl_pivot( bool rotation){ // 0 = neg rotation (ie. CCW)  1 = pos rotation (ie. CW)
    MOTOR_WAKE;
    //PRINTF("PIVOT TURN REQESTED ");
    //int test = rotation;
    //PRINTF(test);

    if (rotation) { // Positive rotation. Clock wise pivot to the right.

      // back motor A up by global val
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      analogWrite(MOTOR_PWMA, mtr_ctl_pivot_speed);

      // fwd B
      digitalWrite(MOTOR_BIN1, HIGH);
      digitalWrite(MOTOR_BIN2, LOW);
      analogWrite(MOTOR_PWMB, mtr_ctl_pivot_speed);
           
    } else {
       // FWD motor A up by global val
      digitalWrite(MOTOR_AIN1, HIGH);
      digitalWrite(MOTOR_AIN2, LOW);
      analogWrite(MOTOR_PWMA, mtr_ctl_pivot_speed);

      // BACK  MTR B
      digitalWrite(MOTOR_BIN1, LOW);
      digitalWrite(MOTOR_BIN2, HIGH);
      analogWrite(MOTOR_PWMB, mtr_ctl_pivot_speed);   

    }
}

int sonar_ping(){  // take a sample number of pings and return rounded avg in CM. Sum adds 1cm to reserve 0 as an error situation. since this is only used as bumper and not measuerment we dont care much atm.
    float duration, distance; 
    int sum = 1, cnt = 0; 

  while (cnt < bot_sen_sonar_ping_cnt)
  {
    // Write 10 MicroSec pulse to trigger pin.
   digitalWrite(trigPinFwd,LOW);// ensure set LOW to start. 
   delayMicroseconds(2); 
   digitalWrite(trigPinFwd,HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPinFwd,LOW);

   // measure response
   duration = pulseIn(echoPinFwd,HIGH);
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

int ir_ping(){
  //SharpIR sensor( SharpIR::GP2Y0A41SK0F,IRPin_1); 
  int distance = IRsensorFWD.getDistance();
  return distance;

}

void svc_ping( ASIZE delay){ // PING service function.
    
  while (1)
  {
    bot_sen_sonar_fwd_ping = sonar_ping();
    
    WAIT(delay);
  }  
}

void bot_cruise ( ASIZE delay){ // Cruise Behavior
    WAIT(1000); // get sensors online.
    

  while (1)
  {}
         
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

  if ( up or down or left or right or circle or square){ // an movement key was pressed 

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
      bot_ctl_pivot(0);
      WAIT(delay);
    } 

    if (right)
    {
      bot_ctl_pivot(1);
      WAIT(delay);
    } 
    if (GamePad.isSquarePressed())
    {
      bot_ctl_backoffturn_left(mtr_ctl_pivot_speed ) ;
      WAIT(delay);
    }

    if (GamePad.isCirclePressed())
      {
      bot_ctl_backoffturn_right(mtr_ctl_pivot_speed ) ;
      WAIT(delay);
      }
  } else{
    stop(); // otherwise brake motors
    WAIT(delay);
  }



/*
  if (GamePad.isCrossPressed())
  {
    
  }

  if (GamePad.isTrianglePressed())
  {
    
  }

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
    

    /**************TASKS *******************/
    
    // LMX Tasks
    create_task((char *)"IDLE",cpu_idle,0,MINSTACK);
    create_task((char *)"STATS",stats_task,10000,MINSTACK*4);
    create_task((char *)"SIGNON",signon,1,MINSTACK*4);
    
    // Level 1 System Tasks
    create_task((char *)"LED",led,250, MINSTACK); // heatbeat. kept as example of how to use semaphore setting and getting
    create_task((char *)"FLASH",flash,850,MINSTACK); // heatbeat. kept as example of how to use semaphore setting
    create_task((char *)"CONLOG",console_log,10000,MINSTACK); // logging to serial output

    // Level 2 Services     
    create_task((char *)"PING",svc_ping,100, MINSTACK); // IR and Sonar Ping service

    // Level 3 Controls
    create_task((char *)"BTCTL",bot_bt_input,5, MINSTACK); // BlueTooth contorler



    scheduler();
    PRINTF("Should never get here.");

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