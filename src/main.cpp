/*############################################################################## */
#define VERSION "Bum Biter MK-2.2.0"
#define DEBUG true

#include "botmsg.h" // botmsg code for Communications 
#include "mtrctl.h" // motor control functions 
#include "sensors.h" // sensor related code
/*############################################################################## */

/*######################## libtask LMX ######################################### <dpa> */
#include <stdio.h> // LMX 
#include <task.h> // LMX
#include <log.h> // LMX
#include <sysclock.h> // LMX
/* 
Choose one of the below:  
#define WAIT(d)  { d *= 10; cnt = 0; while (cnt++ < d) defer(); }
#define WAIT(d)  { msleep(d); }
#define WAIT(d)  { wake_after(d); }
*/
#define WAIT(d)  { wake_after(d); }
#define BAUDRATE 57600
#define PRINTF Serial.println
#define SPRINTF sprintf

/* --------------- System ----------------------- */
#include <Arduino.h> 
#include <util/atomic.h> // Using ATOMIC_BLOCK macro for wheel encoder reading of volitile ints.

/* --------------- Sensors ------------------------ */ 
#include <SharpIR.h> // IR Distance Sensors

/* --------------- BlueTooth ------------------------ */  
#include <Dabble.h> // Dabble Bluetooth Contoller. https://thestempedia.com/docs/dabble
#define CUSTOM_SETTINGS // You can reduce the size of compiled library by enabling only modules you want to use. First define CUSTOM_SETTINGS followed by #define INCLUDE_modulename
#define INCLUDE_GAMEPAD_MODULE // Include Dabble Gamepad module https://thestempedia.com/docs/dabble/game-pad-module
 
/* --------------- I2C  ------------------------ */   
#include <I2CTransfer.h> // I2C Datum TRANSFER from ROS2 Controler. see https://github.com/PowerBroker2/SerialTransfer
#define I2C_ADDR 9  // I2C Slave Address 
I2CTransfer  myTransfer; // create I2C Transfer Obj 

struct ctrlmsg { 
  float x;
  float z;
  char  debug[8];
} botmsg;  // ROS2 Controler> ctrlmsg> I2C"datum"> I2C Callback> statmsg> I2C> ROS2> Topic


/* ------------ <dpa> -------------------------- */
void printkbuf(char *s) {  // dpa
   PRINTF(s);
}



/* Layer 1 function alias definitions    */
//  heartbeat LED .
#define LED1_ON  digitalWrite(LED_HEARTBEAT_PIN, HIGH) // onboard LED used for heartbeat.
#define LED1_OFF digitalWrite(LED_HEARTBEAT_PIN, LOW)  // onboard LED used for heartbeat.


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

String Serialdata = ""; // Output String object for BlueTooth Serial Terminal output.
bool dataflag = 0;

SharpIR IRsensorRight( SharpIR::GP2Y0A41SK0F,IRPin_1);  // RIGHT IR sensor object. 
SharpIR IRsensorLeft( SharpIR::GP2Y0A41SK0F,IRPin_2);  // LEFT IR sensor object. 


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

/* LVL 3 INIT Functions */

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

void svc_I2C_conn (ASIZE delay){ // Internal I2C Data Exchange Service
  while(1){
    txData.z               = botmsg.z; // Float. Currently demanded Z. Feedback for ctrlmsg
    txData.mtr_pos_right   = mtr_sen_pos_a; // right motor encoder position
    txData.mtr_pos_left    = mtr_sen_pos_b;  // left motor encoder position
    txData.mtr_speed_right = mtr_ctl_speed_a;  // motor a speed
    txData.mtr_speed_left  = mtr_ctl_speed_b;  // motor b speed
    txData.sen_sonar_fwd   = bot_sen_sonar_fwd_ping; // forward sonar value
    txData.sen_sonar_rear  = bot_sen_sonar_rear_ping; // rear sonar value
    txData.sen_ir_right    = bot_sen_ir_right_ping; // right IR value
    txData.sen_ir_left     = bot_sen_ir_left_ping; // left IR value 
    txData.x               = botmsg.x; // Float. Currently demanded X. Feedback for ctrlmsg
    strcpy (txData.debug, "INFO"); // short logging message

    botmsg.x =  rxData.x; 
    botmsg.z =  rxData.z;
    strcpy(botmsg.debug,rxData.debug);

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

void svc_motor_test (ASIZE delay)

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
// I2C TEst
        // set up I2C
    Wire.begin(I2C_ADDR); // join i2c bus
    Wire.onReceive(receiveEvent); // register function to be called when a message arrives
    Wire.onRequest(requestEvent); // register function to be called when a request arrives


/* Serial Transfer Library // backup working code

    Wire.begin(I2C_ADDR);// 
    configST I2C_myConfig; 
    I2C_myConfig.debug        = true;
    I2C_myConfig.callbacks    = callbackArr;
    I2C_myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
    myTransfer.begin(Wire, I2C_myConfig);
*/

    printv = printkbuf;

    PRINTF("Howdy Console!\n");
        
    #if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */ // <dpa> libtask set in task.h
    delay(1500);   /* hardware delay, sysclock not running yet */
    #endif

    pid_count = 0; current = 0;


   // create_task((char *)"MTRTST",svc_motor_test,5000, MINSTACK );   
   
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
    return ;
    #endif
}

void loop() 
{
  /* Noting to see here. Move along. Not the droid you're looking for. */
  asm("nop");
}
/* EOF */