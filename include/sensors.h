#pragma once
#include <task.h>
#define WAIT(d)  { wake_after(d); }

/********** SENSOR GLOBALS **********/
int bot_sen_sonar_fwd_ping = 1; // Distance reported from fwd Ultra-Sonic Sensor rounded up to whole CM. Starts at 1 because <2 returns 0 as an out of bounds error cond. 1 is an Unread cond.
int bot_sen_sonar_rear_ping = 1; // Distance reported from rear Ultra-Sonic Sensor rounded up CM. Starts at 1 because <2 returns 0 as an out of bounds error cond. 1 is an Unread cond.
int bot_sen_sonar_ping_cnt = 5; // how many pings to sample for avg 

int bot_sen_ir_right_ping = 0 ; // Distance reported from Right Analog IR 
int bot_sen_ir_left_ping = 0 ; // Distance reported from Left Analog IR 
bool bot_sen_ir_fwd_ping = 0 ; // Fwd Digital IR 
bool bot_sen_ir_rear_ping = 0 ; // Rear Digital IR

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
#define IR_SENSOR_1 GP2Y0A41SK0F // Model of Sarp IR distance sensor for FWD IR
#define IR_SENSOR_2 GP2Y0A41SK0F // Model of Sharp IR distance sensor for FWD IR
  
// Sharp Digital IR distance Sensors 2YD021 - Disabled Mar 12 2022 RM.
// #define IRPin_FWD 4 
// #define IRPin_Rear 5

/* Layer 1 function alias definitions    */
//heartbeat LED .
#define LED_HEARTBEAT_PIN 13
#define LED1_ON  digitalWrite(LED_HEARTBEAT_PIN, HIGH) // onboard LED used for heartbeat.
#define LED1_OFF digitalWrite(LED_HEARTBEAT_PIN, LOW)  // onboard LED used for heartbeat.

void init_2_sonar_setup(){
    pinMode(trigPinFwd, OUTPUT);
    pinMode(echoPinFwd, INPUT);
    pinMode(trigPinRear, OUTPUT);
    pinMode(echoPinRear, INPUT);
}

