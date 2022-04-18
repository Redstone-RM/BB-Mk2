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

#define LED_HEARTBEAT_PIN 13