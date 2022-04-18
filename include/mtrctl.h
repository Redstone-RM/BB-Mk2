#pragma once

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

// Wake up motor driver from low power standby
#define MOTOR_WAKE digitalWrite(MOTOR_STBY, HIGH) 

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


void stop() {
  MOTOR_WAKE;
  // Brakes // //Aye Capt! FULL STOP!
  mtr_ctl_a(0,0); // set motors to zero
  mtr_sen_stat_a = 0;

  mtr_ctl_b(0,0);
  mtr_sen_stat_b = 0;

  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA,0);
   
  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, LOW);
  analogWrite(MOTOR_PWMB,0); 

}



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
  // mtr_cmd_b_PID.SetMode(AUTOMATIC);
  // mtr_cmd_a_PID.SetMode(AUTOMATIC);
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
