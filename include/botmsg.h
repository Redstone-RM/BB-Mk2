#pragma once
#include <Arduino.h>
#include <Wire.h>



struct I2cTxStruct { // A status msg passed back and forthe from an I2C connected board.
  float x; //  twist  current requested X value. Feedback  // 4 bytes
  float z; //  twist  current requested Z value. Feedback  // 4 bytes
  char  debug[8]; // short logging message 7 Char  + Null  // 8 bytes
  int   mtr_pos_right; // right motor encoder position     // 2 bytes
  int   mtr_pos_left; // left motor encoder position       // 2 bytes
  int   mtr_speed_right;  // motor a speed                 // 2 bytes
  int   mtr_speed_left;  // motor b speed                  // 2 bytes
  int   sen_sonar_fwd; // forward sonar value              // 2 bytes
  int   sen_sonar_rear; // rear sonar value                // 2 bytes
  int   sen_ir_right; // right IR value                    // 2 bytes
  int   sen_ir_left; // left IR value                      // 2 bytes
  
} ;                                                        // --------
                                                           // 32 Bytes



struct I2cRxStruct { // A status msg passed back and forthe from an I2C connected board.
  float x; //  twist  current requested X value. Feedback  // 4 bytes
  float z; //  twist  current requested Z value. Feedback  // 4 bytes
  char  debug[8]; // short logging message 7 Char  + Null  // 8 bytes
  int   mtr_pos_right; // right motor encoder position     // 2 bytes
  int   mtr_pos_left; // left motor encoder position       // 2 bytes
  int   mtr_speed_right;  // motor a speed                 // 2 bytes
  int   mtr_speed_left;  // motor b speed                  // 2 bytes
  int   sen_sonar_fwd; // forward sonar value              // 2 bytes
  int   sen_sonar_rear; // rear sonar value                // 2 bytes
  int   sen_ir_right; // right IR value                    // 2 bytes
  int   sen_ir_left; // left IR value                      // 2 bytes

} ;                                                        // --------
                                                           // 32 Bytes



//============
bool newTxData = false;
bool newRxData = false;
bool rqSent = false;

I2cTxStruct txData = {0, 0, "INIT"};
I2cRxStruct rxData;

        // this function is called by the Wire library when a message is received
void receiveEvent(int numBytesReceived) {

    if (newRxData == false) {
            // copy the data to rxData
        Wire.readBytes( (byte*) &rxData, numBytesReceived);
        newRxData = true;
    }
    else {
            // dump the data
        while(Wire.available() > 0) {
            byte c = Wire.read();
        }
    }
}

//===========

void requestEvent() {
    Wire.write((byte*) &txData, sizeof(txData));
    rqSent = true;
}

