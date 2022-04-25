#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Dabble.h>


//=========== Start i2c =============
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
  byte padding[10]; // not sure                            // 10 bytes
} ;                                                        // --------
                                                           // 42 Bytes



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
  byte  padding[10];                                        // 10 bytes
} ;                                                        // --------
                                                            // 42 Bytes

//============
bool newTxData = false;
bool newRxData = false;
bool rqSent = false;

I2cTxStruct txData = {0, 0, "INIT001"};
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


void requestEvent() {
    Wire.write((byte*) &txData, sizeof(txData));
    rqSent = true;
    Serial.println ("I2C Request recieved");
}

//=========== END i2c =============

//=========== Start BLE =============

  // this is the data needed to control the motors
  /*
struct moveItem {
    unsigned long totalMicros; // 4 bytes
    int xInterval;              // 2
    int yInterval;              // 2
    int zInterval;              // 2
    byte xorVal;                // 1
                                //====
                                // 11 Bytes
};
*/
struct moveItem {
    float x;                    // 4 bytes
    float z;                    // 4
    int testval;                // 2
    char debug[1];              // 1    
                                //====
                                // 11 Bytes
};
  // and the struct is overlaid on an array to make it easy to receive data from the PC
  // the received data is copied byte by byte into pcLine
  //   and can then be used as, e.g. moveData.totalMicros
union inputFromPC {
   moveItem moveData;
   byte pcLine[11];
};

 // this creates a working instance of the Union
 // elements in it are referred to as, e.g. inputData.moveData.totalMicros
inputFromPC inputData;

byte pcData[11];
boolean newData = false;
boolean askForData = true;




void displayData() {
  if (newData == false) {
     return;
  }
  Serial.print("<BLE Message recieved ::: ");
  Serial.print(inputData.moveData.x);
  Serial.print(" ");
  Serial.print(inputData.moveData.z);
  Serial.print(" ");
  Serial.print(inputData.moveData.debug);
  Serial.print(" ");
  Serial.print(inputData.moveData.testval);
  Serial.print(" ");
  
  Serial.println('>');
  newData = false;
  askForData = true;
}



