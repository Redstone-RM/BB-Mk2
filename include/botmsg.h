#pragma once
#include <Arduino.h>
#include <Dabble.h>



//=========== Start BLE =============
//  - Define a struct. To hold the data needed to control the motors
//  - The received data is copied byte by byte into pcLine
//  - A working instance of the Union is created and elements in it are referred to as, e.g. inputData.moveData.z
// 
// 
// this is the data needed to control the motors
struct moveItem {
    float x;                    // 4 bytes Twist X
    float z;                    // 4       Twist Z 
    char cmd[3];                // 3       2 Chararter command variable
                                //====
                                // 11 Bytes  // Define this number as BLE_MSG_BYTESIZE
};

#define BLE_MSG_BYTESIZE 11 // Let's keep things Byte Sized 

   // The received data is copied byte by byte into pcLine
  //   and can then be used as, e.g. moveData.x
union inputFromPC {
   moveItem moveData;
   byte pcLine[BLE_MSG_BYTESIZE];
};

 // this creates a working instance of the Union
 // elements in it are referred to as, e.g. inputData.moveData.z
inputFromPC inputData;

byte pcData[BLE_MSG_BYTESIZE]; 
boolean newData = false;

void receiveData() {   // Reads 11 Bytes of serial data.
   if (Dabble.DabbleSerial->available() < BLE_MSG_BYTESIZE) { // DabbleSerial Emulates a serial connection for the HM10 BLE  Module
     // error 
     return;
   }
   for (byte n = 0; n < BLE_MSG_BYTESIZE; n++) { // for each byte 
      pcData[n] = Dabble.DabbleSerial->read(); // Read bytes one at a time
   }
   // TODO check CRC
   for (byte n = 0; n < BLE_MSG_BYTESIZE; n++) {
     inputData.pcLine[n] = pcData[n];  // The struct is overlaid on an array to make it easy to receive data from the PC
   }
   newData = true;
}

void displayData() {
  if (newData == false) {
     return;
  }
  Serial.print("<BLE.Msg::: ");
  Serial.print(inputData.moveData.x);
  Serial.print(" ");
  Serial.print(inputData.moveData.z);
  Serial.print(" ");
  Serial.print(inputData.moveData.cmd);
  Serial.print(" ");
  Serial.println('>');
 
}



