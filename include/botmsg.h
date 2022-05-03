#pragma once
#include <Arduino.h> 
#include <Dabble.h> // https://github.com/STEMpedia/Dabble   https://thestempedia.com/docs/dabble 



//===========  BLE Communication  =============
//  - Define a struct. To hold the data needed to control the motors
//  - The received data is copied byte by byte into pcLine
//  - A working instance of the Union is created and elements in it are referred to as, e.g. inputData.moveData.z

#define BLE_MSG_BYTESIZE 11 // Let's keep things Byte Sized 

// MoveItem Struct for this is the data needed to control the motors
struct moveItem {
    float x;                    // 4 bytes Twist X
    float z;                    // 4       Twist Z 
    char cmd[3];                // 3       2 Chararter command variable
                                //====
                                // 11 Bytes  // Define this number as BLE_MSG_BYTESIZE
}__attribute__( ( packed ) );

// Define a union joining the struct and a byte array of the same size
union inputFromPC {  //  // The struct is overlaid on an array. The received data is copied byte by byte into pcLine and can then be used as, e.g. moveData.x
   moveItem moveData;
   byte pcLine[BLE_MSG_BYTESIZE]; 
};

inputFromPC inputData; // Create a working instance of the Union. Elements in it are referred to as, e.g. inputData.moveData.z
byte pcData[BLE_MSG_BYTESIZE]; 
boolean newData = false;



void receiveData() {   // Reads BLE_MSG_BYTESIZE Bytes of serial data.
   if (Dabble.DabbleSerial->available() < BLE_MSG_BYTESIZE) { // DabbleSerial provides a software serial interface for the HM10 BLE Module
     // error 
     return;
   }
   for (byte n = 0; n < BLE_MSG_BYTESIZE; n++) { // for each byte 
      pcData[n] = Dabble.DabbleSerial->read(); // Read bytes one at a time
   }
   // TODO check CRC
   for (byte n = 0; n < BLE_MSG_BYTESIZE; n++) {
     inputData.pcLine[n] = pcData[n];  // Here is where the struct is overlaid on an array to make it easy to receive data from Serial device
   }
   newData = true;
}



struct statusItem {
  char msg[14];
//    int x;                    // 2 bytes  X Twist status confirmation. A float to int conversion to save 2 BLE bytes since we're only confirming changes.
//    int z;                    // 2        Z Twist status confirmation. A float to int conversion to save 2 BLE bytes since we're only confirming changes.
//    int mtr_pos_right;        // 2
//    int mtr_pos_left;         // 2
//    int mtr_speed_right;      // 2
//    int mtr_speed_left;       // 2
//    int sen_sonar_fwd;        // 2
//    int sen_sonar_rear;       // 2
//    int sen_ir_right;         // 2
//    int sen_ir_left;          // 2
                              //====
                              // 20 Bytes  // #define BLE_STATUS_BYTE_SIZE 20
}__attribute__( ( packed ) );


#define BLE_STATUS_BYTE_SIZE 14
byte statusData[BLE_STATUS_BYTE_SIZE]; 

// Define a union joining the struct and a byte array of the same size
union outputToPC {  //  // The struct is overlaid on an array. The received data is copied byte by byte into pcLine and can then be used as, e.g. moveData.x
   statusItem statusData;
   uint8_t bytes[BLE_STATUS_BYTE_SIZE]; 
};

outputToPC outputData; // Create a working instance of the Union. Elements in it are referred to as, e.g. outputData.statusData.mtr_pos_right
/*
void sendBLEData() {   // SEND  serial data.
   if (Dabble.DabbleSerial->availableForWrite() < BLE_STATUS_BYTE_SIZE ) { // DabbleSerial provides a software serial interface for the HM10 BLE Module
     // TBD Handle this better. Error or serial buffer full for some reason. avoid blocking calls first check the amount of free space in the transmit buffer using availableForWrite().
     return;
   }
   for (byte n = 0; n < BLE_STATUS_BYTE_SIZE; n++) { // for each byte      
      Dabble.DabbleSerial->write(outputData.bytes[n]); // Write bytes one at a time 
   }
}
*/

void sendBLEData() {   // SEND  serial data.
   if (Dabble.DabbleSerial->availableForWrite() < BLE_STATUS_BYTE_SIZE ) { // DabbleSerial provides a software serial interface for the HM10 BLE Module
     // TBD Handle this better. Error or serial buffer full for some reason. avoid blocking calls first check the amount of free space in the transmit buffer using availableForWrite().
     return;
   }
  // Dabble.DabbleSerial->write("<abcdefghijk>");  // Start and ending string markers  < string > 
  // strcpy(outputData.statusData.msg, "<abcdefghijk>");
  strcpy(outputData.statusData.msg, "<0123456789A>");
  Dabble.DabbleSerial->write(outputData.bytes, sizeof(outputData.bytes)); // Write bytes 

}

void displayData() { // this is run when you want to dump the ble msg vars to the serial monitor.
  if (newData == false) {
     
  } else {
    Serial.print("<BLE.Input.Msg::: ");
    Serial.print(inputData.moveData.x);
    Serial.print(" ");
    Serial.print(inputData.moveData.z);
    Serial.print(" ");
    Serial.print(inputData.moveData.cmd);
    Serial.print(" ");
    Serial.println('>');
  }
  // Serial.print("Struct: "  +  String(sizeof(outputData.statusData)) + " " +  String( sizeof(outputData.statusData.x)) );
  Serial.print("<BLE.Output.Status::: ");
 // Serial.print(outputData.statusData.x);
 // Serial.print(" ");
 // Serial.print(outputData.statusData.z);
  /*
  Serial.print(" ");
  Serial.print(outputData.statusData.mtr_pos_right);
  Serial.print(" ");
  Serial.print(outputData.statusData.mtr_pos_left);
  Serial.print(" ");
  Serial.print(outputData.statusData.mtr_speed_right);
  Serial.print(" ");
  Serial.print(outputData.statusData.mtr_speed_left);
  Serial.print(" ");
  Serial.print(outputData.statusData.sen_sonar_fwd);
  Serial.print(" ");
  Serial.print(outputData.statusData.sen_ir_right);
  Serial.print(" ");
  Serial.print(outputData.statusData.sen_sonar_rear);
  Serial.print(" ");
  Serial.print(outputData.statusData.sen_ir_left);
  */
  Serial.print(" ");
  Serial.println('>');



}
