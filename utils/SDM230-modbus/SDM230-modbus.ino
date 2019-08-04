/*****************************************************************************
 * SDM230-modbus.ino
ESP-32
This is  a testing program to scan a few input register of the SDM230-Modbus
Modbus Library: https://github.com/luisgcu/SensorModbusMaster/tree/master
Hardware Info : https://github.com/iotbits-us/mbox-hardware-mk
For the ESp32 to work wih this Library and use Serial 1 the following modification ins requierd on the
ESP32 arduino Core. C:\Users\yourusername\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.2\cores\esp32
HardwareSerial.cpp change the ports pins definition as follow:
#ifndef RX1
#define RX1 16
#endif
#ifndef TX1
#define TX1 17
#endif
#ifndef RX2
#define RX2 9
#endif
#ifndef TX2
#define TX2 10
#endif
*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------

#include <SensorModbusMaster.h>
#define DEBUG_MB
// Define the sensor's modbus address
byte              modbusAddress           = 0x01; // The sensor's modbus address, or SlaveID
uint8_t           mb_regsRead_delay       = 4;    // 5ms delay betewen register read.
int               Mb_offset               = -1;   // The modbus Offset
int               _32bitoffset            = -1;   //offset for register 32 bits
const int         DEREPin                 = 4;    // The pin controlling Recieve Enable and Driver Enable
                                                // on the RS485 adapter, if applicable (else, -1)
                                                // Setting HIGH enables the driver (arduino) to send data
                                                // Setting LOW enables the receiver (sensor) to send data

                                             //v,pf,hz,amp
uint16_t          sdm_230_modbus[]         = { 1, 31, 7, 71 };    //Group 1
float             sdm_230_modbus_data[5]    = { 0, 0, 0, 0 };

// Construct the modbus instance
modbusMaster modbus;

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void setup()
{
    pinMode(DEREPin, OUTPUT);

    Serial.begin(115200);  // Main serial port for debugging via USB Serial Monitor
    Serial1.begin(9600);//, SERIAL_8O1);  // port for communicating with sensor
    modbus.begin(modbusAddress, &Serial1, DEREPin);
    
    // Turn on debugging
    // modbus.setDebugStream(&Serial1);    
    // Allow the sensor and converter to warm up
    Serial.println("Waiting for sensor and adapter to be ready.");
    delay(200);

}

// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{  
 readGroup2();
 delay(7000); 
  
}

  

float  read_int32(uint16_t reg_addr) {                         //Modbus Funtion that reads  Float 32 bits register

  if (modbus.int32FromRegister(0x04, reg_addr + Mb_offset , littleEndian)) {
    return modbus.float32FromFrame(bigEndian);
      delay(5);
   
  }
  else if (modbus.isError) {
    #ifdef DEBUG_MB
    Serial.println("n\ Read Register 32 Failed!");
    #endif
    
    return 0;
  }
}

int16_t read_int16(uint16_t int16_reg_addr) {                   //Modbus Funtion that Reads 16 bits registers

  if (modbus.int16FromRegister(0x04, int16_reg_addr + Mb_offset, littleEndian)) {   //was  littleEndian
    return modbus.int16FromFrame(bigEndian);    //was bigEndian
    // delay(5); 
     
  }
  else if (modbus.isError) {
    
    #ifdef DEBUG_MB
    Serial.println("\n Int16 Read Register 16 Failed!");   
    #endif
    return 0;
  }
}

//*********************Read group2*************************************//
void readGroup2() {
#ifdef  DEBUG_MB
  // Serial.println("=======================");
  Serial.println("It read  SDM230 FLOAT 32 bits Modbus registers   [Group2]");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=======================");
#endif
  uint16_t grp2_lenght = sizeof(sdm_230_modbus) / sizeof(uint16_t);
  for (int k = 0; k < grp2_lenght; k++)
  {
    sdm_230_modbus_data[k] = read_int32(sdm_230_modbus[k]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(sdm_230_modbus_data[k], 2);
    Serial.print(F(", "));
    sdm_230_modbus_data[k] =0;  //clear reg
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=======================");
  Serial.println("\n\n");
#endif
}


