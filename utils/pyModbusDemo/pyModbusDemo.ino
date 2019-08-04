/*****************************************************************************
 * pyModbusDemo.ino

This is  a testing program that works with pyModSlave and ModbusBox ( ESP32)
Modbus Library: https://github.com/luisgcu/SensorModbusMaster/tree/master
pyModSlave : https://sourceforge.net/projects/pymodslave/
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
                                                // Register Group 2
//Registers Group 1 Holding registers                                              
uint16_t          hold_reg_int16[]         = { 1801, 1802, 1803, 1804,1805 };  //Holding registers staring address is 1800, N regs=5
int16_t           hold_reg_int16_data[5]   = { 0, 0, 0, 0,0 };

//Registers Group 2 Input registers                                             
uint16_t          input_reg_int16[]         = { 1, 2, 3, 4,5 };     //input register staring address is 0, Number of regs=5
int16_t           input_reg_int16_data[5]   = { 0, 0, 0, 0, 0 };

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
 readGroup1();    //read holding registers
 delay(5000);
 readGroup2();    //read input registers
 delay(6000);
  
}

  



int16_t holding_read_int16(uint16_t h_input_reg_int16) {                   //Modbus Funtion that Reads 16 bits  Holding registers

  if (modbus.int16FromRegister(0x03, h_input_reg_int16 + Mb_offset, littleEndian)) {   
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

int16_t input_read_int16(uint16_t i_input_reg_int16) {                   //Modbus Funtion that Reads 16 bits  Input registers

  if (modbus.int16FromRegister(0x04, i_input_reg_int16 + Mb_offset, littleEndian)) {   
    return modbus.int16FromFrame(bigEndian);    
     
  }
  else if (modbus.isError) {
    
    #ifdef DEBUG_MB
    Serial.println("\n Int16 Read Register 16 Failed!");   
    #endif
    return 0;
  }
}

//*********************Read group1*************************************//
void readGroup1() {
#ifdef  DEBUG_MB
  // Serial.println("=======================");
  Serial.println("It read  holding register symulated  by pyModSlave  [Group1]");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=======================");
#endif
  uint16_t grp2_lenght = sizeof(hold_reg_int16) / sizeof(uint16_t);
  for (int k = 0; k < grp2_lenght; k++)
  {
    hold_reg_int16_data[k] = holding_read_int16(hold_reg_int16[k]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(hold_reg_int16_data[k], DEC);
    Serial.print(F(", "));
    input_reg_int16_data[k] =0;  //clear reg
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=======================");
  Serial.println("\n\n");
#endif
}

//*********************Read group2*************************************//
void readGroup2() {
#ifdef  DEBUG_MB
  // Serial.println("=======================");
  Serial.println("It read  Input register symulated  by pyModSlave  [Group2]");
  // Serial.println("\n\n");
  Serial.println("=======================");
#endif
  uint16_t grp3_lenght = sizeof(input_reg_int16) / sizeof(uint16_t);
  for (int l = 0; l < grp3_lenght; l++)                                     
  {
    input_reg_int16_data[l] = input_read_int16(input_reg_int16[l]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(input_reg_int16_data[l], DEC);
    Serial.print(F(", "));
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=======================");
  Serial.println("\n\n");
#endif
}
//*********************End Read group3********************************//

