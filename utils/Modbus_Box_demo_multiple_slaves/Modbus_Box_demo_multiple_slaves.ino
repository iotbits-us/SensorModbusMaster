/*****************************************************************************
  __  __           _ _                 ____            
 |  \/  |         | | |               |  _ \           
 | \  / | ___   __| | |__  _   _ ___  | |_) | _____  __
 | |\/| |/ _ \ / _` | '_ \| | | / __| |  _ < / _ \ \/ /
 | |  | | (_) | (_| | |_) | |_| \__ \ | |_) | (_) >  < 
 |_|  |_|\___/ \__,_|_.__/ \__,_|___/ |____/ \___/_/\_\
                                                       
                                                                                                                                                     
This is  a very basic and simple  program to get started with  the  Modbus Box communicating  with several modbus slaves
-Slave #1 is the modbus box  as modbus Slave :-->  NOT USED IN THIS DEMO 
-Slave #2 is Control Techniques AC drive Undrive M200 :--> https://acim.nidec.com/en-us/drives/control-techniques/products/ac-drives/unidrive-m/unidrive-m200
-Slave #3 is ABB ACS310 AC drive :--> https://new.abb.com/drives/low-voltage-ac/general-purpose/acs310
-Slave #4 is Control Techniques Commander SK AC drive :--> https://inverterdrive.com/file/Commander-SK-Advanced-User-Guide
-Slave #5 is Eastron SDM230 Energy Meter :--> http://www.eastrongroup.com/productsview/72.html 
                                         :--> http://www.eastrongroup.com/data/uploads/Eastron_SDM230-Modbus_protocol_V1_2.pdf

Modbus Library: https://github.com/luisgcu/SensorModbusMaster/tree/master
Hardware Info : https://github.com/iotbits-us/mbox-hardware-mk
For the ESp32 to work wih this Library and use Serial1 the following modification is requierd on the
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
// Define the Slaves  modbus address
byte              mb_slv                  = 1;    // modbus Box  as Slave  // Not used in this example 
byte              m200_adr                = 2;    // Control Techniques Undirve M200 
byte              abb_adr                 = 3;    // ABB ACS 310
byte              sk_adr                  = 4;    // Control Techniques Commander SK
byte              enr_mtr                 = 5;    // Energy Meter



uint8_t           mb_regsRead_delay       = 4;    // 5ms delay betewen register read.
int               Mb_offset               = -1;   // The modbus Offset
int               _32bitoffset            = -1;   //offset for register 32 bits
const int         DEREPin                 = 4;    // The pin controlling Recieve Enable and Driver Enable
                                                  // on the RS485 adapter, if applicable (else, -1)
                                                  // Setting HIGH enables the driver (arduino) to send data
                                                  // Setting LOW enables the receiver (sensor) to send data
                                                  
//Control Techniques Commander SK Holding registers ( For information about register check commander SK adanced user manual)                                              
uint16_t          sk_hold_reg_int16[]         = { 419, 502, 505, 704,734,1040,1811, 1812, 1813, 1814 };  
long              sk_hold_reg_int16_data[10]  = {   0,   0,   0,  0,  0,  0,  0, 0, 0, 0 };

uint16_t          sk_hold_reg_int32[]         = { 128, 201, 401, 2021 };  //Commander SK 32 bits holding registers
int32_t           sk_hold_reg_int32_data[4]   = {   0,   0,   0,  0  };


//Registers Energy Meter Input  registers SDM 230 / /v, pf, hz,amp,pdm,cdem,tacen,TrEnerg
uint16_t          sdm_230_modbus[]            = { 1, 31, 7, 71, 85, 259, 343, 345  };    //input register in SDM230 Floating point format ( to IEEE 754) 
float             sdm_230_modbus_data[8]      = { 0, 0,  0,  0,  0,  0,  0,  0 };


//Registers Mbox as Slave Holding registers    adress 1                                            
uint16_t          Mbox_hold_reg_int16[]        = { 1811, 1812, 1813, 1814,1815,1816,1817,1818,1819 };  //Holding registers staring Not used in this demo
long              Mbox_hold_reg_int16_data[9]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//Registers M200  Holding registers    adress 2                                            
uint16_t          m200_hold_reg_int16[]        = { 1811, 1812, 1813, 1814,1815 };  //Holding registers 
long              m200_hold_reg_int16_data[5]  = { 0, 0, 0, 0, 0 };

//Registers ABB ACS310  Holding registers   adress 3 rpm , rpm, hz, amp,torq,kw,outv,                                          
uint16_t          ACS310_hold_reg_int16[]         = { 101,102,103,104,105,106,109,110, 4, 5, 1 };  //Holding registers 
long              ACS310_hold_reg_int16_data[11]  = {   0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0 };


/*
//Registers Group 2 Input registers     Pymod slave                                         
uint16_t          input_reg_int16[]         = { 1, 2, 3, 4,5 };     //input register staring address is 0, Number of regs=5
long              input_reg_int16_data[5]   = { 0, 0, 0, 0, 0 };
*/


// Construct the modbus instance
modbusMaster modbus;

// ---------------------------------------------------------------------------
// Main setup function
// ------------------------------------------------------- --------------------
void setup()
{
    pinMode(DEREPin, OUTPUT);

    Serial.begin(115200);  // Main serial port for debugging via USB Serial Monitor
    Serial1.begin(38400);//, SERIAL_8O1);  // port for communicating with slaves
    modbus.begin(sk_adr, &Serial1, DEREPin);    
    // Turn on debugging
    // modbus.setDebugStream(&Serial1);    
    // Allow the sensor and converter to warm up
    Serial.println("Waiting for Slaves  and Mster to be ready.");
    delay(200);

}

// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{   
  read_sk();    //read  holding  register commander SK
 delay(200);
  read_sk32();  //read SK 32 bits register
   
 delay(5000);
  Serial1.begin(9600);  // Set serial baoud rate for SDm230 
  modbus.begin(enr_mtr, &Serial1, DEREPin); // Modbus driver begin
 delay(100);       
  read_sdm();    //Call subrutine to read  SDM230 FLOAT 32 bits Modbus  
   
 //modbus.begin(mb_slv, &Serial1, DEREPin); // set slave id  mbox
// delay(100);  
 //read_mbox();    //Read  holding register from Mbox Slave
 
delay(5000);     
  Serial1.begin(38400);//Set serial baoud rate for M200 drive
delay(200);
 modbus.begin(m200_adr, &Serial1, DEREPin); // Modbus driver begin M200 AC drive
 delay(100); 
 read_m200();    //Read  holding register from drive M200  

  delay(5000);  
  modbus.begin(abb_adr, &Serial1, DEREPin); //Modbus driver begin for abb  AC drive
 delay(100);
 read_abb();    //read holding register from ABB AC drive
 
 delay(5000); 
 modbus.begin(sk_adr, &Serial1, DEREPin);    // Set Modbus drives fr control Techniques commander SK 
 delay(100);
  
}

  





//*********************Read Commander SK holding registers *************************************//
void read_sk() {
#ifdef  DEBUG_MB
  // Serial.println("=============================");
  Serial.println("It read  holding register from Commader SK");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=============================");
#endif
  uint16_t grp2_lenght = sizeof(sk_hold_reg_int16) / sizeof(uint16_t);
  for (int k = 0; k < grp2_lenght; k++)
  {
    sk_hold_reg_int16_data[k] = holding_read_int16(sk_hold_reg_int16[k]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(sk_hold_reg_int16_data[k], DEC);
    Serial.print(F(", "));    
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=============================");
  Serial.println("\n\n");
#endif
}

//*********************Read Commander Sk32 bits reg *************************************//
void read_sk32() {
#ifdef  DEBUG_MB
  // Serial.println("=============================");
  Serial.println("It read  32 bits register from Commader SK");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=============================");
#endif
  uint16_t grpsk_lenght = sizeof(sk_hold_reg_int32) / sizeof(uint16_t);
  for (int v = 0; v < grpsk_lenght; v++)
  {
    sk_hold_reg_int32_data[v] = read_int32b(sk_hold_reg_int32[v]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(sk_hold_reg_int32_data[v], DEC);
    Serial.print(F(", "));    
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=============================");
  Serial.println("\n\n");
#endif
}

//*********************Read holding register from drive M200 *************************************//
void read_m200() {
#ifdef  DEBUG_MB
  // Serial.println("=============================");
  Serial.println("It read  holding register from M200");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=============================");
#endif
  uint16_t grp2_lenght = sizeof(m200_hold_reg_int16) / sizeof(uint16_t);
  for (int d = 0; d < grp2_lenght; d++)
  {
    m200_hold_reg_int16_data[d] = holding_read_int16(m200_hold_reg_int16[d]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(m200_hold_reg_int16_data[d], DEC);
    Serial.print(F(", "));    
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=============================");
  Serial.println("\n\n");
#endif
}

//*********************Read Mbox as Slave ( not used in this demo) *************************************//
void read_mbox() {
#ifdef  DEBUG_MB
  // Serial.println("=============================");
  Serial.println("It read  holding  register symulated  ModbusBox");
  // Serial.println("\n\n");
  Serial.println("=============================");
#endif
  uint16_t grp3_lenght = sizeof(Mbox_hold_reg_int16) / sizeof(uint16_t);
  for (int l = 0; l < grp3_lenght; l++)                                     
  {
    Mbox_hold_reg_int16_data[l] = holding_read_int16(Mbox_hold_reg_int16[l]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print( Mbox_hold_reg_int16_data[l], DEC);
    Serial.print(F(", "));
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=============================");
  Serial.println("\n\n");
#endif
}
//*********************End Read Mbox********************************//

//*********************Read Energy Meter SDM230*************************************//
void read_sdm() {
#ifdef  DEBUG_MB
  // Serial.println("=============================");
  Serial.println("It read  SDM230 FLOAT 32 bits Modbus registers   ");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=============================");
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
  Serial.println("=============================");
  Serial.println("\n\n");
#endif
}
//
//*********************Read ABB ACS310 Holding registers*************************************// 
void read_abb() {
#ifdef  DEBUG_MB
  // Serial.println("=============================");
  Serial.println("It read  holding register from ABB ACS310");  //<<-----
                                                // Serial.println("\n\n");
  Serial.println("=============================");
#endif
  uint16_t grp2_lenght = sizeof(ACS310_hold_reg_int16) / sizeof(uint16_t);
  for (int x = 0; x < grp2_lenght; x++)
  {
    ACS310_hold_reg_int16_data[x] = holding_read_int16(ACS310_hold_reg_int16[x]);
    delay(mb_regsRead_delay);
#ifdef  DEBUG_MB
    Serial.print(ACS310_hold_reg_int16_data[x], DEC);
    Serial.print(F(", "));    
#endif
  }
#ifdef  DEBUG_MB
  Serial.println();
  Serial.println("=============================");
  Serial.println("\n\n");
#endif
}

//

int16_t holding_read_int16(uint16_t h_input_reg_int16) {                   //Modbus Funtion that Reads 16 bits  Holding registers

  if (modbus.int16FromRegister(0x03, h_input_reg_int16 + Mb_offset, littleEndian)) {   
    return modbus.int16FromFrame(bigEndian);    //was bigEndian
     delay(3); 
     
  }
  else if (modbus.isError) {
    
    #ifdef DEBUG_MB
    Serial.println("\n Int16 Read Register 16 Failed!");   
    #endif
    return 0;
  }
}

int input_read_int16(uint16_t i_input_reg_int16) {                   //Modbus Funtion that Reads 16 bits  Input registers

  if (modbus.int16FromRegister(0x04, i_input_reg_int16 + Mb_offset, littleEndian)) {   
    return modbus.int16FromFrame(bigEndian);    
    delay(3);  
  }
  else if (modbus.isError) {
    
    #ifdef DEBUG_MB
    Serial.println("\n Int16 Read Register 16 Failed!");   
    #endif
    return 0;
  }
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

int32_t  read_int32b(uint16_t reg_addr) {                         //Modbus Funtion that reads  int32 bits regs

  if (modbus.int32FromRegister(0x03, reg_addr + 16383 , littleEndian)) {
    return modbus.int32FromFrame(bigEndian,3);
      delay(5);
   
  }
  else if (modbus.isError) {
    #ifdef DEBUG_MB
    Serial.println("n\ Read Register 32 Failed!");
    #endif
    
    return 0;
  }
}

