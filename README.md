# SensorModbusMaster

This library is designed to use an Arduino as a modbus master to communicate with a sensor/slave via [modbus RTU](https://en.wikipedia.org/wiki/Modbus).  It's specifically written with lots of "higher-level" functions to help out users who are largely unfamiliar with the modbus protocol and want an easy way to get information from a modbus device.

*Note: Do not forget to check the ModbusBox examples at the end of this page.* 

_____

## Using the library

To communicate with a modbus sensor or other modbus slave, first create a stream instance (ie, Serial1, SoftwareSerial, AltSoftSerial) and then an instance of the modbusMaster.

```cpp
// Create the stream instance
HardwareSerial modbusSerial = Serial1;  // ALWAYS use HardwareSerial if it's an option
// OR
// AltSoftSerial modbusSerial;  // AltSoftSerial should be your second choice, if your board is supported
// OR
// SoftwareSerial modbusSerial(txPin, rxPin);  // SoftwareSerial should be your last choice.

// Create the modbus instance
modbusMaster modbus;
```

Within the setup function begin both the serial instance and the modbusMaster instance.  The enable pin allows you to use an RS485 to TTL adapter that is half-duplex with a pin that enables data sending.

```cpp
// Start the stream
modbusSerial.begin(baudRate);

// start the modbus
modbus.begin(modbusSlaveID, modbusSerial, enablePin);
```

Once you've created and begun these, getting data from or adding data to a register is very simple:

```cpp
// Retrieve a 32-bit big endian float from input register 15 (input registers are called with 0x04)
modbus.float32FromRegister(0x04, 15, bigEndian);

// Write the value "56" to holding register 20 as a little-endian unsigned 16-bit integer
modbus.uint16ToRegister(20, 56, littleEndian);
```

The following data types are supported:
- uint16 (16-bit unsigned integer)
    - Value must be within a single 16-bit register
    - bigEndian or littleEndian can be specified, bigEndian will be used by default
    - By default, the modbus command for pre-setting a single register will be used (0x06).  Set the forceMultiple boolean flag to 'true' to force the use of the modbus command for setting multiple resisters (0x10).
- int16 (16-bit signed integer)
    - Value must be within a single 16-bit register
    - bigEndian or littleEndian can be specified, bigEndian will be used by default
    - By default, the modbus command for pre-setting a single register will be used (0x06).  Set the forceMultiple boolean flag to 'true' to force the use of the modbus command for setting multiple resisters (0x10).
- float32 (32-bit float)
    - Value must be in two adjacent 16-bit registers
    - bigEndian or littleEndian can be specified, bigEndian will be used by default
    - Only "fully" big or little endianness is supported - that is both high byte and high word first or both low byte and low word first.
- uint32 (32-bit unsigned integer)
    - Value must be in two adjacent 16-bit registers
    - bigEndian or littleEndian can be specified, bigEndian will be used by default
    - Only "fully" big or little endianness is supported - that is both high byte and high word first or both low byte and low word first.
- int32 (32-bit signed integer)
    - Value must be in two adjacent 16-bit registers
    - bigEndian or littleEndian can be specified, bigEndian will be used by default
    - Only "fully" big or little endianness is supported - that is both high byte and high word first or both low byte and low word first.
- char (c++/ASCII style characters)
    - Characters can be in one or more contiguous 16-bit registers
    - Length of the character array must be specified
    - By default, the modbus command for pre-setting a single register will be used (0x06) if the character array has two or fewer characters.  Set the forceMultiple boolean flag to 'true' to force the use of the modbus command for setting multiple resisters (0x10).
- String (Arduino Strings)
    - Characters can be in one or more contiguous 16-bit registers
    - By default, the modbus command for pre-setting a single register will be used (0x06) if the String has two or fewer characters.  Set the forceMultiple boolean flag to 'true' to force the use of the modbus command for setting multiple resisters (0x10).
- pointer (pointers to other registers)
    - Value must be within a single 16-bit register
    - By default, the modbus command for pre-setting a single register will be used (0x06).  Set the forceMultiple boolean flag to 'true' to force the use of the modbus command for setting multiple resisters (0x10).

There are also mid-level functions available to help to reduce serial traffic by calling many registers at once and low level functions to make raw modbus calls.  See SensorModbusMaster.h for all the available functions and their required and optional inputs
_____


## Notes on modbus maps
While modbus RTU specifications define the format of a data frame and a very simple data structure for a master and slave, there are no specification for what types of data a slave stores, where it is stored, or in what format it is stored.  You **MUST** get this information from the manufacturer/programmer of your modbus device.  Typically this information is shared in what is called a modbus map.

You need the following data from your modbus map:
- the baud rate the device communicaes at (modbus allows any baud rate)
- the parity the device uses on the serial line (modbus technically allows 8O1, 8E1, and 8N2, though some devices may use 8N1)
- the type of register or coils the data you are interested is stored in (ie, holding register, input register, coil, or discrete input)
    - Note - This library does not currently support getting or setting values in coils.
- the register or coil number the data is stored in
- the format of data within the registers/coils (ie, float, integer, bitmask, ascii text)
- whether multi-register numeric data is stored as ["big-endian" or "little-endian"](https://en.wikipedia.org/wiki/Endianness) values (That is, is it high _word_ first or low _word_ first.  There are no specifications for this.)
- whether single-register data is stored "big-endian" or "little-endian" (That is, is it high _byte_ first or low _byte_ first.  Modbus specifies that it should be high byte first (big-endian), but devices vary.)
    - Note - This library only supports data that is "fully" big or little endian.  That is, data must be both high byte and high word first or both low byte and low word first.

Without this information, you have little hope of being able to communicate properly with the device.  You can use programs like [CAS modbus scanner](http://www.chipkin.com/cas-modbus-scanner/) to find a device if its address, baud rate, and parity are unknown, but it may take some time to make a connection.  You can also use the "scanRegisters" utility in this library to get a view of all the registers, but if you don't have a pretty good idea of what you are looking for that will not be as helpful as you might hope.
_____


## TTL and RS485/RS322
While modbus RTU specifications define the format of a data frame transfered over a serial line, the type of serial signal is not defined.  Many modbus sensors communicate over [RS-485](https://en.wikipedia.org/wiki/RS-485).  To interface between RS485 and the TTL used by standard Arduino-type boards, you will need an RS485-to-TTL adapter. There are a number of RS485-to-TTL adapters available.  When shopping for one, be mindful of the logic level of the TTL output by the adapter.  The MAX485, one of the most popular adapters, has a 5V logic level in the TTL signal.  This will _fry_ any board that can only use on 3.3V logic.  You would need a voltage shifter in between the Mayfly and the MAX485 to make it work.  Also note that most RS485-to-TTL adapters are implemented _without_ automatic flow control.  That is, you must manually set voltages on driver enable and receiver enable pins to control the data flow direction.  While this library includes functions for setting the enables, I've found commutation to be much more stable on adapters with built-in flow control.  You will also need an interface board to communicate between an Arduino and any modbus sensor that communicates over [RS422](https://en.wikipedia.org/wiki/RS-422) or [RS232](https://en.wikipedia.org/wiki/RS-232).  Again, mind your voltages and the method of direction control.

------

### ModbusBox .

Following examples are to be used with our ModbusBox powered by ESP32 

Here you can find Modbus Master Hardware specs: [Modbus Box](https://github.com/iotbits-us/mbox-hardware-mk)

The following examples   use Serial1 on the ESP32 ,   the following modification  are required on the ESP32 arduino Core.  C:\Users\yourusername\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.2\cores\esp32
**HardwareSerial.cpp** change the ports pins definition as follow:

```
#ifndef RX1
#define RX1 16 //Pin used by Modbus Box
#endif
#ifndef TX1
#define TX1 17 // Pin used by Modbus Box
#endif
#ifndef RX2
#define RX2 9
#endif
#ifndef TX2
#define TX2 10
#endif
```

------

### ModbusBox  & pyModSlave.

pyModSlave is a free python-based implementation of a ModBus slave application for simulation purposes. You can install the python module or use the precompiled (for Windows only) stand alone GUI (Qt based) utility (unzip and run). pyModSlave also includes a bus monitor for examining all traffic on the bus. You can also download it from pypi [Dowload page](https://sourceforge.net/projects/pymodslave/)

[PymodSlave sample program](https://github.com/luisgcu/SensorModbusMaster/tree/master/utils/pyModbusDemo)

**Hardware connections.**  

![](https://github.com/luisgcu/SensorModbusMaster/blob/master/images/pyModSlave.jpg)

**pyModSlave setup.**

![](https://github.com/luisgcu/SensorModbusMaster/blob/master/images/pyModserial.jpg)

**pyModSlave holding registers set.**

![](https://github.com/luisgcu/SensorModbusMaster/blob/master/images/pymod_hregs.jpg)

**pyModSlave input registers set.**

![](https://github.com/luisgcu/SensorModbusMaster/blob/master/images/pymod_AIreg.jpg)

**Arduino Serial readings.**

![](https://github.com/luisgcu/SensorModbusMaster/blob/master/images/Arduino_serial.jpg)

------

### ModbusBox with several slaves ( Basic program).

The following test is composed by 3 VFD  from different  manufactures and one Energy meter, all the devices are connected in the same RS485 network , the ModbusBox act as Modbus Master it read a array of modbus register from each devices and print the result over the serial port. 

**Code for the example:** [LINK](https://github.com/iotbits-us/SensorModbusMaster/blob/master/utils/Modbus_Box_demo_multiple_slaves/Modbus_Box_demo_multiple_slaves.ino)

**Links to the manual of the slaves in this test:**

**SDM230 Modbus Manual :** [SDM230](http://www.eastrongroup.com/data/uploads/Eastron_SDM230-Modbus_user_manual_V1_4_2015.pdf)

**ABB ACS 310** : [LINK](https://library.e.abb.com/public/0eda39cbd8494c4596d426b81e7884b3/EN_ACS310_UM_D_A4.pdf) 

**Commander SK** : [LINK](https://inverterdrive.com/file/Commander-SK-Advanced-User-Guide)

**Unidrive M200** : [LINK](https://acim.nidec.com/drives/control-techniques/-/media/controltechniques/files/step-by-step-guides/unidrive-m200-control-user-guide.ashx)

















