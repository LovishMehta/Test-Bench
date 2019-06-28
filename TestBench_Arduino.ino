#include <Nextion.h>
#include<Wire.h>
#include <SPI.h>
#include <stdint.h>
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "HV_Pins.h"
#include "Signatures.h"
#include "General_Stuff.h"
#include "bootloader_lilypad328.h"
#include "bootloader_atmega328.h"


#define USE_ATMEGA8 true
#define USE_ATMEGA16U2 true    // Uno USB interface chip
#define USE_ATMEGA32U4 true    // Leonardo
#define USE_ATMEGA168 true
#define USE_ATMEGA1280 true
#define USE_ATMEGA1284 true
#define USE_ATMEGA2560 true
#define USE_ATMEGA256RFR2 false // Pinoccio Scout
//SPI PINS
#define MISO 50
#define MOSI 51
#define SCK 52
#define CSN 53

//motor pins
int ma1=4;
int ma2=5;
int m=6;
int ms=0;

int selction=0,startTest=0,Back=0;
int d4=4; 
int d5=5; 
int d6=6; 
int d7=7; 
int d8=8;
int d11=11;
int upload =0;
char command; 

#define VERSION "1.38"

// make true to use the high-voltage parallel wiring
#define HIGH_VOLTAGE_PARALLEL false
// make true to use the high-voltage serial wiring
#define HIGH_VOLTAGE_SERIAL false
// make true to use ICSP programming
#define ICSP_PROGRAMMING true

#if HIGH_VOLTAGE_PARALLEL && HIGH_VOLTAGE_SERIAL
  #error Cannot use both high-voltage parallel and serial at the same time
#endif

#if (HIGH_VOLTAGE_PARALLEL || HIGH_VOLTAGE_SERIAL) && ICSP_PROGRAMMING
  #error Cannot use ICSP and high-voltage programming at the same time
#endif

#if !(HIGH_VOLTAGE_PARALLEL || HIGH_VOLTAGE_SERIAL || ICSP_PROGRAMMING)
  #error Choose a programming mode: HIGH_VOLTAGE_PARALLEL, HIGH_VOLTAGE_SERIAL or ICSP_PROGRAMMING
#endif

const int ENTER_PROGRAMMING_ATTEMPTS = 50;

const unsigned long BAUD_RATE = 9600;

const byte CLOCKOUT = 9;

#if ICSP_PROGRAMMING

  #ifdef ARDUINO_PINOCCIO
    const byte RESET = SS;  // --> goes to reset on the target board
  #else
    const byte RESET = 10;  // --> goes to reset on the target board
  #endif

  #if ARDUINO < 100
    const byte SCK = 13;    // SPI clock
  #endif

#endif // ICSP_PROGRAMMING


// structure to hold signature and other relevant data about each bootloader
typedef struct {
   byte sig [3];                // chip signature
   unsigned long loaderStart;   // start address of bootloader (bytes)
   const byte * bootloader;     // address of bootloader hex data
   unsigned int loaderLength;   // length of bootloader hex data (bytes)
   byte lowFuse, highFuse, extFuse, lockByte;  // what to set the fuses, lock bits to.
} bootloaderType;


#if USE_ATMEGA168
  #include "bootloader_atmega168.h"
#endif
#if USE_ATMEGA2560
  #include "bootloader_atmega2560_v2.h"
#endif
#if USE_ATMEGA256RFR2
  #include "bootloader_atmega256rfr2_v1.0a.h"
#endif
#if USE_ATMEGA1284
  #include "bootloader_atmega1284.h"
#endif
#if USE_ATMEGA1280
  #include "bootloader_atmega1280.h"
#endif
#if USE_ATMEGA8
  #include "bootloader_atmega8.h"
#endif
#if USE_ATMEGA32U4
  #include "bootloader_atmega32u4.h"
#endif
#if USE_ATMEGA16U2
  #include "bootloader_atmega16u2.h"  // Uno USB interface chip
#endif

const bootloaderType bootloaders [] PROGMEM =
  {
// Only known bootloaders are in this array.
// If we support it at all, it will have a start address.
// If not compiled into this particular version the bootloader address will be zero.

  // ATmega168PA
  { { 0x1E, 0x94, 0x0B },
        0x3E00,               // start address
  #if USE_ATMEGA168
        atmega168_optiboot,   // loader image
        sizeof atmega168_optiboot,
  #else
        0, 0,
  #endif
        0xC6,         // fuse low byte: external full-swing crystal
        0xDD,         // fuse high byte: SPI enable, brown-out detection at 2.7V
        0x04,         // fuse extended byte: boot into bootloader, 512 byte bootloader
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega328P
  { { 0x1E, 0x95, 0x0F },
        0x7E00,               // start address
        atmega328_optiboot,   // loader image
        sizeof atmega328_optiboot,
        0xFF,         // fuse low byte: external clock, max start-up time
        0xDE,         // fuse high byte: SPI enable, boot into bootloader, 512 byte bootloader
        0x05,         // fuse extended byte: brown-out detection at 2.7V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega328
  { { 0x1E, 0x95, 0x14 },
        0x7E00,               // start address
        atmega328_optiboot,   // loader image
        sizeof atmega328_optiboot,
        0xFF,         // fuse low byte: external clock, max start-up time
        0xDE,         // fuse high byte: SPI enable, boot into bootloader, 512 byte bootloader
        0x05,         // fuse extended byte: brown-out detection at 2.7V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega328PB
  { { 0x1E, 0x95, 0x16 },
        0x7E00,               // start address
        atmega328_optiboot,   // loader image
        sizeof atmega328_optiboot,
        0xFF,         // fuse low byte: external clock, max start-up time
        0xDE,         // fuse high byte: SPI enable, boot into bootloader, 512 byte bootloader
        0x05,         // fuse extended byte: brown-out detection at 2.7V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega1280
  { { 0x1E, 0x97, 0x03 },
        0x1FC00,      // start address
  #if USE_ATMEGA1280
        optiboot_atmega1280_hex,
        sizeof optiboot_atmega1280_hex,
  #else
        0, 0,
  #endif
        0xFF,         // fuse low byte: external clock, max start-up time
        0xDE,         // fuse high byte: SPI enable, boot into bootloader, 1280 byte bootloader
        0xF5,         // fuse extended byte: brown-out detection at 2.7V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega2560
  { { 0x1E, 0x98, 0x01 },
        0x3E000,      // start address
  #if USE_ATMEGA2560
        atmega2560_bootloader_hex,// loader image
        sizeof atmega2560_bootloader_hex,
  #else
        0, 0,
  #endif
        0xFF,         // fuse low byte: external clock, max start-up time
        0xD8,         // fuse high byte: SPI enable, boot into bootloader, 8192 byte bootloader
        0xFD,         // fuse extended byte: brown-out detection at 2.7V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega256rfr2
  { { 0x1E, 0xA8, 0x02 },
        0x3E000,      // start address
  #if USE_ATMEGA256RFR2
        atmega256rfr2_bootloader_hex,// loader image
        sizeof atmega256rfr2_bootloader_hex,
  #else
        0, 0,
  #endif
        0xDE,         // fuse low byte: internal transceiver clock, max start-up time
        0xD0,         // fuse high byte: SPI enable, EE save, boot into bootloader, 8192 byte bootloader
        0xFE,         // fuse extended byte: brown-out detection at 1.8V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega16U2
  { { 0x1E, 0x94, 0x89 },
        0x3000,      // start address
  #if USE_ATMEGA16U2
        Arduino_COMBINED_dfu_usbserial_atmega16u2_Uno_Rev3_hex,// loader image
        sizeof Arduino_COMBINED_dfu_usbserial_atmega16u2_Uno_Rev3_hex,
  #else
        0, 0,
  #endif
        0xEF,         // fuse low byte: external clock, m
        0xD9,         // fuse high byte: SPI enable, NOT boot into bootloader, 4096 byte bootloader
        0xF4,         // fuse extended byte: brown-out detection at 2.6V
        0xCF },       // lock bits

  // ATmega32U4
  { { 0x1E, 0x95, 0x87 },
        0x7000,      // start address
  #if USE_ATMEGA32U4
        leonardo_hex,// loader image
        sizeof leonardo_hex,
  #else
        0, 0,
  #endif
        0xFF,         // fuse low byte: external clock, max start-up time
        0xD8,         // fuse high byte: SPI enable, boot into bootloader, 1280 byte bootloader
        0xCB,         // fuse extended byte: brown-out detection at 2.6V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // ATmega1284P family

  // ATmega1284P
  { { 0x1E, 0x97, 0x05 },
        0x1FC00,      // start address
  #if USE_ATMEGA1284
        optiboot_atmega1284p_hex,
        sizeof optiboot_atmega1284p_hex,
  #else
        0, 0,
  #endif
        0xFF,         // fuse low byte: external clock, max start-up time
        0xDE,         // fuse high byte: SPI enable, boot into bootloader, 1024 byte bootloader
        0xFD,         // fuse extended byte: brown-out detection at 2.7V
        0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

  // Atmega8A family

  // ATmega8A
  { { 0x1E, 0x93, 0x07 },
        0x1C00,      // start address
  #if USE_ATMEGA8
        atmega8_hex,
        sizeof atmega8_hex,
  #else
        0, 0,
  #endif
        0xE4,         // fuse low byte: external clock, max start-up time
        0xCA,         // fuse high byte: SPI enable, boot into bootloader, 1024 byte bootloader
        0xFD,         // fuse extended byte: brown-out detection at 2.7V
        0x0F  },      // lock bits: SPM is not allowed to write to the Boot Loader section.

  };  // end of bootloaders

//Defining buttons, pictures or any other objects, which will be used as input from Nextion HMI
//The definition of any object is in the format of Nex<ObjectType> <ObjectName> = Nex<ObjectTyoe>(<Page No.>, <ObjectID>, "<ObjectName>"); 
NexButton mpr = NexButton(4, 1, "mpr");  // Button added
NexButton mpu = NexButton(4, 2, "mpu");  // Button added
NexButton gy = NexButton(4, 3, "gy");
NexButton rfid = NexButton(4, 4, "rfid");
NexButton tcs = NexButton(4, 5, "tcs");
NexButton pir = NexButton(4, 6, "pir");
NexButton ultra = NexButton(4, 7, "ultra");
NexButton joy = NexButton(4, 8, "joy");
NexButton bt = NexButton(6, 1, "bt");
NexButton xbee = NexButton(6, 2, "xbee");
NexButton nrf = NexButton(6, 3, "nrf");
NexButton sdc = NexButton(6, 4, "sdc");
NexButton motor = NexButton(6, 5, "motor");
NexButton motoren = NexButton(6, 6, "motoren");
NexButton servo = NexButton(6, 7, "servo");
NexButton l298n = NexButton(6, 8, "l298n");
NexButton drv = NexButton(6, 9, "drv");
NexButton mega = NexButton(3, 1, "mega");
NexButton uno = NexButton(3, 2, "uno");
NexButton nano = NexButton(3, 3, "nano");
NexButton atmega = NexButton(3, 4, "atmega");
NexPicture start = NexPicture(8, 2, "start");
NexPicture startj = NexPicture(9, 4, "startj");
NexPicture starta = NexPicture(15, 3, "starta");
NexButton mh = NexButton(10, 1, "mh");
NexButton m0 = NexButton(10, 2, "m0");
NexButton ml = NexButton(10, 3, "ml");
NexButton back = NexButton(10, 5, "back");
NexButton Upload = NexButton(15, 10, "Upload"); 
NexButton backa = NexButton(15, 5, "backa");
NexButton backp = NexButton(8, 6, "backp");
NexButton backj = NexButton(9, 7, "backj");

//adding the input objects to the list
// format is  &<objectName>,
NexTouch *nex_listen_list[] = 
{
  &mpr,  // Button added
  &mpu,  // Button added
  &gy,
  &rfid,
  &tcs,
  &pir,
  &ultra,
  &joy,
  &bt,
  &xbee,
  &nrf,
  &sdc,
  &motor,
  &motoren,
  &servo,
  &l298n,
  &drv,
  &mega,
  &uno,
  &nano,
  &atmega,
  &start,
  &startj,
  &starta,
  &mh,
  &m0,
  &ml,
  &back,
  &Upload,
  &backa,
  &backj,
  &backp,
  NULL  // String terminated
};

/* ****************************************************************************************************************************
 *  In the below functions, the selection variable will be assigned to specific value baed on the touch input obtained from HMI. 
 * By using the value of selection variable, the component to be tested is determined.
*/

//This function will be called when mpr is touched HMI
void mprPushCallback(void *ptr)  
{
  selction=1;
} 
//This function will be called when mpr is released HMI
void mprPopCallback(void *ptr) 
{
 // selction=1;
} 

//This function will be called when mpu is touched HMI
void mpuPushCallback(void *ptr)  
{
  selction=2;
}  
//This function will be called when mpu is released HMI
void mpuPopCallback(void *ptr) 
{
 // selction=2;
}

//This function will be called when gy is touched HMI
void gyPushCallback(void *ptr)  
{
  selction=3;
}  
//This function will be called when gy is released HMI
void gyPopCallback(void *ptr) 
{
 // selction=3;
}

//This function will be called when rfid is touched HMI
void rfidPushCallback(void *ptr)  
{
  selction=4;
}  
//This function will be called when rfid is released HMI
void rfidPopCallback(void *ptr) 
{
//  selction=4;
}

//This function will be called when tcs is touched HMI
void tcsPushCallback(void *ptr)  
{
  selction=5;
} 
//This function will be called when tcs is released HMI 
void tcsPopCallback(void *ptr) 
{
  //selction=5;
}

//This function will be called when pir is touched HMI
void pirPushCallback(void *ptr)  
{
  selction=6;
}  
//This function will be called when pir is released HMI
void pirPopCallback(void *ptr) 
{
  //selction=6;
}

//This function will be called when ultrasonic is touched HMI
void ultraPushCallback(void *ptr)  
{
  selction=7;
}  
//This function will be called when ultrasonic is released HMI
void ultraPopCallback(void *ptr) 
{
  //selction=7;
}

//This function will be called when joy is touched HMI
void joyPushCallback(void *ptr)  
{
  selction=8;
}  
//This function will be called when joy is released HMI
void joyPopCallback(void *ptr) 
{
 // selction=8;
}

//This function will be called when bt is touched HMI
void btPushCallback(void *ptr)  
{
  selction=9;
}  
//This function will be called when bt is released HMI
void btPopCallback(void *ptr) 
{
  //selction=9;
}

//This function will be called when xbee is touched HMI
void xbeePushCallback(void *ptr)  
{
  selction=10;
}  
//This function will be called when xbee is released HMI
void xbeePopCallback(void *ptr) 
{
  //selction=10;
}

//This function will be called when nrf is touched HMI
void nrfPushCallback(void *ptr)  
{
  selction=11;
}  
//This function will be called when nrf is released HMI
void nrfPopCallback(void *ptr) 
{
 // selction=11;
}

//This function will be called when sdc is touched HMI
void sdcPushCallback(void *ptr)  
{
  selction=12;
}  
//This function will be called when sdc is released HMI
void sdcPopCallback(void *ptr) 
{
  //selction=12;
}

//This function will be called when motor is touched HMI
void motorPushCallback(void *ptr)  
{
  selction=13;
}  
//This function will be called when motor is released HMI
void motorPopCallback(void *ptr) 
{
  //selction=13;
}

//This function will be called when motoren is touched HMI
void motorenPushCallback(void *ptr)  
{
  selction=14;
}  
//This function will be called when motoren is released HMI
void motorenPopCallback(void *ptr) 
{
  //selction=14;
}

//This function will be called when servo is touched HMI
void servoPushCallback(void *ptr)  
{
  selction=15;
}  
//This function will be called when servo is released HMI
void servoPopCallback(void *ptr) 
{
 // selction=15;
}

//This function will be called when l298n is touched HMI
void l298nPushCallback(void *ptr)  
{
  selction=16;
}  
//This function will be called when l298n is released HMI
void l298nPopCallback(void *ptr) 
{
  //selction=16;
}

//This function will be called when drv is touched HMI
void drvPushCallback(void *ptr)  
{
  selction=17;
}  
//This function will be called when drv is released HMI
void drvPopCallback(void *ptr) 
{
  //selction=17;
}

//This function will be called when mega is touched HMI
void megaPushCallback(void *ptr)  
{
  selction=18;
}  
//This function will be called when mega is released HMI
void megaPopCallback(void *ptr) 
{
  //selction=18;
}

//This function will be called when uno is touched HMI
void unoPushCallback(void *ptr)  
{
  selction=19;
}  
//This function will be called when uno is released HMI
void unoPopCallback(void *ptr) 
{
  //selction=19;
}

//This function will be called when nano is touched HMI
void nanoPushCallback(void *ptr)  
{
  selction=20;
}  
//This function will be called when nano is released HMI
void nanoPopCallback(void *ptr) 
{
//  selction=20;
}

//This function will be called when atmega is touched HMI
void atmegaPushCallback(void *ptr)  
{
  selction=21;
}  
//This function will be called when atmega is released HMI
void atmegaPopCallback(void *ptr) 
{
//  selction=21;
}

/* ****************************************************************************************************************************
 *  In the below functions, the StartTest variable will be assigned to specific value baed on the touch input obtained from HMI. 
 *  When StartTest variable becomes 1, the testing will be started.
*/

//This function will be called when start is touched HMI
void startPushCallback(void *ptr)  
{
  startTest=1;
}  
//This function will be called when start is released HMI
void startPopCallback(void *ptr) 
{
  //startTest=1;
}

//This function will be called when startj is touched HMI
void startjPushCallback(void *ptr)  
{
  startTest=1;
}  
//This function will be called when startj is released HMI
void startjPopCallback(void *ptr) 
{
  //startTest=1;
}

//This function will be called when starta is touched HMI
void startaPushCallback(void *ptr)  
{
  startTest=1;
}  
//This function will be called when starta is released HMI
void startaPopCallback(void *ptr) 
{
  //startTest=1;
}

//This function will be called when back is touched HMI
void backPushCallback(void *ptr)  
{
  Back=1;
}  
//This function will be called when back is released HMI
//This function will be called when back is touched HMI
void backaPushCallback(void *ptr)  
{
  Back=1;
}

//This function will be called when back is touched HMI
void backpPushCallback(void *ptr)  
{
  Back=1;
}

//This function will be called when back is touched HMI
void backjPushCallback(void *ptr)  
{
  Back=1;
}
void backPopCallback(void *ptr) 
{
  //Back=1;
}
void UploadPushCallback(void *ptr)  
{
  upload=1;
}  
//This function will be called when back is released HMI
void UploadPopCallback(void *ptr) 
{
  //upload=1;
}

//*******************************************************************************//
//*******************************************************************************//
//SHORT CIRCUIT DETECTION

//The ShortCircuit() function will trigger the Optocoupler of TestBench, which will cutoff the Gate Voltage of Mosfet. 
//This will turn-off the powersupply to Testing components
void ShortCircuit() 
{
  while(1)
  {
    pinMode(d11,OUTPUT);
    digitalWrite(d11,HIGH); //Optocoupler triggered 
    Serial3.print("page 16"); // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  }
}

//The current() function will check the input current of TestBench using Current Sensor.
//The A2 analog will gives the raw value of current which wil be converted using formula to obtain actual current value in milli ampere(mA).
//If the current() value is greater than 1000mA (1A) then it will consider it as short circuit. 
void current()
{
   
const int currentPin = A2;
int sensitivity = 66;
int adcValue= 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double currentValue = 0;
pinMode(d11,OUTPUT);

for(int i=0;i<10;i++)
{
  adcValue = analogRead(currentPin);
  adcVoltage = (adcValue / 1024.0) * 5000;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity); 
  Serial.print("\t Current = ");
  Serial.println(currentValue,3);
  if(currentValue>1000)
  {
    digitalWrite(d11,HIGH);
    ShortCircuit();
  }
  else
  {
    digitalWrite(d11,LOW);
  }
  delay(10);
}
  return;
}

//*******************************************************************************//
//*******************************************************************************//
//I2C Scanner
//The scanAddress() function is used to read the I2C slave address
uint8_t scanAddress()
  {
    uint8_t error, address, addr=0;
    while(1)
    {
     
  int nDevices;
 
//  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
//      Serial.print("I2C device found at address 0x");
      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
      addr=address;
      nDevices++;
    }
    else if (error==4)
    {
//      Serial.print("Unknown error at address 0x");
      if (address<16){}
//        Serial.print("0");
//      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("No I2C devices found...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
//    Serial3.println("No I2C devices found\n");
  }
  else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Done...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
//    Serial.println("done\n");
    break;
  }
  delay(2000);
    }
    return addr;
  }

//******************************************************************************//
//******************************************************************************//
//MPR121
 //MPR121() function is to perform testing of MPR121 capacitive touch sensor, by reading the Out of Range Register bits. 
void MPR121()
{
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Test in progress...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
    uint8_t MPR121_addr=scanAddress(); //Getting the address of MPR121
    delay(200);
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Test in progress...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
//    Serial.print("Address of MPR121 is 0x");
//    Serial.println(MPR121_addr,HEX);
 while(1)
 {
    //Reading Out of Range Registers
  uint8_t a=0;
  MPR121_addr=0x5A;// I2C address of MPR121
  //Writing 0x8F in the Electrode Configuration Register(0x5E)
  Wire.beginTransmission(MPR121_addr);
  Wire.write(0x5E);
  Wire.write(0x8F); 
  Wire.endTransmission();
  
//  Serial.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
//  Serial.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial.print("Testing");  // This is the text you want to send to that object and atribute mentioned before.
//  Serial.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
//  Serial.write(0xff);
//  Serial.write(0xff);delay(500);

//Reading data from Out of Range Register(0x02)
  Wire.beginTransmission(MPR121_addr);
  Wire.write(0x02); 
  Wire.endTransmission();
  Wire.requestFrom(MPR121_addr,1); 
  while(Wire.available() < 1); 
  a=Wire.read();
//  
//  Serial.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
//  Serial.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial.print("Test");  // This is the text you want to send to that object and atribute mentioned before.
//  Serial.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
//  Serial.write(0xff);
//  Serial.write(0xff);delay(500);

  //printing the Out of rangee register value
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(a);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500);

  //Condition for Working Properly. The value should be 0.
   if(a==0)
  {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("MPR121 is working properly...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500); break;
  }
  else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("MPR121 is not working properly...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(500); break;
  }
      delay(1000); 
 }
 return 0;   
}

//****************************************************************************************//
//****************************************************************************************//
// MPU6050

int Gtest(byte a) // Getting 5bits of Gyro test registers and convert into decimal
  {
    int b0=0,b1=0,b2=0,b3=0,b4=0,c=0;
  b0=bitRead(a,0);
  b1=bitRead(a,1);
  b2=bitRead(a,2);
  b3=bitRead(a,3);
  b4=bitRead(a,4);
  c=(b0*1)+(b1*2)+(b2*4)+(b3*8)+(b4*16);
  return c;
  }
int XAtest(byte a,byte b) // Getting 5bits of X axis Accel test registers and convert into decimal
{
  int b0=0,b1=0,b2=0,b3=0,b4=0,c=0;
  b0=bitRead(b,4);
  b1=bitRead(b,5);
  b2=bitRead(a,5);
  b3=bitRead(a,6);
  b4=bitRead(a,7);
  c=(b0*1)+(b1*2)+(b2*4)+(b3*8)+(b4*16);
  return c;
}
int YAtest(byte a,byte b) // Getting 5bits of Y axis Accel test registers and convert into decimal
{
  int b0=0,b1=0,b2=0,b3=0,b4=0,c=0;
  b0=bitRead(b,2);
  b1=bitRead(b,3);
  b2=bitRead(a,5);
  b3=bitRead(a,6);
  b4=bitRead(a,7);
  c=(b0*1)+(b1*2)+(b2*4)+(b3*8)+(b4*16);
  return c;
}
int ZAtest(byte a,byte b) // Getting 5bits of Z axis Accel test registers and convert into decimal
{
  int b0=0,b1=0,b2=0,b3=0,b4=0,c=0;
  b0=bitRead(b,0);
  b1=bitRead(b,1);
  b2=bitRead(a,5);
  b3=bitRead(a,6);
  b4=bitRead(a,7);
  c=(b0*1)+(b1*2)+(b2*4)+(b3*8)+(b4*16);
  return c;
}

double GFT(int a)  //Calculation of Factory Trim (FT) value for Gyro
{
  if(a==0)
  {
    return 0;
  }
  else
  {
  double c;
  a=a-1;
  c=3275*( pow(1.046,a) );
  return c;
  }  
}

double AFT(int a) //Calculation of Factory Trim (FT) value for Accel
{
  if(a==0)
  {
    return 0;
  }
  else
  {
    double c;
    float b=(a-1)/30.0;
    c=1392.64*( pow(2.7058823529,b));
    return c;
  }
}

double CFT(float a,int b) //Calculation of Change from Factory Trim (FT) value to Self-Test Response (in %)
{
  if(a==0)
  {
    return 0;
  }
  else
  {
  float c=(b-a)/a;
  return c;
  }
}
void MPU6050()
 { 
//  Serial.println("MPU6050 is Selected...");
  uint8_t MPU_addr;// I2C address of the MPU-6050
  uint8_t x,y,z,xyz,AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

  MPU_addr=scanAddress();
  MPU_addr=0x68; //Getting the I2C adress of MPU6050
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(MPU_addr);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);
  delay(200);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Test in progress...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);
//  Serial.print("Address of MPU6050 is 0x");
//  Serial.println(MPU_addr,HEX);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  while(1)
  {
    //Self-Test Enable for Gyro
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); 
  Wire.write(0xE7);
  Wire.endTransmission();
  
  //Self-Test Enable for Accel
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); 
  Wire.write(0xF7);
  Wire.endTransmission();

  //Reading the Gyro and Accel output when Self-Test is enabled
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
 
  //Variables for Self-Test Responce (STR)
  int XASTR=AcX;
  int YASTR=AcY;
  int ZASTR=AcZ;
  int XGSTR=GyX;
  int YGSTR=GyY;
  int ZGSTR=GyZ;
  delay(333);

  //Self-Test Disable for Gyro
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); 
  Wire.write(0x07);
  Wire.endTransmission();
  
  //Self-Test Disable for Accel
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); 
  Wire.write(0x17);
  Wire.endTransmission();

  //Reading the Gyro and Accel output when Self-Test is disabled
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
  
  //Calculation of Self-Test Response(STR)
  XASTR=XASTR-AcX;
  YASTR=YASTR-AcY;
  ZASTR=ZASTR-AcZ;
  XGSTR=XGSTR-GyX;
  YGSTR=YGSTR-GyY;
  ZGSTR=ZGSTR-GyZ;
  delay(333);  

  //Reading Self-Test Registers
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x0D);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,4,true);  
  x=Wire.read()<<8|Wire.read();
  y=Wire.read()<<8|Wire.read();
  z=Wire.read()<<8|Wire.read();
  xyz=Wire.read()<<8|Wire.read();
//  Serial.print("X = "); Serial.print(x,BIN);
//  Serial.print(" | y = "); Serial.print(y,BIN);
//  Serial.print(" | z = "); Serial.print(z,BIN);
//  Serial.print(" | xyz = "); Serial.println(xyz,BIN);  

  // Getting 5bits of Gyro test registers and convert into decimal  
  int Xgt=Gtest(x);
  int Ygt=Gtest(y);
  int Zgt=Gtest(z);

  // Getting 5bits of Accel test registers and convert into decimal
  int Xat=XAtest(x,xyz);
  int Yat=YAtest(y,xyz);
  int Zat=ZAtest(z,xyz);

  //Getting Factory Trim (FT) values for Gyro
  double XGFT=GFT(Xgt);
  double YGFT=-1.0*GFT(Ygt);
  double ZGFT=GFT(Zgt);  

  //Getting CFT% for Gyro 
  float XGCFT=CFT(XGFT,XGSTR);
  float YGCFT=CFT(YGFT,YGSTR);
  float ZGCFT=CFT(ZGFT,YGSTR);
//  Serial.print("Gyro %CFT: \t");
//  Serial.print(XGCFT);Serial.print("%\t ");
//  Serial.print(YGCFT);Serial.print("%\t ");
//  Serial.print(ZGCFT);Serial.println("%\t ");

  //Getting Factory Trim (FT) values for Accel
  double XAFT=AFT(Xat);
  double YAFT=AFT(Yat);
  double ZAFT=AFT(Zat);

  //Getting CFT% for Accel
  float XACFT=CFT(XAFT,XASTR);
  float YACFT=CFT(YAFT,YASTR);
  float ZACFT=CFT(ZAFT,ZASTR);
//  Serial.print("Acc %CFT: \t");
//  Serial.print(XACFT);Serial.print("%\t ");
//  Serial.print(YACFT);Serial.print("%\t ");
//  Serial.print(ZACFT);Serial.println("%\t ");   

  //Checking whether all the CFT values are within the range of -14 to 14 percentage
  if(XGCFT<=14 && XGCFT>=-14 && YGCFT<=14 && YGCFT>=-14 && ZGCFT<=14 && ZGCFT>=-14 && XACFT<=14 && XACFT>=-14 && YACFT<=14 && YACFT>=-14 && ZACFT<=14 && ZACFT>=-14)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("MPU6050 is Working correctly...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500);break;
  }
  else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("MPU6050 is not working correctly...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500);break;
  }
//  Serial.println("\t");
  delay(1000);
  }  
  return;
 }


//********************************************************************************************//
//********************************************************************************************//
//RFID
//Defining all the Internal Registers

  #define MFRC522_SPICLOCK SPI_CLOCK_DIV4    

    byte CommandReg        = 0x01 << 1;  // starts and stops command execution
    byte ComIEnReg       = 0x02 << 1;  // enable and disable interrupt request control bits
    byte DivIEnReg       = 0x03 << 1;  // enable and disable interrupt request control bits
    byte ComIrqReg       = 0x04 << 1;  // interrupt request bits
    byte DivIrqReg       = 0x05 << 1;  // interrupt request bits
    byte ErrorReg        = 0x06 << 1;  // error bits showing the error status of the last command executed 
    byte Status1Reg        = 0x07 << 1;  // communication status bits
    byte Status2Reg        = 0x08 << 1;  // receiver and transmitter status bits
    byte FIFODataReg       = 0x09 << 1;  // input and output of 64 byte FIFO buffer
    byte FIFOLevelReg      = 0x0A << 1;  // number of bytes stored in the FIFO buffer
    byte WaterLevelReg     = 0x0B << 1;  // level for FIFO underflow and overflow warning
    byte ControlReg        = 0x0C << 1;  // miscellaneous control registers
    byte BitFramingReg     = 0x0D << 1;  // adjustments for bit-oriented frames
    byte CollReg         = 0x0E << 1;  // bit position of the first bit-collision detected on the RF interface
    //              0x0F      // reserved for future use
    
    // Page 1: Command
    //              0x10      // reserved for future use
    byte ModeReg         = 0x11 << 1;  // defines general modes for transmitting and receiving 
    byte TxModeReg       = 0x12 << 1;  // defines transmission data rate and framing
    byte RxModeReg       = 0x13 << 1;  // defines reception data rate and framing
    byte TxControlReg      = 0x14 << 1;  // controls the logical behavior of the antenna driver pins TX1 and TX2
    byte TxASKReg        = 0x15 << 1;  // controls the setting of the transmission modulation
    byte TxSelReg        = 0x16 << 1;  // selects the internal sources for the antenna driver
    byte RxSelReg        = 0x17 << 1;  // selects internal receiver settings
    byte RxThresholdReg      = 0x18 << 1;  // selects thresholds for the bit decoder
    byte DemodReg        = 0x19 << 1;  // defines demodulator settings
    //              0x1A      // reserved for future use
    //              0x1B      // reserved for future use
    byte MfTxReg         = 0x1C << 1;  // controls some MIFARE communication transmit parameters
    byte MfRxReg         = 0x1D << 1;  // controls some MIFARE communication receive parameters
    //              0x1E      // reserved for future use
    byte SerialSpeedReg      = 0x1F << 1;  // selects the speed of the serial UART interface
    
    // Page 2: Configuration
    //              0x20      // reserved for future use
    byte CRCResultRegH     = 0x21 << 1;  // shows the MSB and LSB values of the CRC calculation
    byte CRCResultRegL     = 0x22 << 1;
    //              0x23      // reserved for future use
    byte ModWidthReg       = 0x24 << 1;  // controls the ModWidth setting?
    //              0x25      // reserved for future use
    byte RFCfgReg        = 0x26 << 1;  // configures the receiver gain
    byte GsNReg          = 0x27 << 1;  // selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
    byte CWGsPReg        = 0x28 << 1;  // defines the conductance of the p-driver output during periods of no modulation
    byte ModGsPReg       = 0x29 << 1;  // defines the conductance of the p-driver output during periods of modulation
    byte TModeReg        = 0x2A << 1;  // defines settings for the internal timer
    byte TPrescalerReg     = 0x2B << 1;  // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
    byte TReloadRegH       = 0x2C << 1;  // defines the 16-bit timer reload value
    byte TReloadRegL       = 0x2D << 1;
    byte TCounterValueRegH   = 0x2E << 1;  // shows the 16-bit timer value
    byte TCounterValueRegL   = 0x2F << 1;
    
    // Page 3: Test Registers
    //              0x30      // reserved for future use
    byte TestSel1Reg       = 0x31 << 1;  // general test signal configuration
    byte TestSel2Reg       = 0x32 << 1;  // general test signal configuration
    byte TestPinEnReg      = 0x33 << 1;  // enables pin output driver on pins D1 to D7
    byte TestPinValueReg     = 0x34 << 1;  // defines the values for D1 to D7 when it is used as an I/O bus
    byte TestBusReg        = 0x35 << 1;  // shows the status of the internal test bus
    byte AutoTestReg       = 0x36 << 1;  // controls the digital self-test
    byte VersionReg        = 0x37 << 1;  // shows the software version
    byte AnalogTestReg     = 0x38 << 1;  // controls the pins AUX1 and AUX2
    byte TestDAC1Reg       = 0x39 << 1;  // defines the test value for TestDAC1
    byte TestDAC2Reg       = 0x3A << 1;  // defines the test value for TestDAC2
    byte TestADCReg        = 0x3B << 1;   // shows the value of ADC I and Q channels
    //              0x3C      // reserved for production tests
    //              0x3D      // reserved for production tests
    //              0x3E      // reserved for production tests
    //              0x3F      // reserved for production tests

byte PCD_SoftReset     = 0x0F;    // resets the MFRC522
int _chipSelectPin=53;
int _resetPowerDownPin=49;
static constexpr uint8_t UNUSED_PIN = UINT8_MAX;

byte PCD_ReadRegister(  byte reg  ///< The register to read from. One of the PCD_Register enums.
                ) {
  byte value;
  SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);      // Select slave
  SPI.transfer(0x80 | reg);         // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  value = SPI.transfer(0);          // Read the value back. Send 0 to stop reading.
  digitalWrite(_chipSelectPin, HIGH);     // Release slave again
  SPI.endTransaction(); // Stop using the SPI bus
  return value;
}

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init() {
  bool hardReset = false;

  // Set the chipSelectPin as digital output, do not select the slave yet
  pinMode(_chipSelectPin, OUTPUT);
  digitalWrite(_chipSelectPin, HIGH);
  
  // If a valid pin number has been set, pull device out of power down / reset state.
  if (_resetPowerDownPin != UNUSED_PIN) {
    // First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
    pinMode(_resetPowerDownPin, INPUT);
  
    if (digitalRead(_resetPowerDownPin) == LOW) { // The MFRC522 chip is in power down mode.
      pinMode(_resetPowerDownPin, OUTPUT);    // Now set the resetPowerDownPin as digital output.
      digitalWrite(_resetPowerDownPin, LOW);    // Make sure we have a clean LOW state.
      delayMicroseconds(2);       // 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μsl
      digitalWrite(_resetPowerDownPin, HIGH);   // Exit power down mode. This triggers a hard reset.
      // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
      delay(50);
      hardReset = true;
    }
  }

  if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
    PCD_Reset();
  }
  
  // Reset baud rates
  PCD_WriteRegister(TxModeReg, 0x00);
  PCD_WriteRegister(RxModeReg, 0x00);
  // Reset ModWidthReg
  PCD_WriteRegister(ModWidthReg, 0x26);

  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  PCD_WriteRegister(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  PCD_WriteRegister(TPrescalerReg, 0xA9);   // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
  PCD_WriteRegister(TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  PCD_WriteRegister(TReloadRegL, 0xE8);
  
  PCD_WriteRegister(TxASKReg, 0x40);    // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  PCD_WriteRegister(ModeReg, 0x3D);   // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn();            // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(  byte reg, ///< The register to write to. One of the PCD_Register enums.
                  byte value      ///< The value to write.
                ) {
  SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);    // Select slave
  SPI.transfer(reg);            // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  SPI.transfer(value);
  digitalWrite(_chipSelectPin, HIGH);   // Release slave again
  SPI.endTransaction(); // Stop using the SPI bus
}

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset() {
  PCD_WriteRegister(CommandReg, PCD_SoftReset); // Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
  uint8_t count = 0;
  do {
    // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
    delay(50);
  } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
}

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn() {
  byte value = PCD_ReadRegister(TxControlReg);
  if ((value & 0x03) != 0x03) {
    PCD_WriteRegister(TxControlReg, value | 0x03);
  }
}
void RFID()
{
  pinMode(_chipSelectPin,OUTPUT);
  pinMode(_resetPowerDownPin,OUTPUT);
  
  while(1)
  {
    PCD_Init(); // Initialization of RID
    
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Test in progress...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);
  
    byte a=PCD_ReadRegister(VersionReg); //Reads the Version of RFID from Version Register
    
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(a);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);
//  Serial.println(a,HEX);

  PCD_WriteRegister(AutoTestReg,0x09); //Starting the AutoTest by writing 0x09 to AutoTest Register
  byte b=PCD_ReadRegister(CRCResultRegH);//Reading the value of Higher order Bits output from CRCResultRegH 
  
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(b);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);
//  Serial.println(b);

  byte c=PCD_ReadRegister(CRCResultRegL);//Reading the value of Lower order Bits output from CRCResultRegL
  
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(c);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);
//  Serial.println(c);
  PCD_WriteRegister(AutoTestReg,0x00);//Ending the Auto Test by writing 0x00 to AutoTest Register

  //Condition for Proper Working. If all the output bits are 1, (i.e.,) 255 for higher byte and 255 for lower byte, then RFID is working Properly
 if(b==255 && c==255)
 {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("RFID-RC522 is working...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);break;
 }
 else
 {
Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("RFID-RC522 is not working...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);break;
 }

   Serial.println("\t");
  delay(1000);
  }
  return;
}
//********************************************************************************************//
//********************************************************************************************//
//BMP180

void BMP180()
{
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Testing..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  while(1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Test in progress...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);

  //Reading the Chip ID of BMP180 from the CHip-ID register 0xD0. 0x77 is the I2C addreess of BMP180
  Wire.beginTransmission(0x77);
  Wire.write(0xD0); 
  Wire.endTransmission();
  Wire.requestFrom(0x77,1); 
  while(Wire.available() < 1); 
  uint8_t a=Wire.read();
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Chip ID:");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(a);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);

  //condition for proper communication. If the Chip ID is 85(in Decimal) then BMP is Properly Communicating
  if(a==85)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("BMP180 is communicating correctly...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);break;
  }
  else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("BMP180 is not communicating correctly...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);break;
  }
  }
  return; 
}

//********************************************************************************************//
//********************************************************************************************//
//Joystick

//Converting of X-axis output of joystick into percentage
float xout()
{
  int xin=0;int xo=0;
  xo=analogRead(xin);
  float xf=xo/1023.00;
  xo=xf*100;
  return xo;
}

//Converting of Y-axis output of joystick into percentage
float yout()
{
  int yin=1;int yo=0;
  yo=analogRead(yin);
  float yf=yo/1023.00;
  yo=100-(yf*100);
  return yo;
}

//Joystick Function is to test Joystick module
void joystick()
{
  int xo,yo,jcount=0,i=0;
  while(1)
  {
  Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Joystick Testing Started...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  
  while(1)
  {
    //Giving instruction as Move Up to user
     Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Move UP...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);

  //Getting the joystick output as percentage
  xo=xout();
  yo=yout();

  //Sending the percentage values of X and Y axis to Nextion HMI sliders xj and yj
  Serial3.print("xj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(xo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("yj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(yo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  
  if(yo==100) //This condition is to checks whether the joystick moved up or not 
  {
    //increment jcount when it satisfy the condition
    jcount++;delay(500);
    break;
  }
  }

  while(1)
  {
    //Giving instruction as Move Down to user
     Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Move DOWN...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);

    //Getting the joystick output as percentage
  xo=xout();
  yo=yout();

    //Sending the percentage values of X and Y axis to Nextion HMI sliders xj and yj
  Serial3.print("xj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(xo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("yj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(yo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  if(yo==0) //This condition is to checks whether the joystick moved down or not 
  {
    //increment jcount when it satisfy the condition
    jcount++;delay(500);
    break;
  }
  }

while(1)
  {
    //Giving instruction as Move Right to user
     Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Move RIGHT...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);

    //Getting the joystick output as percentage
  xo=xout();
  yo=yout();
  Serial3.print("xj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(xo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("yj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(yo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  if(xo==100)  //This condition is to checks whether the joystick moved Right or not
  {
    //increment jcount when it satisfy the condition
    jcount++;delay(500);
    break;
  }
  }

  while(1)
  {
    //Giving instruction as Move Left to user
     Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Move LEFT...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);

//Getting the joystick output as percentage
  xo=xout();
  yo=yout();
  Serial3.print("xj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(xo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("yj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(yo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  if(xo==0)  //This condition is to checks whether the joystick moved Right or not
  {
    //increment jcount when it satisfy the condition
    jcount++;delay(500);
    break;
  }
  }
  while(1)
  {
    Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Stop...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
    xo=xout();
  yo=yout();
  Serial3.print("xj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(xo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("yj.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print(yo);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);  
  if(xo>=47 && xo<53 && yo>48 && yo<52)
  {
    delay(500);break;  
  }
  }

  // Checking whether all the 4 movements are properly occured or not. If jcount is 4 then its working properly
  if(jcount==4)
  {
    Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Joystick Success...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);break;
  }
  else
  {
    Serial3.print("t0j.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Joystick Failure...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(5000);break;
  }
  
  }
  return;
}
//********************************************************************************************//
//********************************************************************************************//
//TCS3200

//red() function is used to read Red frequency of RGB color value
int red()
{
 pinMode(d6,OUTPUT);//s2
  pinMode(d7,OUTPUT);//s3
  pinMode(d8,INPUT);//out 
  int f,i=0;
  while(i<5)
  {
digitalWrite(d6,LOW);
digitalWrite(d7,LOW);//setting for RED color sensor
delay(100);
f = pulseIn(d8, HIGH);//reading frequency
i++;
  }
return f;
}

//greeeen() function is used to read Green frequency of RGB color value
int green()
{
  pinMode(d6,OUTPUT);//s2
  pinMode(d7,OUTPUT);//s3
  pinMode(d8,INPUT);//out
  int f,i=0;
  while(i<5)
  {
digitalWrite(d6,HIGH);
digitalWrite(d7,HIGH);// setting for GREEN color sensor
delay(100);
f = pulseIn(d8, HIGH);
i++;
  }
return f;
}

//blue() function is used to read Blue frequency of RGB color value
int blue()
{
  pinMode(d6,OUTPUT);//s2
  pinMode(d7,OUTPUT);//s3
  pinMode(d8,INPUT);//out
int f,i=0;
  while(i<5)
  {
digitalWrite(d6,LOW);
digitalWrite(d7,HIGH);// setting for BLUE color sensor
delay(100);
f = pulseIn(d8, HIGH);
i++;
  }
return f;
}

//Test for matching Red Color
int testRed()
{
  int i=0;
  while(i<10)
  {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Show a Red paper..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500);i++;
  }
  i=0;
  while(1)
  {
    // if the Red value is lesser than Blue and Green, then it is Red and it will return 1. Else it will return 0.
  if( red() < green() && red() < blue() )
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Red is matched..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);return 1;
  }
  else
  {
    if(i>20)
    {
      return 0;
    }
  }
  i++;
  delay(100);
  }
}

//Test for matching Green Color
int testGreen()
{
  int i=0;
  while(i<10)
  {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Show a Green paper..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500);i++;
  }
  i=0;
  while(1)
  {
    // if the Green value is lesser than Blue and Red, then it is Green and it will return 1. Else it will return 0.
  if( green() < red() && green() < blue() )
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Green is matched..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);return 1;
  }
  else
  {
    if(i>20)
    {
      return 0;
    }
  }
  i++;
  delay(100);
  }
}

//Test for matching Blue color
int testBlue()
{
  int i=0;
  while(i<10)
  {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Show a Blue paper..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(500);i++;
  }
  i=0;
  while(1)
  {
    // if the Blue value is lesser than Red and Green, then it is Blue and it will return 1. Else it will return 0.
  if( blue() < red() && blue() < green() )
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Blue is matched..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);return 1;
  }
  else
  {
    if(i>20)
    {
      return 0;
    }
  }
  i++;
  delay(100);
  }
}

//TCS3200() function is to Test the TCS3200 Color sensor
void TCS3200()
{
  pinMode(d4,OUTPUT);//s0
  pinMode(d5,OUTPUT);//s1
  pinMode(d6,OUTPUT);//s2
  pinMode(d7,OUTPUT);//s3
  pinMode(d8,INPUT);//out
  digitalWrite(d4,HIGH);//s0
  digitalWrite(d5,LOW);//s1
  int ccount =0;
  //Count value will be added with 1, if the conditions are satisfied in each test. 
  ccount+=testRed(); 
  ccount+=testGreen();
  ccount+=testBlue();
  //If all the Three Colors are matched then the ccount will be 3 and we can say that, it works properly
  if(ccount==3)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Color sensor success..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  }
  else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Color sensor failed..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  }
  
}

//********************************************************************************************//
//********************************************************************************************//
//Ultrasonic

void ultrasonic()
{
  long duration;
  int distance;int i=0;
  pinMode(d4, OUTPUT); // Sets the trigPin as an Output
   pinMode(d5, INPUT); // Sets the echoPin as an Input
   Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Ultrasonic sensor testing..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  while(1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Ultrasonic sensor testing..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);

    
    // Clears the trigPin
    digitalWrite(d4, LOW);
    delayMicroseconds(2);
    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(d4, HIGH);
    delayMicroseconds(10);
    digitalWrite(d4, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(d5, HIGH);
    //Serial3.print(duration);
    
    // Calculating the distance
    distance= duration*0.034/2;
    // Prints the distance on the Serial Monitor
    //Serial3.print("\tDistance: ");
    //Serial3.println(distance);
    delay(500);i++;
    //The fixed distance is 5. so it should be in the range of 4 to 5 for proper working
    if(distance>=4 && distance<=6)
    {
      Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Ultrasonic sensor Success..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);break;
     }
     else
     {
      if(i>20)
      {
        Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Ultrasonic sensor Failure..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);break;
        
      }
     }
  }
  
}

//********************************************************************************************//
//********************************************************************************************//
//PIR

int motion()
{
  int pirState = LOW;  // we start, assuming no motion detected
    int val = 0; int i=0; int pcount=0;
    pinMode(d4, INPUT);     // declare sensor as input
   while(i<=100)
   {
    val = digitalRead(d4);  // read input value
      if (val == HIGH) {            // check if the input is HIGH
              if (pirState == LOW) {
          // we have just turned on
         // Serial3.println("Motion detected!");
          // We only want to print on the output change, not state
          pcount++;
          pirState = HIGH;
        }
      } else {
        if (pirState == HIGH){
          // we have just turned of
         // Serial3.println("Motion ended!");
          // We only want to print on the output change, not state
          pcount++;
          pirState = LOW;
        }
      }
    delay(100);i++;
    }
    return pcount;
}
void PIR()
{
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("PIR Testing..");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  int pcount=0;

  //Instructing user to do any motionn in-front of PIR
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Do Motion infront of PIR...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  //if Motion is detected increment pcount value
  if( motion() >=2 )
  {
    pcount++;
  }

  //Instructing user not to do any motionn in-front of PIR
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Don't do any motion...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  // if motion is not detected , then increment pcount 
  if(motion()==0)
  {
    pcount++;
  }
  
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Getting result...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(3000);

  // if the pcount is 2 then we can say it passed two motion test and working properly
  if(pcount==2)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("PIR success...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  }
  else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("PIR Fail...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);delay(2000);
  }
  
}

//********************************************************************************************//
//********************************************************************************************//
//Xbee
void XBEE()
{
  int error=0;
  Serial.begin(9600);
  Serial1.begin(9600);
  int i=0;
  int ferr=0;char c='0';
//  Serial.println("Xbee Start");

  //enter AT mode
  Serial.write("+++");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
//    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
//  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial3.print((char)Serial.read());  // This is the text you want to send to that object and atribute mentioned before.
//  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
//  Serial3.write(0xff);
//  Serial3.write(0xff);
    } i++;
  }  
  i=0;
delay(1000);
Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("ATMode");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  
//reset
Serial.write("ATRE\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
//    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
//  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial3.print((char)Serial.read());  // This is the text you want to send to that object and atribute mentioned before.
//  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
//  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
//  Serial3.write(0xff);
//  Serial3.write(0xff);
//  Serial.print((char)Serial.read());
    }i++;
  }
  i=0;
  delay(1000);
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Reset");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
//  Serial.println("reset");

//set ID as 3200
  Serial.write("ATID3200\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
    ferr=1;error=1;
  }
  }
  i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }

//set CH as C
  Serial.write("ATCHC\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
    ferr=1;error=1;
  }
  }i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }

  //Set DH as 0
Serial.write("ATDH0\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  //Serial.print((char)Serial.read());
    }i++;
  }
  i=0;
  delay(1000);

//set DL as 0
  Serial.write("ATDL0\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
//    Serial.println("..");
    ferr=1;error=1;
  }
  }i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }

//set MY as 0
  Serial.write("ATMY0\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
    ferr=1;error=1;
  }
  }i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }
  
  //set NI as XBEE-2
  Serial.write("ATNIXBEE-2\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
    ferr=1;error=1;
  }
  }i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }
  
//set AP as 0
  Serial.write("ATAP0\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
    ferr=1;error=1;
  }
  }i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }

  //come out of AT command mode
  Serial.write("ATCN\r\n");
  while(!Serial.available());
  while(i<1000)
  {
    if(Serial.available())
    {
  c=Serial.read();
    }i++;
  if(c=='E')
  {
    ferr=1;error=1;
  }
  }i=0;
  delay(1000);
  if(ferr==1)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Incompatible Firmware or Xbee Configuration... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Please upload newest firmware in 802.15.4 TH function set... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(5000);
    return;
  }
  delay(2000);

 Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Xbee Configuration  Done... ");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(2000);
  
 if(error==0)
  {
    int xcount=0; 
    delay(1000);
 Serial.write("5"); //Sending 5 from test xbee to in-built xbee

 //In-built Xbee receiving data
 while(!Serial1.available());
  while(Serial1.available())
  {
  c=Serial1.read();
  }
    delay(1000);
  if(c!='O')
  {
    //if the data is matched increment the xcount
  if(c=='5'){
    xcount++;
  }
  }
  delay(1000);
  
 Serial1.write("6");//Sending 6 from in-built xbee to test xbee

 //Test Xbee receiving data
 while(!Serial.available());
  while(Serial.available())
  {
  c=Serial.read();
  }
    delay(1000);
  if(c!='O')
  {
    //if the data is matched then increment xcount
  if(c=='6'){
    xcount++;
  }
  }
 delay(1000);

//if the xbee properly transmitted and received the data, then the xcount will be 2.
//otherwise xount will not be 2 and we cann say it is not communicating properly.
 if(xcount==2)
 {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Communication Yes...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
 }
 else
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("Communication No...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
  }
  }    
  
}
//********************************************************************************************//
//********************************************************************************************//
//NRF
//NRF Reciever PinsSPI are SPI pins
#define CE 49

//Transmitter pins using bitbang
#define MISO_pin 36
#define MOSI_pin 35
#define SCK_pin 34
#define CSN_pin 33
#define CE_pin 32

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

// commands
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF
#define W 1
#define R 0
//Registers
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD      0x1C
#define FEATURE     0x1D
 //****************************************************************************************************/

// Data being Sent from the Transmitter Nrf
uint8_t Buff[5] = {'S','O','N','R','F'};

// Initialization of receiver SPI
void SPI_Init(void)
{    

//    GPIO_PinDirection(SCK_PIN,OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */ 
//    GPIO_PinDirection(MOSI_PIN,OUTPUT);
//    GPIO_PinDirection(MISO_PIN,INPUT);
//    GPIO_PinDirection(SSEL_PIN,OUTPUT);
    pinMode(CSN,OUTPUT);
pinMode(MISO,INPUT);
pinMode(MOSI, OUTPUT);
pinMode(SCK,OUTPUT);
pinMode(SCK,OUTPUT);
pinMode(CE,OUTPUT);
    //SPI_DisableChipSelect();                  /* Disable the Slave Select */
digitalWrite(CSN,HIGH);
digitalWrite(CE,LOW);
    SPCR = 0x53; //setup SPI: Master mode, MSB first, SCK phase low, SCK idle low
    SPSR = 0x00;
 
}

//Initialization of Transmitter NRf
void Soft_spi()
{
pinMode(CSN_pin,OUTPUT);
pinMode(MISO_pin,INPUT);
pinMode(MOSI_pin, OUTPUT);
pinMode(SCK_pin,OUTPUT);
                 /* Disable the Slave Select */
digitalWrite(CSN_pin,HIGH);
digitalWrite(CE_pin,LOW);// put your setup code here, to run once:
}

/***************************************************************************************************
                          uint8_t SPI_Write (uint8_t spiData_u8)
 ****************************************************************************************************
 * I/P Arguments : 
                 uint8_t : Byte of data to be send on SPI.

 * Return value  : 
                 uint8_t : Returns back the data send on SPI, this is used in case of SD card.

 * description :
                 This function is used to send a byte of data through SPI.              
 ****************************************************************************************************/
void SPI_Write (uint8_t spiData_u8)
{

    SPDR = spiData_u8;

    // Wait for transmission complete
    while(!(SPSR & (1<<SPIF)));


    
}

uint8_t SPI_Read(void)
{
    uint8_t spiData_u8;

    SPDR = 0xff;
    while(!(SPSR & (1<<SPIF)));
    spiData_u8 = SPDR;

    return spiData_u8;
}

//Receiver GetReg
uint8_t GetReg(uint8_t reg)
{
  delayMicroseconds(10); //minimum delay between 2 commands in nrf should be 10 us
  
  digitalWrite(CSN,LOW); //the nRF starts listening for commands when the CSN-pin goes low.
  
  delayMicroseconds(10);//after a delay of 10us it accepts a single byte through SPI, which tells the nRF which bytes you
             //want to read/write to, and if you want to read or write to it.
  SPI_Write(reg);
  delayMicroseconds(10);//a 10us delay later it then accepts further bytes which is either written to the above 
                        //specified register, or a number of dummy bytes (that tells the nRF how many bytes you want to read out)
  reg = SPI_Read();
  delayMicroseconds(10);
  digitalWrite(CSN,HIGH); //when finished close the connection by setting CSN to high again.
  return reg;
}

//Transmitter NRF
uint8_t GetReg_bitBang(uint8_t reg)
{
  digitalWrite(CSN_pin, LOW); // put your main code here, to run repeatedly:
byte returnValue = bitBang(reg);
byte returnValue1 = bitBang(0xff);
digitalWrite(CSN_pin, HIGH);
return returnValue1;
}

// Receiver Write function
void *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)
{
  //ReadWrite"("W" or "R"), "reg"(the register), "*val"(an array with the package), "antval"(number of integer in package")
  if(ReadWrite == W )
  {
    reg = 0x20|reg; // Add the write bit to the reg
  }
  static uint8_t ret[32];
  delayMicroseconds(10);
  digitalWrite(CSN,LOW);
  delayMicroseconds(10);
  SPI_Write(reg);
  delayMicroseconds(10);

  int i;
  for(i=0; i < antVal; i++)
  {
    if(ReadWrite == R && reg != W_TX_PAYLOAD)
    {
      ret[i]=SPI_Read();
      delayMicroseconds(10); 
    }
    else
    {
      SPI_Write(val[i]);
      delayMicroseconds(10);
    }
  }
  digitalWrite(CSN,HIGH);
  return ret;
}

//Transmitter Wrtie function
uint8_t *WriteToNrf_bitBang(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t ant)
{
  if(ReadWrite == W )
  {
    reg = 0x20|reg; // Add the write bit to the reg
  }
  static uint8_t ret[32];
  delayMicroseconds(10);
  digitalWrite(CSN_pin,LOW);  // CSN_pin Low , so that NRF statrts to listen for SPI command
  delayMicroseconds(10);
  bitBang(reg);

  int i;
  for(i=0; i < ant; i++)
  {
    if(ReadWrite == R && reg != W_TX_PAYLOAD)
    {
      ret[i] = bitBang(0xff);
      delayMicroseconds(10); 
    }
    else
    {
      bitBang(val[i]);
      delayMicroseconds(10);
    }
  }
  digitalWrite(CSN_pin,HIGH);  //CSN_pin_pin High - nrf does not acCE_pinpt SPI commands
  return ret;
}

//Receiver Reset
 void resett()
  {
    delayMicroseconds(10);
    digitalWrite(CSN,LOW);
    delayMicroseconds(10);
    SPI_Write(0x20|0x07);
    delayMicroseconds(10);
    SPI_Write(0x70);
    delayMicroseconds(10);
    digitalWrite(CSN,HIGH);
  }

// Transmitter Reset
void resett_bitBang()
  {
    delayMicroseconds(10);
    digitalWrite(CSN_pin,LOW);   //nrf starts listening for SPI commands when CSN_pin goes LOW
    delayMicroseconds(10);
    bitBang(0x20|0x07);  //Write to the particular register that's why adding 0x20 to reg address
    delayMicroseconds(10);
    bitBang(0x70);       // Reset all IRQ in STATUS regiter
    delayMicroseconds(10);
    digitalWrite(CSN_pin,HIGH);// CSN_pin high . the nrf stops listening for SPI commands
  }

//Receiver Nrf initialization  
void nrf_init()
{
  delay(100);
  uint8_t val[5];

  resett();
  Serial.println("Finding NRF Receiver Module");
  uint8_t test = GetReg(STATUS);
  Serial.print("Status :");
  Serial.println(test,HEX);
  if(test == 0x0E)
  {
    Serial.println("\t Receiver Module found.....");
    Serial.println("\tConfiguring the Receiver module....\n");
  }
   else {
    Serial.println(" Receiver Module Not Found...");
    Serial.println("Exiting..");
    return;
  }

  
//  Enable Auto Ack
  val[0] =0x01;
  WriteToNrf(W,EN_AA, val,1); //Enable auto acknowledgement data pipe 0
  Serial.println(GetReg(EN_AA), HEX);

 //  Retries and Delay
  val[0]=0x2F;
  WriteToNrf(W,SETUP_RETR, val,1);//Setup automatic transmission, 0b00101111 so '0010'->after end of transmission wait for 750us and then retransmit and '1111'->Retransmit 15 times.


 // No of pipes
  val[0] =0x01;
  WriteToNrf(W,EN_RXADDR,val,1); // Receiver address, Enable data pipe 0
 

 // Rf addr width
  val[0]=0x03;
  WriteToNrf(W,SETUP_AW, val,1);//Setup address width,"11"->5 bytes
  

 //  RF channel
  val[0]=0x01;
  WriteToNrf(W,RF_CH,val,1);// Set the frequency of channel (2400+Rf_ch reg)MHZ ->2.4GHZ
  

 //RF setup: 
  val[0]=0x07;
  WriteToNrf(W,RF_SETUP,val,1);//RF Setup Register, 0b00000111, bit 3->'1 Mbps', bit 2,1->'11'-0dBm RF output power in TX Mode
  Serial.println(GetReg(RF_SETUP), HEX);

 // Setting Receiver Address
  uint8_t *data;
  int i;
  for(i=0;i<5;i++)
  {
    val[i]=0x12;
  }
  WriteToNrf(W,RX_ADDR_P0, val ,5);              // Filling Receiver address pipe

 
  //  payload width
  val[0] =5;
  WriteToNrf(W,RX_PW_P0, val,1);// Number of bytes in reciver payload in data pipe 0("5 = 5 bytes of data at a time")
  

  //  CONFIG
  val[0]=0x0F;//0x0b00001110;                     //0b00011110, bit0-"Rx = 1" "Tx=0",bit1-"0->Power down, 1->Power up", bit2-"CRC =1->2bytes encoding scheme", 
  WriteToNrf(W,CONFIG,val,1);    //bit3-"1= enable crc", bit4-"MAx transmitting retries '0'->interrupt not reflected on IRQ,'1'->Reflect active low interupt on IRQ"
                                //bit5-"Bit used to MAsk(set) interrupt caused when nrf has succesfully transmitted a packet '0'->enable ,'1'->disable", bit6->"Bit used to mask(set) interrupt caused when data has been succesfully recieved,'1'->disable interrupt,'0'->enable enable interrupt                               
  delay(100);               // Delay of 100 milliseconds -> device needs 1.5ms to reach standby mode i.e to come in awake state (previously i was giving 100 microsecond delay)
 }

 //Transmitter nrf initialization
 void nrf_init_bitBang()
{
  delay(100);
  uint8_t val[5];

  resett_bitBang();
  Serial.println("Finding Transmitter NRF Module");
  char test = GetReg_bitBang(STATUS);
  Serial.print(test,HEX);
  if(test == 0x0E)
  {
    Serial.println("\tTransmitter Module found.....");
    Serial.println("\tConfiguring the Transmitter module....\n");
  }
   else {
    Serial.println("Module Not Found...");
    Serial.println("Exiting..");
    return;
  }

  //  Enable Auto Ack
  val[0] =0x01;
  WriteToNrf_bitBang(W,EN_AA, val,1); //Enable auto acknowledgement data pipe 0

 //  Retries and Delay
  val[0]=0x2F;
  WriteToNrf_bitBang(W,SETUP_RETR, val,1);//Setup automatic transmission, 0b00101111 so '0010'->after end of transmission wait for 750us and then retransmit and '1111'->Retransmit 15 times.
  

  //  No of pipes
  val[0] =0x01;
  WriteToNrf_bitBang(W,EN_RXADDR,val,1); // ReCE_piniver address, Enable data pipe 0
 
  
  //  Rf addr width
  val[0]=0x03;
  WriteToNrf_bitBang(W,SETUP_AW, val,1);//Setup address width,"11"->5 bytes
  Serial.println(GetReg_bitBang(SETUP_AW), HEX);

 //    RF channel
  val[0]=0x01;
  WriteToNrf_bitBang(W,RF_CH,val,1);// Set the frequency of channel (2400+Rf_ch reg)MHZ ->2.4GHZ
  

  //  RF setup
  val[0]=0x07;
  WriteToNrf_bitBang(W,RF_SETUP,val,1);//RF Setup Register, 0b00000111, bit 3->'1 Mbps', bit 2,1->'11'-0dBm RF output power in TX Mode
 
// defining pointer to unsigned 8 bit integer. *Data is used here to get the pointer returned by the *WriteToNrf Register
  uint8_t *data;
  int i;

 // Setting Address of NRF Transmitter
  for(i=0;i<5;i++)
  {
    val[i]=0x12;
  }
  WriteToNrf_bitBang(W,TX_ADDR, val ,5); //setting Transmission address as 0x1212121212 
 
  
 //  npayload width
  val[0] =5;
  WriteToNrf_bitBang(W,RX_PW_P0, val,1);// Number of bytes in reciver payload in data pipe 0("5 = 5 bytes of data at a time")
  

 //CONFIG
  val[0]=0x0E;//0b00001110;                     //0b00011110, bit0-"Rx = 1" "Tx=0",bit1-"0->Power down, 1->Power up", bit2-"CRC =1->2bytes encoding scheme", 
  WriteToNrf_bitBang(W,CONFIG,val,1);    //bit3-"1= enable crc", bit4-"MAx transmitting retries '0'->interrupt not reflected on IRQ,'1'->Reflect active low interupt on IRQ"
                                //bit5-"Bit used to MAsk(set) interrupt caused when nrf has sucCE_pinsfully transmitted a packet '0'->enable ,'1'->disable", bit6->"Bit used to mask(set) interrupt caused when data has been sucCE_pinsfully recieved,'1'->disable interrupt,'0'->enable enable interrupt                               
  delay(100);               // Delay of 100 milliseconds -> deviCE_pin needs 1.5ms to reach standby mode i.e to come in awake state (previously i was giving 100 microsecond delay)
 }


// Transmitter NR transmit function
 void transmit_nrf_bitBang(uint8_t *W_buff)
  {
    Serial.println("\nTransmitting...");
    WriteToNrf_bitBang(R, FLUSH_TX, W_buff,0); // Flushes transmitter FIFO
    WriteToNrf_bitBang(R, W_TX_PAYLOAD, W_buff, 5); // Write Tranmsitter payload
    delay(10);  //needs a 10ms delay to work after loading the nrf with the paylload for some reason
    digitalWrite(CE_pin,HIGH); //CE high = Transmit the data
    delayMicroseconds(20);  //delay at least 10us
    digitalWrite(CE_pin,LOW);  //CE Low = Stop Transmitting
    delay(10);             //long delay again before proceeding
  }

  // Receiver Nrf Function to receive data
  void receive_nrf(void)
  {
    sei();
    Serial.println("Receiving");
    digitalWrite(CE,HIGH);
    delay(1000);
    digitalWrite(CE,LOW);
    cli();
  }

  byte bitBang(byte _send)
{
  byte _receive = 0;
  for(int i =0; i<8; i++)
  {
    digitalWrite(MOSI_pin, bitRead(_send, (7-i)));
    digitalWrite(SCK_pin,HIGH);
    bitWrite(_receive, 7-i,digitalRead(MISO_pin));
    digitalWrite(SCK_pin,LOW);
  }
  return _receive;
  }

 
             
//uint8_t val[5] = {1,2,3,4,5};

void IRQ_init()
{
DDRE = DDRE &0xEF; // 0b11101111 Pin 4 of Port E is input
PORTE = PORTE | 0x10; // 0b00010000 Pin 4 of Port E is pulled up
}
void interrupt_init()
{
  cli();   // clear I bit of SREG i.e. Globally disable all interrupts
  EICRB = EICRB | 0X02; // 0b00000010 (10) it will ser Interrupt Sense Control bits of INT4 for FALLING EDGE Detction
  EIMSK = EIMSK | 0x10 ; //0b00010000 Setting(MAsking) INT4 bit in External interrupt MAsk Registeer
  sei();
}
uint8_t data;

// Interrupt Subroutine for Receiver
ISR(INT4_vect)
{
 cli();
 digitalWrite(CE,LOW);
// Getting data from Receiver payload Register 
 char *data;
 resett();
 data=WriteToNrf(R,R_RX_PAYLOAD,data,5);

// Displaying Received Data  
 Serial.print("\n Data :");
 for(int i=0;i<5;i++)
 {
 Serial.print(*(data+i));
 }
 Serial.println("");

// Storing Received data to array arr[]
 char arr[5];
 for(int i=0;i<5;i++)
 {
arr[i] = *(data+i);
 }

 // Matching the received data with the Required data 
 if(arr[2] == Buff[2])
 {
  Serial.println("NRF working");
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("NRF working");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
 }
 else
 {
  Serial.println("NRF not Working");
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("NRF not working");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
 }
 digitalWrite(CE,HIGH);
 sei();
 }

 void NRF()
 {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("NRF Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
  
  Soft_spi(); 
SPI_Init();
resett();
nrf_init_bitBang();
nrf_init();
IRQ_init();
interrupt_init();
sei();
while(1)
{
  transmit_nrf_bitBang(Buff);
    delay(130);
    resett_bitBang();
 resett();
 receive_nrf(); 
}
 }

//********************************************************************************************//
//********************************************************************************************// 
//BLUETOOTH

void BT()
{
  return;
}

//********************************************************************************************//
//********************************************************************************************// 
//SDCARD READER

void SDCARD()
{
  SPI_Init();
delay(20);
for(int i=0;i<10;i++)
{
 
  SPI_Write(0xff);
}


digitalWrite(MOSI,HIGH);        // Make data out pin high
digitalWrite(CSN,HIGH);          // MAke chip select pin high
int k=0;
byte x;

digitalWrite(CSN,LOW);       // MAke chip select pin low(Transition of chip select pin from high to low )
SPI_Write(0x40);             
SPI_Write(0x00>>24);
SPI_Write(0x00>>16);
SPI_Write(0x00>>8);
SPI_Write(0x00);
SPI_Write(0x95);
int ret = 0;
while(( x = SPI_Read())==0xff)
{
  
  if(ret++>0xfe)
  {
    break;
  }
}
    if(x == 0x01)
    {
      
      //Serial.println(x,HEX);
      Serial.println("SD card working");
    }
    else
    {
      Serial.println("SD card NOT working :(");
    }
    digitalWrite(CSN,HIGH);
    
  return;
}

//********************************************************************************************//
//********************************************************************************************// 
//DC MOTOR AND DRIVERS 

//mhPushCallback and mhPopCallback are the function for increase the pwm value, so that motor speed will increase upto its maximum speed (PWM 255) in Clockwise Direction
// It will also Reduce the Speed of motor running in Anti-clockwise direction.
void mhPushCallback(void *ptr) //This function is called when mh is Touched in Nextion HMI
{
  if(ms<255)
  {
   ms=ms+10;
  }
}  
void mhPopCallback(void *ptr) //This function is called when mh is Released in Nextion HMI
{
  if(ms<255)
  {
   ms=ms+10;
  }
  if(ms>255)
  {
    ms=255;
  }
}

//m0PushCallback and m0PopCallback are the function to make the PWM value to zero in order to stop motor.
void m0PushCallback(void *ptr) //This function is called when m0 is Touched in Nextion HMI
{
   ms=0;
}  
void m0PopCallback(void *ptr) //This function is called when m0 is Released in Nextion HMI
{
   ms=0;
}

//mlPushCallback and mlPopCallback are the function for increase the pwm value, so that motor speed will increase upto its maximum speed (PWM 255) in Anti-clockwise direction
// It will also Reduce the Speed of motor running in Clockwise direction.
void mlPushCallback(void *ptr) //This function is called when ml is Touched in Nextion HMI
{
  if(ms > -255)
  {
   ms=ms-10;
  }
  if(ms < -255)
  {
    ms=-255;
  }
}  
void mlPopCallback(void *ptr) //This function is called when mh is Released in Nextion HMI
{
  if(ms > -255)
  {
   ms=ms-10;
  }
  if(ms < -255)
  {
    ms=-255;
  }
}

void MOTOR()
{
  Back=0;
  while(1)
  {
    nexLoop(nex_listen_list); 

//printing PWM value in Nextion HMI
Serial.print("mot.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial.print("\"");
  Serial.print(ms);  // This is the value you want to send to that object and atribute mentioned before.
  Serial.print("\"");
  Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial.write(0xff);
  Serial.write(0xff);
  //PWM control for Clockwise Operation of Motor
  if(ms>=0)
  {
  digitalWrite(ma1,HIGH);
  digitalWrite(ma2,LOW);
  analogWrite(m,ms);
  }
  // PWM control for Anti-clockwise operation of Motor
  if(ms<0)
  {
    digitalWrite(ma1,LOW);
  digitalWrite(ma2,HIGH);
  analogWrite(m,(-ms));
  }

  //when back button is clicked in HMI, it will end the motor test by breaking the loop
  if(Back==1)
  {
    break;
  }
  }
  return;
}

//********************************************************************************************//
//********************************************************************************************// 
//SERVO MOTOR
#include <Servo.h> //Servo Library
Servo ser; //object for servo is ser

void SERVO()
{
  ser.attach(10,600,2300); //Assigning Digital Pin 10 for servo
  delay(1000);
  
  ser.write(0); // 0 degree position
  delay(1000);
  ser.write(90);  //90 degree position
  delay(1000);
  ser.write(180); //180 degree position
  delay(1000);
  ser.write(0); //0 degree position
  delay(1000);
  return;
}

//********************************************************************************************//
//********************************************************************************************// 
//MOTOR WITH ENCODER

void MOTOREN()
{
  return;
}
//********************************************************************************************//
//********************************************************************************************// 
//ACTIVE DEVICES TESTING

//*********************************FUNCTIONS OF ACTIVE DEVICES********************************
void getFuseBytes ()
  {
  Serial.print (F("LFuse = "));
  showHex (readFuse (lowFuse), true);
  Serial.print (F("HFuse = "));
  showHex (readFuse (highFuse), true);
  Serial.print (F("EFuse = "));
  showHex (readFuse (extFuse), true);
  Serial.print (F("Lock byte = "));
  showHex (readFuse (lockByte), true);
  Serial.print (F("Clock calibration = "));
  showHex (readFuse (calibrationByte), true);
  }  // end of getFuseBytes

bootloaderType currentBootloader;


// burn the bootloader to the target device
void writeBootloader ()
  {
  bool foundBootloader = false;

  for (unsigned int j = 0; j < NUMITEMS (bootloaders); j++)
    {

    memcpy_P (&currentBootloader, &bootloaders [j], sizeof currentBootloader);

    if (memcmp (currentSignature.sig, currentBootloader.sig, sizeof currentSignature.sig) == 0)
      {
      foundBootloader = true;
      break;
      }  // end of signature found
    }  // end of for each signature

  if (!foundBootloader)
    {
       Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("No bootloader support for this device."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
    
    return;
    }

  // if in the table, but with zero length, we need to enable a #define to use it.
  if (currentBootloader.loaderLength == 0)
    {
    
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("Bootloader for this device is disabled, edit " __FILE__ " to enable it."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
    return;
    }

  unsigned int i;

  byte lFuse = readFuse (lowFuse);

  byte newlFuse = currentBootloader.lowFuse;
  byte newhFuse = currentBootloader.highFuse;
  byte newextFuse = currentBootloader.extFuse;
  byte newlockByte = currentBootloader.lockByte;


  unsigned long addr = currentBootloader.loaderStart;
  unsigned int  len = currentBootloader.loaderLength;
  unsigned long pagesize = currentSignature.pageSize;
  unsigned long pagemask = ~(pagesize - 1);
  const byte * bootloader = currentBootloader.bootloader;


  byte subcommand = 'U';

  // Atmega328P or Atmega328
  if (currentBootloader.sig [0] == 0x1E &&
      currentBootloader.sig [1] == 0x95 &&
      (currentBootloader.sig [2] == 0x0F || currentBootloader.sig [2] == 0x14)
      )
    //{
    //Serial.println (F("Type 'L' to use Lilypad (8 MHz) loader, or 'U' for Uno (16 MHz) loader ..."));
//    subcommand = 'U';
//    do
//      {
//      subcommand = toupper (Serial.read ());
//      } while (subcommand != 'L' && subcommand != 'U');
//
//    if (subcommand == 'L')  // use internal 8 MHz clock
//      {
//      Serial.println (F("Using Lilypad 8 MHz loader."));
//      bootloader = ATmegaBOOT_168_atmega328_pro_8MHz_hex;
//      newlFuse = 0xE2;  // internal 8 MHz oscillator
//      newhFuse = 0xDA;  //  2048 byte bootloader, SPI enabled
//      addr = 0x7800;
//      len = sizeof ATmegaBOOT_168_atmega328_pro_8MHz_hex;
//      }  // end of using the 8 MHz clock
//    else
      
          Serial.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("Using Uno Optiboot 16 MHz loader."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
     //}  // end of being Atmega328P


  

    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
Serial3.print (F("Bootloader address = 0x"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  //delay(1000);
  
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (addr, HEX);
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
 
  

    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print (F("Bootloader length = "));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  //delay(1000);
 

    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.print (len);
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  //delay(1000);
  

    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
 Serial3.println (F(" bytes."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);


  unsigned long oldPage = addr & pagemask;

         Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F("Verifying ..."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);

  // count errors
  unsigned int errors = 0;
  // check each byte
  for (i = 0; i < len; i++)
    {
    byte found = readFlash (addr + i);
    byte expected = pgm_read_byte(bootloader + i);
    if (found != expected)
      {
      if (errors <= 100)
        {
       // Serial.print (F("Verification error at address "));
       // Serial.print (addr + i, HEX);
       // Serial.print (F(". Got: "));
        showHex (found);
        //Serial.print (F(" Expected: "));
        showHex (expected, true);
        }  // end of haven't shown 100 errors yet
      errors++;
      }  // end if error
    }  // end of for

  if (errors == 0)
   {
           Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
 Serial3.println (F("No errors found."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
   }
  else
    {
    //Serial.print (errors, DEC);
           Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F(" verification error(s)."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000); 
    if (errors > 100)
      Serial.println (F("First 100 shown."));
   // return;  // don't change fuses if errors
    }  // end 

 // Serial3.println (F("Type 'Q' to quit, 'V' to verify, or 'G' to program the chip with the bootloader ..."));
 
//  do
//    {
//    command = toupper (Serial.read ());
//    } while (command != 'G' && command != 'V' && command != 'Q');
//   
  // let them do nothing
  if(errors  != 0)                                    // Checking for the condition whether bootloader is there or not 
  {
     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F("PRESS UPLOAD BOOTLOADER "));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  int i = 0;
  while(1)                                           //Asking Whether want to upload Bootloader or Quit 
  {
    
    nexLoop(nex_listen_list);
     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  // Serial3.print(F("waiting "));
  Serial3.print(upload);
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
    if(upload == 1)
    {
      Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.print (F("Inside upload "));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
      command = 'G';
      break;
    }
    if(Back == 1)
    {
      return;
    }
   
  }
 

  if (command == 'G')
    {

    // Automatically fix up fuse to run faster, then write to device
    if (lFuse != newlFuse)
      {
      if ((lFuse & 0x80) == 0)
       // Serial.println (F("Clearing 'Divide clock by 8' fuse bit."));

      //Serial.println (F("Fixing low fuse setting ..."));
      writeFuse (newlFuse, lowFuse);
      delay (1000);
      stopProgramming ();  // latch fuse
      if (!startProgramming ())
        return;
      delay (1000);
      }

     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F("Erasing chip ..."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    
    eraseMemory ();
    
     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F("Writing bootloader ..."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    for (i = 0; i < len; i += 2)
      {
      unsigned long thisPage = (addr + i) & pagemask;
      // page changed? commit old one
      if (thisPage != oldPage)
        {
        commitPage (oldPage, true);
        oldPage = thisPage;
        }
      writeFlash (addr + i, pgm_read_byte(bootloader + i));
      writeFlash (addr + i + 1, pgm_read_byte(bootloader + i + 1));
      }  // end while doing each word

    // commit final page
    commitPage (oldPage, true);
    
        Serial.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("Written."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);

    }  // end if programming

  
       Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F("Verifying ..."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);

  // count errors
  unsigned int errors = 0;
  // check each byte
  for (i = 0; i < len; i++)
    {
    byte found = readFlash (addr + i);
    byte expected = pgm_read_byte(bootloader + i);
    if (found != expected)
      {
      if (errors <= 100)
        {
       // Serial.print (F("Verification error at address "));
       // Serial.print (addr + i, HEX);
       // Serial.print (F(". Got: "));
        showHex (found);
        //Serial.print (F(" Expected: "));
        showHex (expected, true);
        }  // end of haven't shown 100 errors yet
      errors++;
      }  // end if error
    }  // end of for

  if (errors == 0)
   {
           Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
 Serial3.println (F("No errors found."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
   }
  else
    {
    //Serial.print (errors, DEC);
           Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.println (F(" verification error(s)."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
   
    if (errors > 100)
      Serial.println (F("First 100 shown."));
    return;  // don't change fuses if errors
    }  // end if

 
    //Serial.println (F("Writing fuses ..."));

    writeFuse (newlFuse, lowFuse);
    writeFuse (newhFuse, highFuse);
    writeFuse (newextFuse, extFuse);
    writeFuse (newlockByte, lockByte);

    // confirm them
    getFuseBytes ();
      // end if programming

  
      Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.print(F("Done."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  }
 
     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
   Serial3.print(F("WORKING PROPERLY"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  } // end of writeBootloader

void getSignature ()
  {
  foundSig = -1;

  byte sig [3];
 // Serial.print (F("Signature = "));
  readSignature (sig);
  for (byte i = 0; i < 3; i++)
    showHex (sig [i]);

  //Serial.println ();

  for (unsigned int j = 0; j < NUMITEMS (signatures); j++)
    {

    memcpy_P (&currentSignature, &signatures [j], sizeof currentSignature);

    if (memcmp (sig, currentSignature.sig, sizeof sig) == 0)
      {
      foundSig = j;
                Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print (F("Processor = "));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
      
                Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (currentSignature.desc);
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);

       Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("Flash memory size = "));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
      //Serial.print (F("Flash memory size = "));
     //Serial.print (currentSignature.flashSize, DEC);
     
       Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (currentSignature.flashSize, DEC);
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    // Serial.println (F(" bytes."));
     if (currentSignature.timedWrites)
       Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("Writes are timed, not polled."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
    //    Serial.println (F("Writes are timed, not polled."));
       return;
      }  // end of signature found
    }  // end of for each signature
//
              Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
 Serial.println (F("Unrecogized signature."));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
Serial.println (F("Unrecogized signature."));
  }  // end of getSignature

//***************************************Active device main function*************************//
void ACTIVE()
{
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("INSIDE ACTIVE"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  initPins ();
   stopProgramming ();
  if (startProgramming ())
    {
        Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("Inside Start Programming"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    getSignature ();
    
        Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("SIGNATURES OBTAINED"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    getFuseBytes ();
      Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("FUSES OBTAINED"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);

    // if we found a signature try to write a bootloader
    if (foundSig != -1)
    {
      writeBootloader ();
    }
      Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("OUT OF WRITE BOOTLOADER FUNCTION"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    stopProgramming ();
     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("programming stopped"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    }
    return;
}

//********************************************************************************************//
//********************************************************************************************// 




void setup() {
  Serial3.begin(9600);
  pinMode(d11,OUTPUT);
  digitalWrite(d11,LOW);
  current();
  Serial.begin(9600);
  delay(1000);
Wire.begin();SPI.begin();
  mpr.attachPush(mprPushCallback);  
  mpr.attachPop(mprPopCallback); 

  mpu.attachPush(mpuPushCallback);  
  mpu.attachPop(mpuPopCallback);

  gy.attachPush(gyPushCallback);  
  gy.attachPop(gyPopCallback);

  rfid.attachPush(rfidPushCallback);  
  rfid.attachPop(rfidPopCallback);

  tcs.attachPush(tcsPushCallback);  
  tcs.attachPop(tcsPopCallback);

  pir.attachPush(pirPushCallback);  
  pir.attachPop(pirPopCallback);

  ultra.attachPush(ultraPushCallback);  
  ultra.attachPop(ultraPopCallback);

  joy.attachPush(joyPushCallback);  
  joy.attachPop(joyPopCallback);

  bt.attachPush(btPushCallback);  
  bt.attachPop(btPopCallback);

  xbee.attachPush(xbeePushCallback);  
  xbee.attachPop(xbeePopCallback);

  nrf.attachPush(nrfPushCallback);  
  nrf.attachPop(nrfPopCallback);

  sdc.attachPush(sdcPushCallback);  
  sdc.attachPop(sdcPopCallback);

  motor.attachPush(motorPushCallback);  
  motor.attachPop(motorPopCallback);

  motoren.attachPush(motorenPushCallback);  
  motoren.attachPop(motorenPopCallback);

  servo.attachPush(servoPushCallback);  
  servo.attachPop(servoPopCallback);

  l298n.attachPush(l298nPushCallback);  
  l298n.attachPop(l298nPopCallback);

  drv.attachPush(drvPushCallback);  
  drv.attachPop(drvPopCallback);

  mega.attachPush(megaPushCallback);  
  mega.attachPop(megaPopCallback);

  uno.attachPush(unoPushCallback);  
  uno.attachPop(unoPopCallback);

  nano.attachPush(nanoPushCallback);  
  nano.attachPop(nanoPopCallback);

  atmega.attachPush(atmegaPushCallback);  
  atmega.attachPop(atmegaPopCallback);


  start.attachPush(startPushCallback);  
  start.attachPop(startPopCallback);

  startj.attachPush(startjPushCallback);  
  startj.attachPop(startjPopCallback);

  starta.attachPush(startaPushCallback);  
  starta.attachPop(startaPopCallback);

  back.attachPush(backPushCallback);  
  back.attachPop(backPopCallback);

  backa.attachPush(backPushCallback);

  backp.attachPush(backPushCallback);

  backj.attachPush(backPushCallback);
  
  Upload.attachPush(UploadPushCallback);  
  Upload.attachPop(UploadPopCallback);

  //Motor pins
  pinMode(ma1,OUTPUT);
  pinMode(ma2,OUTPUT);
  pinMode(m,OUTPUT);

  mh.attachPush(mhPushCallback);
  mh.attachPop(mhPopCallback);
  
  m0.attachPush(m0PushCallback);
  m0.attachPop(m0PopCallback);
  
  ml.attachPush(mlPushCallback);
  ml.attachPop(mlPopCallback);

  Serial3.print("page 0"); // This is the text you want to send to that object and atribute mentioned before.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(3000);
  
  // put your setup code here, to run once:

}

void loop() {

  nexLoop(nex_listen_list);
  current();

  if(startTest==1)
  {
    current();
    
  //MPR121 TEST
  if(selction==1)
  {
   Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("MPR121 Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000); 
    MPR121();
  }

  //MPU6050 TEST
  if (selction==2)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("MPU6050 Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    MPU6050();
  }

  //GY87 TEST
  if (selction==3)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("GY87 Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    BMP180();
  }

  //RFID TEST
  if (selction==4)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("RFID Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    RFID();
  }

  //COLOR SENSOR TEST
  if(selction==5)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("COLOR SENSOR Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    TCS3200();
  }

  //PIR TEST
  if (selction==6)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("PIR Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    PIR();
  }
   //ULTRASONIC TEST
  if(selction==7)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("ULTRASONIC Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  ultrasonic();
  }

  //JOYSTICK MODULE TEST
  if (selction==8)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("JOYSTICK Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    joystick();
  }

  //BLUETOOTH MODULE TEST
  if(selction==9)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("BLUETOOTH Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    BT();
  }

  //XBEE TEST
  if (selction==10)
  {
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("XBEE Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    XBEE();
  }

  //NRF TEST
  if(selction==11)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("NRF Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    NRF();
  }

  //SDCARD TEST
  if(selction==12)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("SDCARD READER Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    SDCARD(); 
  }

  //DC MOTOR TEST
  if(selction==13)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("DC MOTOR Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    MOTOR(); 
  }

  //DC MOTOR WITH ENCODER TEST
  if(selction==14)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("DC MOTOR WITH ENCODER Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    MOTOREN(); 
  }

  //SERVO MOTOR TEST
  if(selction==15)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("SERVO MOTOR Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    SERVO(); 
  }

  //L298N MOTOR DRIVER TEST
  if(selction==16)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("L298N Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    MOTOR(); 
  }

  //DRV8833 MOTOR DRIVER TEST
  if(selction==17)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("DRV8833 Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    MOTOR(); 
  }

  //ARDUINO MEGA TEST
  if(selction==18)
  {
     Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("ARDUINO MEGA Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  
     while(1)
      {
        nexLoop(nex_listen_list);
           Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(startTest);  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
        
       if(startTest==1)
         {
            Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print(F("Inside If"));  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
           ACTIVE();
           startTest = 0;
           
         }
       if(Back == 1)
         {
           Back =0;
           break;
           //return;
         }
       }
  } // end of Selection 18     
      

  //ARDUINO UNO TEST
  if(selction==19)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("ARDUINO UNO Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    ACTIVE(); 
  }

  //ARDUINO NANO TEST
  if(selction==20)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("ARDUINO NANO Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    ACTIVE(); 
  }

  //ATMEGA2560 DEVELOPMENT BOARD TEST
  if(selction==21)
  {
    Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.print("ATMEGA Test Starts...");  // This is the text you want to send to that object and atribute mentioned before.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
    ACTIVE(); 
  }
  Serial3.print("monitor.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.println (F("OUT OF FUNCTION"));
  Serial3.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial3.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial3.write(0xff);
  Serial3.write(0xff);
  delay(1000);
  
selction=0;startTest=0;
  }
  // put your main code here, to run repeatedly:

}
