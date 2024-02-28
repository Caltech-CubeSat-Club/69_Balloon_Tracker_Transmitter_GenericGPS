                             /*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 29/12/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//**************************************************************************************************
// 1) Hardware related definitions and options - specify lora board type and pins here
//**************************************************************************************************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup. 

#define NSS 6                                  //select on LoRa device
#define NRESET -1                                //reset on LoRa device
#define DIO0 7                                  //DIO0 on LoRa device, used for RX and TX done 
#define LED1 2                                  //On board LED, high for on
#define BME280_ADDRESS_EXT 0x76                          //for DS18B20 temperature sensor 

#define RXpin 8                                //pin number for GPS RX input into Arduino - TX from GPS
#define TXpin 9                                //pin number for GPS TX output from Arduino- RX into GPS

#define SD_CS 10                                //SD card select pin

#define LORA_DEVICE DEVICE_SX1276               //this is the device we are using

#define SD_FILENAME "HABData.txt"               //name of file to store data on SD card

//**************************************************************************************************
// 2) Program Options
//**************************************************************************************************

//#define ClearAllMemory                          //Clears memory of stored tracker information, counts, errors etc

//**************************************************************************************************
// 3) LoRa modem settings 
//**************************************************************************************************
  // carrier frequency:                   868.0 MHz
  // bit rate:                            300.0 kbps
  // frequency deviation:                 60.0 kHz
  // output power:                        17 dBm
  // preamble length:                     32 bits

//Tracker mode
const uint32_t TrackerFrequency = 445000000;     // carrier frequency in Hz
const int8_t TrackerTXpower = 20;                //LoRa TX power in dBm

const uint8_t TXBUFFER_SIZE = 128;               //defines the maximum size of the trasnmit buffer;


//**************************************************************************************************
// 4) GPS Options - Settings for a generic GPS, no attempt is made to put the GPS into balloon mode
//**************************************************************************************************

#define GPSBaud 9600                              //GPS Baud rate

#define USESOFTSERIALGPS                          //if your using software serial for the GPS, enable this define      

//#define USEHARDWARESERIALGPS                    //if your using hardware serial for the GPS, enable this define
// #define HARDWARESERIALPORT Serial1                //if your using hardware serial for the GPS, define the port here  

const uint16_t WaitGPSFixSeconds = 60;            //when in flight the time to wait for a new GPS fix

                                                   
//**************************************************************************************************
// 7) Memory settings - define the type of memory to use for non-Volatile storage.
//    Default is internal ATmega device EEPROM but EEPROM has a limited write endurance of 'only' 
//    100,000 writes. Since the non-Volatile memory selected is written to at each transmission loop
//    and error, its highly recommended to use one of the FRAM options, these have an endurance of
//    100,000,000,000,000 writes.   
//**************************************************************************************************

#define Memory_Library <EEPROM_Memory.h>
//#define Memory_Library <FRAM_MB85RC16PNF.h>
//#define Memory_Library <FRAM_FM24CL64.h>

int16_t Memory_Address = 0x50;                     //default I2C address of MB85RC16PNF and FM24CL64 FRAM

//**************************************************************************************************
// 8) HAB Flight Settings
//**************************************************************************************************

char FlightID[] = "Flight1";                       //flight ID for HAB packet

const uint16_t SleepTimesecs = 13;             //sleep time in seconds after each TX loop

const char ThisNode = '1';                         //tracker number for search packet


