/******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 28/12/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is a tracker intended for use as a high altitude balloon (HAB) tracker. The
  program sends out a standard format payload with LoRa that is compatible with the HABHUB online tracking
  system.

  The HAB payload is constructed thus;

  PayloadID,Sequence,Time,Lat,Lon,Alt,Satellites,Volts,Temperature,Resets,Status,Errors,TXGPSfixms,Checksum
  Field 0      1      2    3   4   5      6        7       8         9      10     11        12       13

  The LoRa and frequency settings can be changed in the Settings.h file. There is the option of sending
  out a much shorter Search mode binary location only payload. This is intended for ground based searching
  and locating. The frequency and LoRa settings of the Search mode packet can be different to the Tracker
  mode used by the HAB payload. There is also the option of sending the HAB payload in FSK RTTY format,
  see the Settings.h file for all the options. FSK RTTY gets sent at the same frequency as the Tracker mode
  HAB packet. The LT.transmitFSKRTTY() function sends at 1 start bit, 7 data bits, no parity and 2 stop bits.
  For full control of the FSK RTTY setting you can use the following alternative function;

  LT.transmitFSKRTTY(chartosend, databits, stopbits, parity, baudPerioduS, pin)

  There is a matching Balloon Tracker Receiver program which writes received data to the Serial monitor as well
  as a small OLED display.

  In the Settings.h file you can set the configuration for either a Ublox GPS or a Quectel L70\L80. The GPSs
  are configured for high altitude balloon mode.

  It is strongly recommended that a FRAM option is fitted for this transmitter. The sequence, resets and error
  nembers are stred in non-volatile memory. This defaults to EEPROM which has a limited endurance of only
  100,000 writes, so in theory the limt is reached after the transmission of 100,000 hab packets. The use of
  a FRAM will extend the life of the tracker to circa 100,000,000,000,000 transmissions.

  Changes:
  240420 - Change to work with Easy Pro Mini style modules
  300420 - Improve error detection for UBLOX GPS library

  ToDo:

  Serial monitor baud rate is set at 115200
*******************************************************************************************************/

#define Program_Version "V1.1"

#include <Arduino.h>

#include <SX127XLT.h>                            //include the appropriate library  

SX127XLT LT;                                     //create a library class instance called LT

#include "Settings.h"
#include "ProgramLT_Definitions.h"

//**************************************************************************************************
// HAB tracker data - these are the variables transmitted in payload
//**************************************************************************************************
uint32_t TXSequence;                             //sequence number of payload
uint8_t TXHours;                                 //Hours
uint8_t TXMinutes;                               //Minutes
uint8_t TXSeconds;                               //Seconds
float TXLat;                                     //latitude from GPS
float TXLon;                                     //longitude from GPS
uint16_t TXAlt;                                  //altitude from GPS
uint8_t TXSatellites;                            //satellites used by GPS
float TXmAmps;                                //measured tracker supply current
int8_t TXTemperature;                            //measured temperature
float TXBattTemp;                                //measured battery temperature
uint16_t TXResets;                               //number of tracker resets
uint8_t TXStatus = 0;                            //used to store current status flag bits
uint16_t TXErrors;                               //number of tracker Errors
uint32_t TXGPSfixms;                             //fix time of GPS
//**************************************************************************************************

uint8_t TXPacketL;                               //length of LoRa packet sent
uint8_t  TXBUFFER[TXBUFFER_SIZE];                //buffer for packet to send

#include Memory_Library

#include <SPI.h>

#include <TinyGPS++.h>                           //http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                 //create the TinyGPS++ object

#ifdef USESOFTSERIALGPS
//#include <NeoSWSerial.h>                       //https://github.com/SlashDevin/NeoSWSerial
//NeoSWSerial GPSserial(RXpin, TXpin);           //The NeoSWSerial library is an option to use and is more reliable
//at GPS init than software serial
#include <SoftwareSerial.h>
SoftwareSerial GPSserial(RXpin, TXpin);
#endif

#ifdef USEHARDWARESERIALGPS
#define GPSserial HARDWARESERIALPORT
#endif

#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <SparkFunBME280.h>
#include <SdFat.h>
BME280 bme280_ext;
LSM9DS1 lsm9ds1;
File sdcard;
SdFat SD;


uint32_t GPSstartms;                             //start time waiting for GPS to get a fix


void loop()
{
  Serial.println(F("Start Loop"));

  GPSstartms = millis();

  if (!gpsWaitFix(WaitGPSFixSeconds))
  {
    GPSOutputOff();
    sendCommand(NoFix);                          //report a GPS fix error
    delay(1000);                                 //give receiver enough time to report NoFix
  }
  Serial.println();

  do_Transmissions();                            //do the transmissions

  Serial.println(F("Sleep"));
  LT.setSleep(CONFIGURATION_RETENTION);          //put LoRa device to sleep, preserve lora register settings
  Serial.flush();                                //make sure no serial output pending before goint to sleep

  delay(SleepTimesecs * 1000);

  Serial.println(F("Wake"));
  LT.wake();                                     //wake the LoRa device from sleep
}


void do_Transmissions()
{
  //this is where all the transmisions get sent
  uint32_t startTimemS;
  uint8_t index;

  incMemoryUint32(addr_SequenceNum);             //increment Sequence number

  TXPacketL = buildHABPacket();
  Serial.print(F("HAB Packet > "));
  printBuffer(TXBUFFER, (TXPacketL + 1));             //print the buffer (the packet to send) as ASCII
  saveBuffer(TXBUFFER, (TXPacketL + 1));              //save the buffer to SD card
  digitalWrite(LED1, HIGH);
  startTimemS = millis();
  TXPacketL = LT.transmit(TXBUFFER, (TXPacketL + 1), 10000, TrackerTXpower, WAIT_TX); //will return packet length sent if OK, otherwise 0 if transmit error
  digitalWrite(LED1, LOW);
  printTXtime(startTimemS, millis());
  reportCompletion();
  Serial.println();
}


void printTXtime(uint32_t startmS, uint32_t endmS)
{
  Serial.print(F(" "));
  Serial.print(endmS - startmS);
  Serial.print(F("mS"));
}


void reportCompletion()
{
  Serial.print(F(" "));
  if (TXPacketL == 0)
  {
    Serial.println();
    reporttransmitError();
  }
  else
  {
    Serial.print(TXPacketL);
    Serial.print(F("bytes"));
    setStatusByte(LORAError, 0);
  }
}


void printBuffer(uint8_t *buffer, uint8_t size)
{
  uint8_t index;

  for (index = 0; index < size; index++)
  {
    Serial.write(buffer[index]);
  }
}


void saveBuffer(uint8_t *buffer, uint8_t size)
{

  sdcard = SD.open(SD_FILENAME, FILE_WRITE);
  if (!sdcard)
  {
    sendCommand(NoSD);
    Serial.println(F("Error opening file"));
    return;
  }

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    sdcard.print(buffer[index]);
  }

  sdcard.println();
  sdcard.flush();
  sdcard.close();
}


uint8_t buildHABPacket()
{
  //build the HAB tracker payload
  uint16_t index, j, CRC;
  uint8_t Count, len;
  char LatArray[12], LonArray[12];

  TXSequence = readMemoryUint32(addr_SequenceNum);              //Sequence number is kept in non-volatile memory so it survives TXResets
  TXResets =  readMemoryUint16(addr_ResetCount);                //reset count is kept in non-volatile memory so it survives TXResets
  TXmAmps = readBatteryCurrent();
  Serial.print(F("TXmAmps "));
  Serial.print(TXmAmps);
  Serial.println(F("mV"));
  TXTemperature = (int8_t) readExternalTemp();
  TXErrors = readMemoryUint16(addr_TXErrors);

  dtostrf(TXLat, 7, 5, LatArray);                                //format is dtostrf(FLOAT,WIDTH,PRECISION,BUFFER);
  dtostrf(TXLon, 7, 5, LonArray);                                //converts float to character array

  len = sizeof(TXBUFFER);
  memset(TXBUFFER, 0, len);                                      //clear array to 0s
  Count = snprintf((char*) TXBUFFER,
                   TXBUFFER_SIZE,
                   "$%s,%lu,%02d:%02d:%02d,%s,%s,%d,%d,%d,%02d,%d,%d,%d,%d,%lu",
                   FlightID,
                   TXSequence,
                   TXHours,
                   TXMinutes,
                   TXSeconds,
                   LatArray,
                   LonArray,
                   TXAlt,
                   TXSatellites,
                   TXmAmps,
                   TXBattTemp,
                   TXTemperature,
                   TXResets,
                   TXStatus,
                   TXErrors,
                   TXGPSfixms
                  );

  CRC = 0xffff;                                   //start value for CRC16

  for (index = 1; index < Count; index++)         //element 1 is first character after $ at start (for LoRa)
  {
    CRC ^= (((uint16_t)TXBUFFER[index]) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  TXBUFFER[Count++] = '*';
  TXBUFFER[Count++] = Hex((CRC >> 12) & 15);      //add the checksum bytes to the end
  TXBUFFER[Count++] = Hex((CRC >> 8) & 15);
  TXBUFFER[Count++] = Hex((CRC >> 4) & 15);
  TXBUFFER[Count] = Hex(CRC & 15);
  return Count;
}


char Hex(uint8_t lchar)
{
  //used in CRC calculation in buildHABPacket
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}


uint8_t buildLocationOnly(float Lat, float Lon, uint16_t Alt, uint8_t stat)
{
  uint8_t len;
  LT.startWriteSXBuffer(0);                   //initialise buffer write at address 0
  LT.writeUint8(LocationBinaryPacket);        //identify type of packet
  LT.writeUint8(Broadcast);                   //who is the packet sent too
  LT.writeUint8(ThisNode);                    //tells receiver where is packet from
  LT.writeFloat(Lat);                         //add latitude
  LT.writeFloat(Lon);                         //add longitude
  LT.writeInt16(Alt);                         //add altitude
  LT.writeUint8(stat);                        //add tracker status
  len = LT.endWriteSXBuffer();                //close buffer write
  return len;
}


void reporttransmitError()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();              //read the the interrupt register
  Serial.print(F("TXError,"));
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                //print IRQ status
  LT.printIrqStatus();                         //prints the text of which IRQs set
  incMemoryUint16(addr_TXErrors);              //increase the error count
  setStatusByte(LORAError, 1);
}


void incMemoryUint32(uint32_t addr)
{
  uint32_t val = readMemoryUint32(addr);
  val++;
  writeMemoryUint32(addr, val);
}


void incMemoryUint16(uint32_t addr)
{
  uint16_t val = readMemoryUint16(addr);
  val++;
  writeMemoryUint16(addr, val);
}


void setStatusByte(uint8_t bitnum, uint8_t bitval)
{
  //program the status byte

  if (bitval == 0)
  {
    bitClear(TXStatus, bitnum);
  }
  else
  {
    bitSet(TXStatus, bitnum);
  }
}


void setTrackerMode()
{
  Serial.println(F("setTrackerMode"));
  LT.setupFSK(TrackerFrequency);
}


uint8_t sendCommand(char cmd)
{
  uint8_t len;
  TXmAmps = readBatteryCurrent();
  Serial.print(F("Send Cmd "));
  Serial.write(cmd);
  Serial.println();

  LT.startWriteSXBuffer(0);                 //start the write packet to buffer process
  LT.writeUint8(cmd);                       //this byte defines the packet type
  LT.writeUint8(Broadcast);                 //destination address of the packet, the receivers address
  LT.writeUint8(ThisNode);                  //source address of this node
  LT.writeUint16(TXmAmps);                  //add the battery voltage
  LT.writeUint16((uint16_t) readBatteryTemp()); //add the external temperature
  LT.writeUint8(TXStatus);                  //add the status byte
  len = LT.endWriteSXBuffer();              //close the packet, get the length of data to be sent

  //now transmit the packet, set a timeout of 5000mS, wait for it to complete sending

  digitalWrite(LED1, HIGH);                 //turn on LED as an indicator
  TXPacketL = LT.transmitSXBuffer(0, len, 5000, TrackerTXpower, WAIT_TX);
  digitalWrite(LED1, LOW);                  //turn off indicator LED

  return TXPacketL;                         //TXPacketL will be 0 if there was an error sending
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  //flash LED to show tracker is alive
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void clearAllMemory()
{
  //clears the whole of non-volatile
  Serial.println(F("Clear Memory"));
  fillMemory(addr_StartMemory, addr_EndMemory, 0);
}


float readExternalTemp()
{
  return bme280_ext.readTempC();
}


void printExtTemp()
{
  Serial.print(F("Temperature "));
  Serial.print(readExternalTemp(), 1);
  Serial.println(F("C"));
}


float readAltitude()
{
  return bme280_ext.readFloatAltitudeMeters();
}


void printAltitude()
{
  Serial.print(F("Altitude "));
  Serial.print(readAltitude(), 1);
  Serial.println(F("m"));
}


float readPressure()
{
  return bme280_ext.readFloatPressure();
}


void printPressure()
{
  Serial.print(F("Pressure "));
  Serial.print(readPressure(), 1);
  Serial.println(F("Pa"));
}


void printBatteryCurrent()
{
  Serial.print(F("Current "));
  Serial.print(readBatteryCurrent());
  Serial.println(F("mA"));
}


float readBatteryCurrent()
{
 // Number of samples to average the reading over
  // Change this to make the reading smoother... but beware of buffer overflows!
  const int avgSamples = 20;

  int sensorValue = 0;

  float sensitivity = -90.0 / 80.0; //90mA per 770mV. Inverted
  float Vref = 2435; // Output voltage with no current: ~ 1950mV

  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue += analogRead(A0);

    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    // DOES DELAY, SO IF OTHER STUFF STOPS WHILE SENSING CURRENT, IT'S BECAUSE OF THIS
    delay(2);

  }

  sensorValue = sensorValue / avgSamples;

  // The on-board ADC is 10-bits -> 2^10 = 1024 -> 5V / 1024 ~= 4.88mV
  // The voltage is in millivolts
  float voltage = 4.88 * sensorValue;

  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure
  return (voltage - Vref) * sensitivity;
}


void printBatteryTemp()
{
  Serial.print(F("Battery Temp "));
  Serial.print(readBatteryTemp(), 1);
  Serial.println(F("C"));
}


float readBatteryTemp() {
  uint8_t i;
  float average;
  float samples[5];

  // take N samples in a row, with a slight delay
  for (i=0; i < 5; i++) {
   samples[i] = analogRead(A1);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< 5; i++) {
     average += samples[i];
  }
  average /= 5;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = 100000 / average; // 100 kOhm resistor is used

  float steinhart;
  steinhart = average / 12200;     // (R/Ro) Ro is resistance at a reference temp.
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (20.5556 + 273.15); // + (1/To) To is lab temperature 20.5556C
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  
  return steinhart;
}


//***********************************************************
// Start GPS Functions
//***********************************************************

void GPSTest()
{
  uint8_t GPSchar;
  uint32_t startmS;
  startmS = millis();

  while ( (uint32_t) (millis() - startmS) < 2000)       //allows for millis() overflow
  {
    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      Serial.write(GPSchar);
    }
  }
  Serial.println();
  Serial.println();
  Serial.flush();
}


bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix

  uint32_t startmS, waitmS;
  uint8_t GPSchar;

  Serial.flush();

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F("s "));
  Serial.flush();

  GPSOutputOn();

  waitmS = waitSecs * 1000;
  startmS = millis();

  while ( (uint32_t) (millis() - startmS) < waitmS)       //allows for millis() overflow
  {

    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      gps.encode(GPSchar);
      Serial.write(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated())
    {
      TXLat = gps.location.lat();
      TXLon = gps.location.lng();
      TXAlt = (uint16_t) gps.altitude.meters();

      //Altitude is used as an unsigned integer, so that the binary payload is as short as possible.
      //However gps.altitude.meters(); can return a negative value which converts to
      //65535 - Altitude, which we dont want. So we will assume any value over 60,000M is zero

      if (TXAlt > 60000)
      {
        TXAlt = 0;
      }

      TXHours = gps.time.hour(),
      TXMinutes = gps.time.minute(),
      TXSeconds = gps.time.second(),
      TXSatellites = gps.satellites.value();

      setStatusByte(GPSFix, 1);

      TXGPSfixms = millis() - GPSstartms;

      Serial.flush();
      Serial.println();
      Serial.print(F("Have GPS Fix "));
      Serial.print(TXGPSfixms);
      Serial.print(F("mS"));
      Serial.println();
      GPSprintTime();
      GPSprintDate();
      Serial.flush();

      return true;
    }

  }

  //if here then there has been no fix and a timeout
  GPSOutputOff();
  setStatusByte(GPSFix, 0);                     //set status bit to flag no fix
  incMemoryUint16(addr_TXErrors);
  Serial.println(F("Error No GPS Fix"));
  return false;
}


void GPSprintTime()
{
  uint8_t hours, mins, secs;
  hours = gps.time.hour();
  mins = gps.time.minute();
  secs = gps.time.second();

  Serial.print(F("Time "));

  if (hours < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(hours);
  Serial.print(F(":"));

  if (mins < 10)
  {
    Serial.print(F("0"));
  }
  Serial.print(mins);
  Serial.print(F(":"));

  if (secs < 10)
  {
    Serial.print(F("0"));
  }
  Serial.println(secs);
}


void GPSprintDate()
{
  Serial.print(F("Date "));
  Serial.print(gps.date.day());
  Serial.print(F("/"));
  Serial.print(gps.date.month());
  Serial.print(F("/"));
  Serial.println(gps.date.year());
}


void GPSOutputOn()
{
  GPSserial.begin(GPSBaud);
  while (GPSserial.available()) GPSserial.read();  //make sure input buffer is empty
}


void GPSOutputOff()
{
  //turns off serial output from GPS
  GPSserial.end();
}




//***********************************************************
// End GPS Functions
//***********************************************************


void setup()
{
  uint32_t i;
  uint16_t j;

  Serial.begin(115200);                     //Setup Serial console ouput
  Serial.println();
  Serial.println();
  Serial.println(F("HAB_Balloon_Tracker_Transmitter Starting"));

  memoryStart(Memory_Address);              //setup the memory
  j = readMemoryUint16(addr_ResetCount);
  j++;
  writeMemoryUint16(addr_ResetCount, j);
  j = readMemoryUint16(addr_ResetCount);

  Serial.print(F("TXResets "));
  Serial.println(j);


#ifdef ClearAllMemory
  clearAllMemory();
#endif

  SPI.begin();                              //initialize SPI
  Wire.begin();                             //initialize I2C

  bme280_ext.setI2CAddress(BME280_ADDRESS_EXT); //set the I2C address of the BME280

  if (!bme280_ext.beginI2C())
  {
    Serial.println(F("BME280 not found"));
    while (1)
    {
      led_Flash(50, 50);                     //long fast speed flash indicates device error
    }
  }

  if (!lsm9ds1.begin())
  {
    Serial.println(F("LSM9DS1 not found"));
    while (1)
    {
      led_Flash(50, 50);                     //long fast speed flash indicates device error
    }
  }

  if (!SD.begin(SD_CS))
  {
    Serial.println(F("SD card not found"));
    while (1)
    {
      led_Flash(50, 50);                     //long fast speed flash indicates device error
    }
  }

  sdcard = SD.open("test.txt", FILE_WRITE);
  if (!sdcard)
  {
    Serial.println(F("Error opening file"));
    while (1)
    {
      led_Flash(50, 50);                     //long fast speed flash indicates device error
    }
  }
  sdcard.close();

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("LoRa Device error"));
    while (1)
    {
      led_Flash(50, 50);                     //long fast speed flash indicates device error
    }
  }

  setTrackerMode();

  j = readMemoryUint16(addr_TXErrors);
  Serial.print(F("TXErrors "));
  Serial.println(j);

  Serial.print(F("TXSequence "));
  i = readMemoryUint32(addr_SequenceNum);
  Serial.println(i);

  Serial.println();
  printBatteryCurrent();
  printExtTemp();
  Serial.println();

  TXStatus = 0;                               //clear all TX status bits

  sendCommand(PowerUp);                       //send power up command, includes supply mV and config, on tracker settings

  GPSOutputOn();
  Serial.println();
  Serial.println(F("GPS output test"));
  Serial.flush();
  GPSTest();                                  //copy GPS output to serial monitor as a test

  delay(2000);

  GPSstartms = millis();

  setTrackerMode();                              //so that commands indicating wait for a GPS go out

  while (!gpsWaitFix(5))                         //wait for the initial GPS fix, this could take a while
  {
    sendCommand(NoFix);
    led_Flash(2, 50);                            //two short LED flashes to indicate GPS waiting for fix
  }


  GPSOutputOn();
  delay(2000);                                   //GPS may be in software backup allow time for it to wakeup
  GPSOutputOff();
}
