/**
 * @file ADS1299.hh
 * @author Markus Bäcker (markus.baecker@ovgu.de)
 * @brief Driver class for TI ADS1299 in combination with ESP32-S1
 * @version 0.1
 * @date 2022-02-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ____ADS1299__
#define ____ADS1299__

#include <Arduino.h>
#include <SPI.h>
#include "config.h"
// Wireless data transmission
#include "UDPcom.h"




class ADS1299 {
public:
  ADS1299();
  //
  //Thread_DRDY(){
    // xTaskCreatePinnedToCore(
   // Task_read_DRDY, "read_DRDY", 1024, NULL, 3 , NULL, 0);
  //}

  //Attributes
  int DRDY; 
  int CS;  //pin numbers for "Data Ready" (DRDY) and "Chip Select" CS (Datasheet, pg. 26)
  long outputCount; // For packet loss testing
  int stat_1, stat_2;    // used to hold the status register for boards 1 and 2
  unsigned short int ChGain[8]; // gain for each channel
  
  void setup_master(int _DRDY, int _CS);
  void setup_slave(int _DRDY, int _CS);
  
  //ADS1299 SPI Command Definitions (Datasheet, Pg. 35)
  //System Commands
  void WAKEUP();
  void STANDBY();
  void RESET();
  void START();
  void STOP();

  //Data Read Commands
  void RDATAC();  // Reads data from ADC Channels continious
  void SDATAC();  // Stops continous reading
  void RDATA();   // Read one

  //Register Read/Write Commands
  byte getDeviceID();
  byte RREG(byte _address);
  //void RREG(byte _address, byte _numRegistersMinusOne);  //to read multiple consecutive registers (Datasheet, pg. 38)

  void printRegisterName(byte _address);

  void WREG(byte _address, byte _value);                              //
  void WREG(byte _address, byte _value, byte _numRegistersMinusOne);  //

  void updateData();
  

  //SPI Arduino Library Stuff
  byte transfer(byte _data);

  //------------------------//
  void activateTestSignals(byte _channeladdress);  // Activate a (1 mV x V REF / 2.4) Square-Wave Test Signal on all Channels
  float convertHEXtoVolt(long hexdata);  //convert Data Bytes to float Voltage values
  void attachInterrupt();
  void detachInterrupt();  // Default
  void begin();            // Default
  void end();
  void setBitOrder(uint8_t);
  void setDataMode(uint8_t);
  void setClockDivider(uint8_t);
  void calculateLSB(uint8_t gain, float vref);
  void setSingleended();
  //------------------------//
  void TI_setup(); // for Debugging purpose

  void Task_data(void const * argument);
  TaskHandle_t				task_handle;

private:
  // uninitalised pointers to SPI objects
  SPIClass *vspi = NULL;
  
  static void Task_read_DRDY(){}; // Task called by RTOS 
  float LSB; // unit for Voltage conversion
  byte regData [24];	// array is used to mirror register data
  long channelData [16];	// array used when reading channel data board 1+2
  String UDPstring;
  int packetloss;
};

#endif