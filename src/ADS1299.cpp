/*
    Class for interfacing ADS1299
    @file ADS1299.cpp
    @author Markus Baecker
    @version 11/2021
*/

#include "ADS1299.hh"

/**
 * @brief Construct a new ADS1299::ADS1299 object
 * 
 */
ADS1299::ADS1299()
{ // Standard constructor for initial definitions
  for (int i = 0; i < sizeof ChGain / sizeof ChGain[0]; ++i)//quirky way to get length of array from size
  {
    ChGain[i] = GAIN;  // initial Gain for all channels
  }                                                                     
  LSB = ((2 * 4.5) / GAIN) / (16777216 - 1); // 2*Vref+-/GAIN/(2^24bits -1)
  Serial.println("ADS constructed");
  packetloss = 0;
  // **** ----- SPI Setup ----- **** //
  // initialise instance of the SPIClass attached to VSPI
  vspi = new SPIClass(VSPI);
}

/// <summary>Configures SPI and ADS1299 for</summary>
/// <param name="_DRDY">Pin which goes HIGH when data is ready to be read by MCU</param>
// <param name="_CS">Pin which goes LOW when MCU wants to talk to specified ADS1299</param>
/// <returns>Nothing</returns>

/**
 * @brief Sets up master ADS1299, configures SPI and ADS1299 for
 * @details supply slave ADS1299 with clock and BIAS
 * @param _DRDY DATA Ready pin goes LOW, when DATA is available
 * @param _CS  CHIP SELECT pin, goes LOW, when MCU communicates with ADS
 */
void ADS1299::setup_master(int _DRDY, int _CS)
{
  CS = _CS;
  DRDY = _DRDY;
  
  // Set direction register for SCK and MOSI pin.
  // MISO pin automatically overrides to INPUT.
  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  // initialise vspi with default pins
  // SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(PIN_NUM_RST, OUTPUT);
  pinMode(PIN_NUM_STRT, OUTPUT);
  pinMode(PIN_NUM_PWD, OUTPUT);
  pinMode(PIN_NUM_DRDY, INPUT);
  // set up slave select pins as outputs as the Arduino API
  // doesn't handle automatically pulling SS low
  pinMode(VSPI_SS, OUTPUT); // VSPI SS

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); // 1...2.4 MHz, clock polarity = 0; clock phase = 1 (pg. 8)

  digitalWrite(SCK, LOW);
  digitalWrite(MOSI, LOW);
  digitalWrite(SS, HIGH);

  // at startup all inputs must be low
  digitalWrite(PIN_NUM_RST, LOW);
  digitalWrite(PIN_NUM_STRT, LOW);
  // LDO power down
  digitalWrite(PIN_NUM_PWD, LOW);
  delay(10); // wait for oscillator startup 20us
  digitalWrite(PIN_NUM_PWD, HIGH);
  // at startup all inputs must be low
  digitalWrite(PIN_NUM_RST, HIGH);
  delayMicroseconds(20 * TCLK_cycle); // Recommended 18 Tclk before using device
  SDATAC();                           // DEVICE wakes up in RDATAC so no Registers could be written.
}

// ADS1299 SPI Command Definitions (Datasheet, Pg. 35)
/*--------------------------------------------------*/
/*---------------------- System Commands ---------------------*/
/*--------------------------------------------------*/

/**
 * @brief Wakes up the ADC
 * 
 */
void ADS1299::WAKEUP()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_WAKEUP);
  digitalWrite(CS, HIGH);
  delayMicroseconds(4 * TCLK_cycle); // wait 4 Tclk (Pg. 35)
}
/**
 * @brief Put the ADC in Standby
 *  5.1 mW instead of 22 mW in running mode
 */
void ADS1299::STANDBY()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_STANDBY);
  digitalWrite(CS, HIGH);
}

/**
 * Reset all Registers
 * sends command 0x06
 * @return nothing
 */
void ADS1299::RESET()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_RESET);
  digitalWrite(CS, HIGH);
  delayMicroseconds(18 * TCLK_cycle); // Recommended 18 Tclk before using device
}
/**
 * @brief Send SPI command or pull START pin LOW, sync multiple ADS
 * 
 */
void ADS1299::START()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_START);
  digitalWrite(PIN_NUM_STRT, HIGH);
  delayMicroseconds(4 * TCLK_cycle); // wait 4 clk cycles after this command (DS pg.36)
  digitalWrite(CS, HIGH);
}
/**
 * @brief STOP data conversion, allows REGISTER reading/writing
 * 
 */
void ADS1299::STOP()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_STOP);
  digitalWrite(PIN_NUM_STRT, LOW);
  delayMicroseconds(4 * TCLK_cycle); // wait 4 clk cycles after this command (DS pg.36)
  digitalWrite(CS, HIGH);
}
/**
 * @brief Read one data chunk from ADS. Transmits 0001 0010 (12h)
 */
void ADS1299::RDATA()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_RDATA);
  digitalWrite(CS, HIGH);
}

/**
 * @brief Read Data Continuously mode. Transmits 0001 0000 (10h)
 * Reads data from ADC Channels continious
 * @return nothing
 */
void ADS1299::RDATAC()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_RDATAC);
  digitalWrite(CS, HIGH);
}

/**
 * @brief Stop Read Data Continuously mode. Transmits 0001 0001 (11h)
 * @return nothing
 */
void ADS1299::SDATAC()
{
  digitalWrite(CS, LOW);
  vspi->transfer(_SDATAC);
  digitalWrite(CS, HIGH);
  delayMicroseconds(4 * TCLK_cycle); // wait 4 clk cycles after this command (DS pg.36)
}

/*--------------------------------------------------*/
/*---------------------- Register Commands ---------------------*/
/*--------------------------------------------------*/

/**
 * @brief Read Register from ADS1299.
 * It answers with the contained values, call SDATAC before reading/writing registers
 * @param _address Single Register adress to read
 */
byte ADS1299::RREG(byte _address)
{                                           //  reads ONE register at _address
  byte opcode1 = _address + 0x20;           //  RREG expects 001rrrrr where rrrrr = _address
  digitalWrite(CS, LOW);                    //  open SPI
  vspi->transfer(opcode1);                  //  opcode1
  vspi->transfer(0x00);                     //  opcode2
  regData[_address] = vspi->transfer(0x00); //  update mirror location with returned byte
  digitalWrite(CS, HIGH);                   //  close SPI
                                            // Serial.println(regData[_address]);
  return regData[_address];                 // return requested register value
}

/**
 * @brief Write Settings as 1 Byte of Data to Registers
 * 
 * @param _address of Regsiter
 * @param _value of Register
 */
void ADS1299::WREG(byte _address, byte _value)
{
  // write one Register
  uint8_t opcode1 = _address + 0x40;
  // Send WREG command & address
  vspi->transfer(opcode1);
  // Send number of registers to write
  vspi->transfer(0x00);
  // Write the value to the register
  vspi->transfer(_value);

} //

/*
void ADS1299::WREG(byte _address, byte _value, byte _numRegistersMinusOne) {
  //overloaded version for multiple registers

}  //
*/

/*--------------------------------------------------*/
/*----------------------  Utility Functions ---------------------*/
/*--------------------------------------------------*/
/**
 * @brief Sets the adjecent Voltage value of the smallest bit of a measurement
 * 
 * @param gain currently used Gain value for channel
 * @param vref currently used Voltage reference
 */
void ADS1299::calculateLSB(uint8_t gain, float vref)
{ // Gain as int, vref in mV
  // LSB = (vref) / gain / (16777216 - 1);             // 16.777.216 = 2^24 <-Analog | 2^23 = 8.388.608
  LSB = 1000 * (vref) / (8388608);
}

/**
 * @brief Converts hexadecimal output of ADS to voltage value
 * 
 * @param hexdata incoming 24 bit value
 * @return float output signed voltage in mV (LSB is ~nV)
 */
float ADS1299::convertHEXtoVolt(long hexdata)
{
  int sign = hexdata & 0x800000; // AND data with MSB sign; from 8 to F is -
  float voltage = 0.0;
  if (sign == 0)
  {
    voltage = hexdata * LSB;
  } // positive range
  else
  { // 2^24 negative range
    voltage = (hexdata - 0xFFFFFF) * LSB;
  }
  return voltage;
}

void ADS1299::getDeviceID()
{
  digitalWrite(CS, LOW); // Low to communicated

  vspi->transfer(_SDATAC);          // 0001 0001 SDATAC Stop Data continous
  vspi->transfer(_RREG);            // 0010 0000 RREG Read Register
  vspi->transfer(0x00);             // 0000 0000 Asking for 1 byte ID
  byte data = vspi->transfer(0x00); // byte to read (hopefully 0b???11110) should be 3E or 62
  // Device ID you should get:                    REV_ID[2:0] 1 DEV_ID[1:0] NU_CH[1:0]
  digitalWrite(CS, HIGH); // HIGH to stop communication
}

void ADS1299::activateTestSignals(byte _channeladdress)
{
  SDATAC();            // 0001 0001 Stop Data reading, to write new settings
  WREG(CONFIG3, 0xE0); // internal Reference Voltage 1110 0000 no bias
  WREG(CONFIG1, 0xD6); // 0x96= 1001 0110 Daisy En 250 SPS |
  WREG(CONFIG2, 0xD0); // Config2: 0100(x40)0010(x02)->x00-> 11010000(D0) internal test signal
  // 0xD5 for faster higher test signal
  WREG(_channeladdress, 0x05); // CHnSET: 0100 0101 Setting: 00000101 no PGA, just activate a (1 mV x V REF / 2.4) Square-Wave Test Signal
  // RDATAC();             // read data continous
}

void ADS1299::setSingleended()
{
  WREG(CONFIG1, 0xD5);
  WREG(CH1SET, 0x00); // measures normal on CH1
  WREG(CH2SET, 0x00); // measures normal on CH1
  WREG(CH3SET, 0x00); // measures normal on CH1
  WREG(CH4SET, 0x00); // measures normal on CH1
  WREG(CH5SET, 0x00); // measures normal on CH1
  WREG(CH6SET, 0x00); // measures normal on CH1
  WREG(CH7SET, 0x00); // measures normal on CH1
  WREG(CH8SET, 0x00);

  WREG(MISC1, 0x10); // connect SRB1 to neg Electrodes
  // Bias activate
  WREG(CONFIG3, 0xEC);    // b’x1xx 1100  Turn on BIAS amplifier, set internal BIASREF voltage
  WREG(BIAS_SENSN, 0x0F); // CH1-4 negative bias sensing
  WREG(BIAS_SENSP, 0x0F); // CH1-CH4 pos bias sensing
}

void ADS1299::updateData()
{
  calculateLSB(1, 4.5); // 4.5 Vref set
  if (digitalRead(PIN_NUM_STRT == HIGH))
  { // read only if data can be available
    if (digitalRead(PIN_NUM_DRDY) == LOW)
    {
      digitalWrite(CS, LOW);
      RDATA();

      long output[9];
      long dataPacket;
      float results_mV[9];
      for (int i = 0; i < 9; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          byte dataByte = vspi->transfer(0x00);      // issue 3 SCLK cycles to read 24 bit
          dataPacket = (dataPacket << 8) | dataByte; // left shift each Byte
        }
        output[i] = dataPacket; // save 9 entries, 3 Bytes each
        dataPacket = 0;
      }
      digitalWrite(CS, HIGH);

      // Serial.print(", ");
      for (int i = 1; i < 9; i++)
      { // exclude ADS status bits at [0]
        // Serial.print(output[i], DEC);
        results_mV[i] = convertHEXtoVolt(output[i]); // convertHEXtoVolt()
        if (Serial.available())
        {
          Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%d\n", results_mV[1], results_mV[2], results_mV[3], results_mV[4], results_mV[5], results_mV[6], results_mV[7], results_mV[8], packetloss);
          // Serial.print(results_mV[i],8);
          // if (i != 8) Serial.print(",");
          // if (i=8) Serial.println();
        }
      }

      // Send a packet ~ 1ms
      udp.beginPacket(udpAddress, udpPort);
      udp.printf("%f,%f,%f,%f,%f,%f,%f,%f,%d\n", results_mV[1], results_mV[2], results_mV[3], results_mV[4], results_mV[5], results_mV[6], results_mV[7], results_mV[8], packetloss);
      udp.endPacket();
      packetloss++;
      Serial.println();
    }
  }
}

/**
 * Translate received Bytes from ADS Settings to human readible format
 */
void ADS1299::printRegisterName(byte _address)
{
  if (_address == ID)
  {
    Serial.print("ID, ");
  }
  else if (_address == CONFIG1)
  {
    Serial.print("CONFIG1, ");
  }
  else if (_address == CONFIG2)
  {
    Serial.print("CONFIG2, ");
  }
  else if (_address == CONFIG3)
  {
    Serial.print("CONFIG3, ");
  }
  else if (_address == LOFF)
  {
    Serial.print("LOFF, ");
  }
  else if (_address == CH1SET)
  {
    Serial.print("CH1SET, ");
  }
  else if (_address == CH2SET)
  {
    Serial.print("CH2SET, ");
  }
  else if (_address == CH3SET)
  {
    Serial.print("CH3SET, ");
  }
  else if (_address == CH4SET)
  {
    Serial.print("CH4SET, ");
  }
  else if (_address == CH5SET)
  {
    Serial.print("CH5SET, ");
  }
  else if (_address == CH6SET)
  {
    Serial.print("CH6SET, ");
  }
  else if (_address == CH7SET)
  {
    Serial.print("CH7SET, ");
  }
  else if (_address == CH8SET)
  {
    Serial.print("CH8SET, ");
  }
  else if (_address == BIAS_SENSP)
  {
    Serial.print("BIAS_SENSP, ");
  }
  else if (_address == BIAS_SENSN)
  {
    Serial.print("BIAS_SENSN, ");
  }
  else if (_address == LOFF_SENSP)
  {
    Serial.print("LOFF_SENSP, ");
  }
  else if (_address == LOFF_SENSN)
  {
    Serial.print("LOFF_SENSN, ");
  }
  else if (_address == LOFF_FLIP)
  {
    Serial.print("LOFF_FLIP, ");
  }
  else if (_address == LOFF_STATP)
  {
    Serial.print("LOFF_STATP, ");
  }
  else if (_address == LOFF_STATN)
  {
    Serial.print("LOFF_STATN, ");
  }
  else if (_address == GPIO)
  {
    Serial.print("GPIO, ");
  }
  else if (_address == MISC1)
  {
    Serial.print("MISC1, ");
  }
  else if (_address == MISC2)
  {
    Serial.print("MISC2, ");
  }
  else if (_address == CONFIG4)
  {
    Serial.print("CONFIG4, ");
  }
}

/*

    //------------------------//
    void ADS1299::attachInterrupt(){}
    void ADS1299::detachInterrupt(){} // Default
    void ADS1299::begin(){} // Default
    void ADS1299::end(){}
    void ADS1299::setBitOrder(uint8_t){}
    void ADS1299::setDataMode(uint8_t){}
    void ADS1299::setClockDivider(uint8_t){}
    //------------------------//
    */
