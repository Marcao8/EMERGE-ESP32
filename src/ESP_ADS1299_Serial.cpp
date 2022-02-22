/**
 * @brief ESP 32 ADS1299 Serial Interface Prototype
 * ToDo:
Run Tasks with RTOS on core 0
1. Establish Serial connection from ESP32 to PC
2. Setup SPI to 1 MHz for ADS com
3. Setup ADS
4. Acquire readings
5. Send to PC
6. Send to Host via UDP or ESPNOW
7. Receive Settings (FIRMAMATA)
 */

#include <Arduino.h>
#include <SPI.h>    //external Library <>
#include "config.h" // Settings
#include "UDPcom.h"
#include "ADS1299.hh" // own Lib

// Instances
LEDparameter ledpara; // define the width of signal or also on time
ADS1299 ADS1;

// Task function declarations
void Task_Blink(void *pvParameters);
void TaskRead_BAT_V(void *pvParameters);

// Are we currently connected?
long outputCount = 0;

void setup()
{
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  Serial.begin(2000000);

  ADS1.setup_master(PIN_NUM_DRDY, PIN_CS_1);
  delay(100);

  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
      Task_Blink // Function to implement the task
      ,"Task Blink" // A name just for humans
      ,1024 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,NULL // Task input parameter
      ,1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,NULL // Task handle.
      ,0); // Core where the task should run
          //  in ARDUINO on ESP32: main() runs on core 1 with priority 1
  xTaskCreatePinnedToCore(
      TaskRead_BAT_V, "ReadBAT_V", 1024 // Stack size
      ,NULL, 3 // Priority
      ,NULL, 0);

    xTaskCreatePinnedToCore(
      (TaskFunction_t) &ADS1299::Task_data, "ADS1_DATA_TASK", 1024 // Stack size
      ,NULL, 2 // Priority
      ,NULL, 1); // core

  connectToWiFi(networkName, networkPswd);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
    while (1)
    {
      delay(1000);
    }
  }
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  delay(10);
  ADS1.getDeviceID();
  ADS1.RREG(CONFIG2);
  ADS1.WREG(CONFIG1, 0xD5);
  // ADS1.WREG(CONFIG3,0xEC); // 6C: power down Buffer | EC: enable Bias
  // ADS1.activateTestSignals(CH4SET); //measure testsignal on CH4
  ADS1.WREG(CH1SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH2SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH3SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH4SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH5SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH6SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH7SET, 0x00); // measures normal on CH1
  ADS1.WREG(CH8SET, 0x00);

  // ADS1.WREG(CH2SET,0x03); //measures MVDD on CH2
  // ADS1.WREG(CH3SET,0x01); //shorted  on CH3
  // ADS1.activateTestSignals(CH4SET); //measure testsignal on CH4
  // ADS1.WREG(CH5SET,0x01); //shorted  on CH5
  // ADS1.WREG(CH6SET,0x01); //shorted  on CH6
  ADS1.WREG(MISC1, 0x10); // connect SRB1 to neg Electrodes

  // Bias activate
  ADS1.WREG(CONFIG3, 0xEC);    // b’x1xx 1100  Turn on BIAS amplifier, set internal BIASREF voltage
  ADS1.WREG(BIAS_SENSN, 0x00); // CH1 n bias sensing
  ADS1.WREG(BIAS_SENSP, 0x0F); // CH1-CH8 p bias sensing

  ADS1.START();
  delay(1000);
}

void loop()
{

  while (digitalRead(PIN_NUM_DRDY) != LOW)
  {
  };
  // when DRDY goes low-> read new data
  ADS1.updateData();
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Task_Blink(void *pvParameters)
// This is a task for pulsating status LED
{
  int brightness = 0; // how bright the LED is
  int fadeAmount = 5;
  (void)pvParameters;
  ledcSetup(ledpara.PWM_CHANNEL, ledpara.PWM_FREQUENCY, ledpara.PWM_RESOUTION);
  ledcAttachPin(ledRed, ledpara.PWM_CHANNEL);

  for (;;) // A Task shall never return or exit.
  {
    ledcWrite(ledpara.PWM_CHANNEL, brightness);
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
    vTaskDelay(100); // one tick delay (15ms) in between reads for stability
  }
}

void TaskRead_BAT_V(void *pvParameters)
// This is a task for serial printing Battery voltage
{
  (void)pvParameters;
  for (;;)
  {
    digitalWrite(ledBlue, HIGH);

    int rawADC = analogRead(BatteryPin);
    float BatteryVoltage = 2 * rawADC * 3.3 / 4095; // Voltage devider *ADC*
    // print out the value you read:
    // Serial.print("Battery Voltage: ");
    // Serial.println(BatteryVoltage);
    digitalWrite(ledBlue, LOW);
    vTaskDelay(1000);
  }
}

void Task_printADSdata(void *pvParameters)
// This is a task for serial printing one ADS1299 reading
{
  static long outputCount = 0;
  (void)pvParameters;
  for (;;)
  {
    if (digitalRead(PIN_NUM_DRDY) == LOW)
    {
      digitalWrite(PIN_CS_1, LOW);
      //        long output[100][9];
      long output[9];
      long dataPacket;
      for (int i = 0; i < 9; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          // byte dataByte = vspi->transfer(0x00);
          // dataPacket = (dataPacket << 8) | dataByte;
        }
        //            output[outputCount][i] = dataPacket;
        output[i] = dataPacket;
        dataPacket = 0;
      }
      digitalWrite(PIN_CS_1, HIGH);
      Serial.print(outputCount);
      Serial.print(", ");
      for (int i = 0; i < 9; i++)
      {
        Serial.print(output[i], HEX);
        if (i != 8)
          Serial.print(", ");
      }
      Serial.println();
      outputCount++;
      vTaskDelay(2000);
    }
  }
}

// TI SETUP https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/634236/ads1298-cannot-get-ecg-signal
void TIsetup()
{ /*
CONFIG1 	0x46
CONFIG2 	0x10
CONFIG3 	0x41
LOFF 	Default
CH1Set 	0x81
CH2Set 	0x60
CH3-8Set 	0x81
*/

  ADS1.RREG(CONFIG2);
  // ADS1.WREG(CONFIG3,0xEC); // 6C: power down Buffer | EC: enable Bias
  ADS1.activateTestSignals(CH4SET); // measure testsignal on CH4
  ADS1.WREG(CH1SET, 0x00);          // measures normal on CH1
  ADS1.WREG(CH2SET, 0x03);          // measures MVDD on CH2
  ADS1.WREG(CH3SET, 0x01);          // shorted  on CH3
  // ADS1.activateTestSignals(CH4SET); //measure testsignal on CH4
  ADS1.WREG(CH5SET, 0x01);     // shorted  on CH5
  ADS1.WREG(CH6SET, 0x01);     // shorted  on CH6
  ADS1.WREG(MISC1, 0x00);      // Not connect SRB1 to neg Electrodes
  ADS1.WREG(BIAS_SENSN, 0x01); // CH1 n bias
  ADS1.START();
  delay(1000);
}
