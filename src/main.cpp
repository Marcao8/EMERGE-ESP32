/**
 * @file ADS1299.cpp
 * @author Markus Bäcker (markus.baecker@ovgu.de)
 * @brief EMERGE ECG main file
 * @version 0.1
 * @date 2022-03-02
 * @note
 * @copyright Copyright (c) 2022
 *  @note Run Tasks with RTOS on core 0
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

ADS1299 ADS1; //AUFSEHER:IN
ADS1299 ADS2; //HILFELEISTENDE:R


// Task function declarations
void Task_Blink(void *pvParameters);
void TaskRead_BAT_V(void *pvParameters);
void IRAM_ATTR DRDY_ISR();

// Are we currently connected?
long outputCount = 0;
bool data_ready = false;
void setup()
{
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(12,OUTPUT);
  digitalWrite(ledBlue, 1);
  digitalWrite(ledRed, 1);
  Serial.begin(2000000);

  ADS1.setup_master(PIN_NUM_DRDY_1, PIN_CS_1);

  // ADS2 = new ADS1299;
  ADS2.setup_slave(PIN_NUM_DRDY_2, PIN_CS_2);

  // Errorcheck ADS responding
  if (ADS1.getDeviceID() != 0b00111110)
  {
    Serial.println(F("Ops! ADS1299 controller not found"));
  }
  else if (ADS2.getDeviceID() != 0b00111110)
  {
    Serial.println(F("Ops! ADS1299 responder not found"));
  }
  else
  {
    Serial.println(F("Both ADS1299 responding correctly"));
  }

  delay(100); // wait for things to settle down
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
      Task_Blink // Function to implement the task
      ,
      "Task Blink" // A name just for humans
      ,
      1024 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL // Task input parameter
      ,
      1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL // Task handle.
      ,
      1); // Core where the task should run
  //  in ARDUINO on ESP32: main() runs on core 1 with priority 1, higher number= higher prio
  xTaskCreatePinnedToCore(
      TaskRead_BAT_V, "ReadBAT_V", 1024 // Stack size
      ,
      NULL, 1 // Priority
      ,
      NULL, 0);

  // xTaskCreatePinnedToCore(
  //   (TaskFunction_t) &ADS1299::Task_data, "ADS1_DATA_TASK", 1024 // Stack size
  //   ,NULL, 1 // Priority
  //   ,NULL, 0); // core

  connectToWiFi(networkName, networkPswd);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
    while (1)
    {
      delay(1000);
    } 
  }
  digitalWrite(ledRed, 0);
  ADS1.setSingleended();
  //ADS2.setSingleended();
ADS2.activateTestSignals(CH1SET);
ADS2.activateTestSignals(CH2SET);
ADS2.activateTestSignals(CH3SET);
ADS2.activateTestSignals(CH4SET);
ADS2.activateTestSignals(CH5SET);
ADS2.activateTestSignals(CH6SET);
ADS2.activateTestSignals(CH7SET);
ADS2.activateTestSignals(CH8SET);
   //ADS1.START();
   digitalWrite(PIN_NUM_STRT,HIGH);
   ADS1.RDATAC();
   delayMicroseconds(20 * TCLK_cycle);
   ADS2.RDATAC();
  

  Serial.printf("ADS1 CS: %d ; DRDY %d \n", ADS1.CS, ADS1.DRDY);
  Serial.printf("ADS2 CS: %d ; DRDY %d \n", ADS2.CS, ADS2.DRDY);
ADS1.START();
  //interrupt for DRDY
  attachInterrupt(PIN_NUM_DRDY_1, DRDY_ISR, FALLING);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{

  if (data_ready == true)
  { digitalWrite(12,LOW);
    float mydata2[9];
    float mydata1[9];
    float* array; 
    array = ADS1.updateData();
    for (int i = 0; i < 9; i++) {
    mydata1[i] = array[i];}

    array = ADS2.updateData();
    for (int i = 0; i < 9; i++) {
    mydata2[i] = array[i];}

    sendUDP(mydata1, mydata2);
    data_ready = false;
    digitalWrite(12,HIGH);
  }

  // when DRDY goes low-> read new data
}

/*------------------------
    // only for debugging purposes
 int newData = digitalRead(PIN_NUM_DRDY_2);
 static int lastnewData = 0; //remains over current loop
  
  if (newData != lastnewData)
  {
    //delay(100); 
    if (newData == LOW)
    {
      
    } 
    else{ //do nothing
    }
  }
   lastnewData  = newData;
--------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void IRAM_ATTR DRDY_ISR()
{
  data_ready = true;
}


void Task_Blink(void *pvParameters)
{ // This is a task for pulsating status LED
  int brightness = 0; // how bright the LED is
  int fadeAmount = 15;
  (void)pvParameters;
  ledcSetup(ledpara.PWM_CHANNEL, ledpara.PWM_FREQUENCY, ledpara.PWM_RESOUTION);
  ledcAttachPin(ledGreen, ledpara.PWM_CHANNEL);

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
    vTaskDelay(300); // one tick delay (15ms) in between reads for stability
  }
}

void TaskRead_BAT_V(void *pvParameters){
  // This is a task for serial printing Battery voltage
  (void)pvParameters;
  for (;;)
  {

    int rawADC = analogRead(BatteryPin);
    float BatteryVoltage = 2 * rawADC * 3.3 / 4095; // Voltage devider *ADC*
                                                    // print out the value you read:
    Serial.print("Battery Voltage: ");
    Serial.println(BatteryVoltage);

    vTaskDelay(600000); // every minute
  }
}

