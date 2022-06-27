/**
 * @file main.cpp
 * @author Markus Baecker (markus.baecker(at)ovgu.de)
 * @brief EMERGE ECG main file, part of research project EMERGE at Otto-von-Guericke Universtity, Germany
 * @version 0.1
 * @date 2022-03-02
 * @copyright Copyright (c) 2022
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include <Arduino.h>
#include <SPI.h> //external Lib <>

#include "config.h"   // Settings
#include "UDPcom.h"   // own Lib
#include "ADS1299.hh" // own Lib

// Instances
LEDparameter ledpara; // define LED parameters
ADS1299 ADS1;
ADS1299 ADS2;
// Task function declarations
void Task_Blink(void *pvParameters);
void TaskRead_BAT_V(void *pvParameters);
void Task_DataProcess(void *pvParameters);
void IRAM_ATTR DRDY_ISR(void);
void ESPmemcheck();
void ADSerrorcheck();
// Global variables
bool ADS_connected = false;
bool data_ready = false;
float BatteryVoltage = 0.0;
int StatusLED = ledBlue;
/* Stores the handle of the task that will be notified when the interrupt is called by ADS1299 */
TaskHandle_t DataTaskHandle;

void setup()
{
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  // pinMode(12,OUTPUT);
  digitalWrite(ledBlue, 1);
  digitalWrite(ledRed, 1);
  Serial.begin(115200);

  ADS1.setup_master(PIN_NUM_DRDY_1, PIN_CS_1);
  delay(10); // wait for clock to settle
  ADS2.setup_slave(PIN_NUM_DRDY_2, PIN_CS_2);

  ESPmemcheck();
  ADSerrorcheck();

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

  WiFiScan();
  // connectToWiFi(networkName, networkPswd);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    while (1)
    {
      delay(1000);
    }
    // StatusLED = ledRed;
  }
  Serial.println("connected");
  // StatusLED = ledBlue;
  digitalWrite(ledRed, 0);
  ADS1.setSingleended();
  ADS2.setSingleended();

  ADS2.activateTestSignals(CH5SET);
  ADS2.activateTestSignals(CH6SET);
  ADS2.activateTestSignals(CH7SET);
  ADS2.activateTestSignals(CH8SET);

  ADS1.WREG(CONFIG3, 0xEC);         // Bias on
  digitalWrite(PIN_NUM_STRT, HIGH); // Synchronize Start of ADC's
  ADS1.RDATAC();
  ADS2.RDATAC();

  // interrupt for DRDY
  attachInterrupt(PIN_NUM_DRDY_1, DRDY_ISR, FALLING);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  if (data_ready == true)
  {
    digitalWrite(12, LOW);
    results ADS_1;
    ADS_1 = ADS1.updateResponder();
    results ADS_2;
    if (BOARD_V3 == 1)
    {
      ADS_2 = {0, 0, 0, 0, 0, 0, 0, 0}; // ADS2.updateResponder();
    }
    if (BOARD_V3 == 0)
    {
      ADS_2 = ADS2.updateResponder();
    }

    sendUDP(ADS_1.mVresults, ADS_2.mVresults);
    data_ready = false;
    digitalWrite(12, HIGH);
  }
  if (ADS_connected == false)
  {
    digitalWrite(12, LOW);
    results ADS_1;
    ADS_1 = {0, 0, 0, 0, 0, 0, 0, 0};
    results ADS_2;

    ADS_2 = {0, 0, 0, 0, 0, 0, 0, 0};

    sendUDP(ADS_1.mVresults, ADS_2.mVresults);

    digitalWrite(12, HIGH);
    delay(10);
  }

  // when DRDY goes low-> read new data
}

/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void IRAM_ATTR DRDY_ISR(void)
{
  data_ready = true;
  /* Wake up DataTask  */
  // xTaskResumeFromISR( DataTaskHandle);
}

void Task_DataProcess(void *pvParameters)
{
  while (1)
  {
    digitalWrite(12, LOW);
    results ADS_1;
    ADS_1 = ADS1.updateResponder();
    results ADS_2;
    if (ADS_connected == false)
    {
      ADS_2 = {0, 0, 0, 0, 0, 0, 0, 0}; // ADS2.updateResponder();
    }
    if (BOARD_V3 == 0)
    {
      ADS_2 = ADS2.updateResponder();
    }

    sendUDP(ADS_1.mVresults, ADS_2.mVresults);

    digitalWrite(12, HIGH);

    vTaskSuspend(NULL); // wait until rewoked by interrupt
  }
  // vTaskDelete(NULL); // delete own Task, never called
}

void Task_Blink(void *pvParameters)
{                     // This is a task for pulsating status LED
  int brightness = 0; // how bright the LED is
  int fadeAmount = 15;
  //(void)pvParameters;
  ledcSetup(ledpara.PWM_CHANNEL, ledpara.PWM_FREQUENCY, ledpara.PWM_RESOUTION);
  ledcAttachPin(StatusLED, ledpara.PWM_CHANNEL);

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
    vTaskDelay(300 / portTICK_PERIOD_MS); // one tick delay (15ms) in between reads for stability
  }
}

void TaskRead_BAT_V(void *pvParameters)
{
  // This is a task for serial printing Battery voltage
  (void)pvParameters;
  for (;;)
  {

    int rawADC = analogRead(BatteryPin);
    BatteryVoltage = 2 * rawADC * 3.3 / 4095; // Voltage devider *ADC*
                                              // print out the value you read:

    Serial.print("Battery Voltage: ");
    Serial.println(BatteryVoltage);
    // check for Low voltage
    if (BatteryVoltage < 3.2)
    {
      digitalWrite(ledRed, HIGH);
    }

    vTaskDelay(60000 / portTICK_PERIOD_MS); // every minute
  }
}

void ESPmemcheck()
{
  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Minimum free heap: %d", ESP.getMinFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
}

void ADSerrorcheck()
{
  if (ADS1.getDeviceID() != 0b00111110)
  {
    log_d("ADS1299 Nr.1 not found... restart \n");
    delay(500);
    // ESP.restart();
    ADS1.setup_master(PIN_NUM_DRDY_1, PIN_CS_1);
    ADS2.setup_slave(PIN_NUM_DRDY_2, PIN_CS_2);
    // StatusLED = ledRed;
  }
  if (ADS2.getDeviceID() != 0b00111110)
  {
    log_d("ADS1299 Nr.2 not found... restart \n");
    delay(1000);
    // ESP.restart();
    ADS1.setup_master(PIN_NUM_DRDY_1, PIN_CS_1);
    ADS2.setup_slave(PIN_NUM_DRDY_2, PIN_CS_2);
    // StatusLED = ledRed;
  }
  else
  {
    ADS_connected = true;
  }
}
