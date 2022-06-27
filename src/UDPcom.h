/**
 * @file UDPcom.h
 * @author Markus Bäcker (markus.baecker@ovgu.de)
 * @brief Networking class, Sets up ESP Wifi and connects to host
 * @version 0.1
 * @date 2022-02-21
 * @details Sends 16Channel + UDP number + BatteryV as Payload to Host
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include "credentials.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <esp_pm.h>
#include <esp_wifi_types.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define DESIRED_ADDRESS udpAddress
#define DESIRED_PORT udpPort
#define BUFSIZE 512

#define WIFI_TIMEOUT_MS 20000 // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

extern boolean connected;
extern WiFiUDP udp;
extern float BatteryVoltage;


void WiFiEvent(WiFiEvent_t event);
void WiFiScan();
void connectToWiFi(const char * ssid, const char * pwd);
void sendUDP(double data_array1[],double data_array2[]);
void TaskWiFialive(void * parameter);


/* Initialize credentials like:
 static const char ssid[4][16] = { "Network1", "Network2", "3",
                           "4" };
    static const char pswd[4][16] = { "Password1", "Password2", "Password3",
                           "Password4" };
    static const char hostip[4][16] = { "192.168.137.1", "192.168.178.40", "192.168.2.60",
                           "192.168.1.1" }; 
const int   known_ssid_count = sizeof(ssid) / sizeof(ssid[0]); // number of known networks
static int found_ssid = 0;
void sendUDPbin(float data_array1[],float data_array2[]);
void floatToByte(byte* arr, float value);
*/