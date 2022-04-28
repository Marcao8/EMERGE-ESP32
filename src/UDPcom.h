/**
 * @file UDPcom.h
 * @author Markus Bäcker (markus.baecker@ovgu.de)
 * @brief Networking class, Sets up ESP Wifi and connects to host
 * @version 0.1
 * @date 2022-02-21
 * @details Sends 16Channelx4Byte(float)+ 1Byte BatteryV = 68B Payload to Host, Additional 4 Bytes Header per Message, 250 Samples per Second = 17 KB/s
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

extern boolean connected;
extern WiFiUDP udp;
void WiFiEvent(WiFiEvent_t event);
void connectToWiFi(const char * ssid, const char * pwd);
void sendUDP(double data_array1[],double data_array2[]);
void sendUDPbin(float data_array1[],float data_array2[]);
void floatToByte(byte* arr, float value);