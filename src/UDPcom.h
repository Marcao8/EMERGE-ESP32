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

extern boolean connected;
extern WiFiUDP udp;
void WiFiEvent(WiFiEvent_t event);
void connectToWiFi(const char * ssid, const char * pwd);
