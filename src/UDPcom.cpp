#include "UDPcom.h"

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

//Connect to the WiFi network
 void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  
  //register event handler
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_17dBm); //2 dBm before
  WiFi.setSleep(false); //latency improvement
    //Initiate connection
  

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}

void sendUDP(char data, int method){
   const char buffer = data;
    //udp.beginPacket(udpAddress,udpPort);
    //udp.print(buffer);
    // CLIENT: 

sockaddr_in _send_addr;
_send_addr.sin_family         = AF_INET,
_send_addr.sin_port           = htons( udpPort );
// also tried local broadcast 192.168.4.255
_send_addr.sin_addr.s_addr    = inet_addr( udpAddress ); 
_send_addr.sin_len            = sizeof( _send_addr );


int _sock;
_sock = socket( AF_INET, SOCK_DGRAM, IPPROTO_IP );

int broadcast = 1;
setsockopt( _sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast) );

//sendto( _sock, buffer, 180, 0, (const struct sockaddr*) &_send_addr.sin_addr.s_addr,
  // sizeof(_send_addr) );
/*
      s = socket(AF_INET, SOCK_DGRAM, 0);
      sendto(s,*buffer,size,flags,to,tolen);
      udp.beginPacket(udpAddress, udpPort);
      int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
    /
    for (int i = 0; i < strlen(buffer); i++)
      {   udp.write(buffer[i]);   }
      // Send a packet ~ 1ms
     udp.beginPacket(udpAddress, udpPort);
      udp.printf("%f,%f,%f,%f,%f,%f,%f,%f,%d\n", results_mV[1], results_mV[2], results_mV[3], results_mV[4], results_mV[5], results_mV[6], results_mV[7], results_mV[8], packetloss);
      
      */
     //udp.endPacket();
     

}