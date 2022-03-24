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

void sendUDP(double data_array1[],double data_array2[]){
int buffersize = 410;
char buffer[buffersize];
static int packetloss;
      snprintf(buffer, buffersize, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
               data_array1[1], data_array1[2], data_array1[3], data_array1[4], data_array1[5], data_array1[6], data_array1[7], data_array1[8],
               data_array2[1], data_array2[2], data_array2[3], data_array2[4], data_array2[5], data_array2[6], data_array2[7], data_array2[8], packetloss);
      digitalWrite(12,HIGH);
      digitalWrite(12,LOW);
      udp.beginPacket(udpAddress, udpPort);
      udp.printf(buffer);
      udp.endPacket();
      packetloss++;
 /* if (Serial.available())
      {
        Serial.println(buffer);
        // empty the RX buffer, so that only one ADS reading will be putput per incoming message
        byte dump = Serial.read();
      }
      */
}

void sendUDPbin(float data_array1[],float data_array2[]){

udp.beginPacket(udpAddress, udpPort);  
//udp.write(data_array1);
udp.endPacket();
}

void floatToByte(byte* arr, float value)
{
      long l = *(long*) &value;

      arr[0] = l & 0x00FF;
      arr[1] = (l >> 8) & 0x00FF;
      arr[2] = (l >> 16) & 0x00FF;
      arr[3] = l >> 24;
}

/*
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
      
      
     //udp.endPacket();
     

}
*/