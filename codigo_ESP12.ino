
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>

#define WIFI_SSID "SEITA"         //SSID do wifi
#define WIFI_PASS "zns2g4sev"         //Senha do Wifi
#define MQTT_SERV "io.adafruit.com"   //Servidor do adafruit IO
#define MQTT_PORT 1883                
#define MQTT_NAME "danielhm"      //Nome do usuário adafruit.io
#define MQTT_PASS "e384f662b94d47ff8d1e756b67301f95"        //chave de ativação do feed do adafruit.io

WiFiClient client;                       //Set up MQTT and WiFi clients
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);
Adafruit_MQTT_Subscribe liga = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/feeds/onoff"); //Set up the feed you're subscribing to. Here our feed name is som
Adafruit_MQTT_Publish crianca = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/ss"); //Set up the feed you're subscribing to. Here our feed name is som
Adafruit_MQTT_Publish invasion = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/infra"); //Set up the feed you're subscribing to. Here our feed name is som

int pinSom = 16;
int readSom = 5;
int pinIR = 4;
int readIR = 14;

void setup()
{
  Serial.begin(115200);
  Serial.print("\n\nConnecting Wifi... ");               //Conectar ao WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
  }
  Serial.println("OK!");
  Serial.println(WiFi.localIP());
  mqtt.subscribe(&liga);
  pinMode(pinSom,OUTPUT);
  digitalWrite(pinSom,LOW);
  pinMode(readSom,INPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  MQTT_connect();
  Adafruit_MQTT_Subscribe * subscription;

  //laço que ler os subscribes
  while((subscription = mqtt.readSubscription(400))){
    if(subscription == &liga){
      Serial.print("SOM: ");
      Serial.println((char*) liga.lastread);
      if(!strcmp((char*) liga.lastread, "SSON")){
        digitalWrite(pinSom, HIGH);
      } else if(!strcmp((char*) liga.lastread, "SSOFF")) {
        digitalWrite(pinSom, LOW);
      }
    }
  }

  if(digitalRead(pinSom)){
    int Digital;    
    Digital = digitalRead (readSom);      
    //... and outputted here   
    if(Digital==1){
      Serial.println ("Extreme value: reached");
        crianca.publish(Digital);
    } else {
      Serial.println ("Extreme value: not reached");
    }
    if (!digitalRead(readIR)) {
      // print() & println() can't handle printing long longs. (uint64_t)
      Serial.println(1);
      invasion.publish(1);
    }
    delay (10);
  }
  
  if(!mqtt.ping()){
    mqtt.disconnect();
  }
}

//Função de conexão com o MQTT
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()){                                   // Stop if already connected.
    return;
  }
  Serial.print("\nConnecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0){                     // connect will return 0 for connected 
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);                                       // wait 5 seconds
       retries--;
       //if (retries == 0){
         //while (1);                                       // basically die and wait for WDT to reset me
       //}
  }
  Serial.println("MQTT Connected!");
}
