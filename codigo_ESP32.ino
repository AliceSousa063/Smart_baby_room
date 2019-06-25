
//Bibliotecas para conexão com internet e ao MQTT
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

//#include <iostream.h>
#include <string.h>

//Bibliotecas para o LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <WiFi.h>
#include <time.h>

LiquidCrystal_I2C lcd(0x3F,20,4);


#define WIFI_SSID "SEITA"         //SSID do wifi
#define WIFI_PASS "zns2g4sev"         //Senha do Wifi
#define MQTT_SERV "io.adafruit.com"   //Servidor do adafruit IO
#define MQTT_PORT 1883                
#define MQTT_NAME "danielhm"      //Nome do usuário adafruit.io
#define MQTT_PASS "e384f662b94d47ff8d1e756b67301f95"        //chave de ativação do feed do adafruit.io

//WiFiClientSecure client;
WiFiClient client;                       //Set up MQTT and WiFi clients
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);
Adafruit_MQTT_Subscribe onoff = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/feeds/onoff"); //Set up the feed you're subscribing to. Here our feed name is onoff
Adafruit_MQTT_Subscribe estado = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/feeds/estado");
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/temp"); 
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/feeds/humid");

//variável que ler os entradas de dados do dht11
DHT dhht(18,DHT11);



void setup()
{
  Serial.begin(115200);
  dhht.begin();
  lcd.begin (16,2);
      
  Serial.print("\n\nConnecting Wifi... ");               //Conectar ao WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
  }
  Serial.println("OK!");
  Serial.println(WiFi.localIP());
  mqtt.subscribe(&onoff);                                //Subscribe para o feed do led
  mqtt.subscribe(&estado);                                  //Subscribe para o feed do dht
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  pinMode(26,OUTPUT);
  digitalWrite(26,LOW);
  
  
  lcd.init();    
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Wifi Conectado");
  delay(5000);
  lcd.clear();
  
  configTime(-3*3600,3600, "d.st1.ntp.br");//Site para pegar o horario e fuso horario
   
  
  MQTT_connect();
}


void loop() {
 
  DysplayTimeInfo();
  
  float h = dhht.readHumidity();
  float t = dhht.readTemperature();
  
  Adafruit_MQTT_Subscribe * subscription;
  MQTT_connect();

  if(isnan(t) || isnan(h)){
    Serial.println("Falha na leitura");
  }else{
    Serial.print("Temperatura: ");
    Serial.println(t);
    Serial.print("Umidade: ");
    Serial.println(h);
  }
  delay(500);
  
  //laço que ler os subscribes
  while((subscription = mqtt.readSubscription(1000))){
    if(subscription == &onoff){
      Serial.print("onoff: ");
      Serial.println((char*) onoff.lastread);
      if(!strcmp((char*) onoff.lastread, "DHTON")){
        digitalWrite(17, HIGH);
      } else if(!strcmp((char*) onoff.lastread, "DHTOFF")){
        digitalWrite(17, LOW);
      } else if(!strcmp((char*) onoff.lastread, "BUZZER")){
        Serial.println("O Buzzer Vai tocar...");
//        buzzer();
      } else if(!strcmp((char*) onoff.lastread, "LEDON")){
        digitalWrite(26,HIGH);
      } else if(!strcmp((char*) onoff.lastread, "LEDOFF")){
        digitalWrite(26,LOW);
      }
    }
    
    if(subscription == &estado){
      Serial.print("Deu certo mandar pro Adafruit: ");
      if(!strcmp((char*) estado.lastread, "TEMP")){
        Serial.println(t);
        temp.publish(t);
      } else if(!strcmp((char*) estado.lastread, "HUMID")){
        Serial.println(h);
        humid.publish(h);
      } else if(!strcmp((char*) estado.lastread, "STATUS")){
        Serial.println(t);
        Serial.println(h);
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(3,0);
        lcd.print("T: ");
        lcd.print(t);
        temp.publish(t);
        delay(10);
        lcd.setCursor(3,1);
        lcd.print("H: ");
        lcd.print(h);
        humid.publish(h);
        delay(5000);
        lcd.clear();
      }
      
    }
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

/////Função para exibir o dia, mes, ano e as horas pelo lcd
void DysplayTimeInfo(){

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Não conseguiu obter as horas");
    lcd.setCursor(0,0);
    lcd.print("Nao conseguiu");
    return;
  }
  Serial.println(&timeinfo, "%A, %B, %d %Y %H:%M:%S");
  
  lcd.backlight();
  lcd.setCursor(5,0);
  //lcd.print(&timeinfo, "%A, %B, %d %Y %H:%M:%S");
  lcd.print(&timeinfo, "%H:%M");
  lcd.setCursor(0,1);
  lcd.print(&timeinfo, "%d, %B, %Y");

}
