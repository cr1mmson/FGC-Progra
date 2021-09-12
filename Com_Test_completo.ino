/*
 *TEST COMPLETO
 *SENSORES: MQ2, ANEMOMETRO,TL05i
 *PARA MEDICION DE: GAS, VELOCIDAD DEL VIENTO, TEMPERATURA Y HUMEDAD
*/

#include <WiFi.h> 
#include <PubSubClient.h> //Libreria para publicación y recepción de datos. (Buscar como PubSubClient)
#include <Wire.h>         //Conexión de dispositivos I2C
#include <NTPClient.h> //Libreria NTPClient(instalar) Es para darle la hora al programa
#include <WiFiUdp.h> //includio en la libreria NTPClient
#include "Adafruit_SHT31.h"

Adafruit_SHT31 sensor = Adafruit_SHT31();

WiFiUDP ntpUDP;//(parte del proceso de conexión al wifi)
NTPClient timeClient(ntpUDP);

const char *ssid = " "; // Nombre del SSID
const char *password = " "; // Contraseña
                                      //Modificar al nombre que se asigne en el dashboard.
#define TEAM_NAME "com/Cindy/002"  //  proyecto/usuario/no.estacion (a donde se enviará la información)
String output;

//*********************************************************
//***************  INFO DEL SERVER  ***********************
//*********************************************************
const char *mqtt_server = "test.mosquitto.org"; //broker al que conectamos
int mqtt_port = 1883;

char msg[50]; 
char topic_name[250];
char msg_r[50];
char temperaturenow [15];
char humiditynow [15];
char gas[15];
char velo1now[15];

//variables sensor gas
int smokeA0 = 35;
int sensorThres = 1400;

//Variables Anemometro
float veloc1=35;
int tiempo=0;
int cnt=0;
float v1=0;
float v2=0;
int INTERNAL;
#define analogReference;

// network variables
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void setup() {
  pinMode(smokeA0,INPUT);
  Serial.begin(115200);
  Wire.begin(21, 22); // Pines I2C del ESP32
  Serial.println("Conectando a WiFi");
  //iniciar Wifi
  setupWiFi();
  //Setup para MQTT
  Serial.println("Activando MQTT");
  setupMQTT();
  //Inicia la hora
  timeClient.begin();
  timeClient.setTimeOffset(-21600);

if(sensor.begin(0x44)){
  Serial.println("sensor encontrado");
  }
  else{
    Serial.println("sensor no encontrado");
    }
}
  boolean publicar_flag = true;  //flag utilizada para el tiempo

void loop() {
  
   if (!mqtt_client.connected())
  {
    reconnect();
    delay(2000);
  }
   mqtt_client.loop();
   timeClient.update();

   v1 =(analogRead(35)); // lectura de sensor a0
veloc1 = (v1*0.0076);

   
//Se publican los datos cada 1 min.
if((timeClient.getMinutes() % 1 == 00) && (timeClient.getSeconds() == 00) && publicar_flag)
{
 
    Serial.println("Datos publicados en MQQTT Server: ");
    publicarDatos();
    publicar_flag=false;
}
  else if (timeClient.getSeconds() != 00)
  {
    publicar_flag = true;
  }
    //Velocidad
    Serial.print(veloc1);  //muestra la velocidad del viento en el LCD 
    Serial.println(": Km/h");
    Serial.println("___");
    if (veloc1>v2){
      v2=veloc1;
      Serial.println (v2,1); // muestra la velocidad maxima que alcanzo  
    } 
    delay(1000);

  //gas
  
  /*int analogSensor = analogRead(smokeA0);  //funcion de gas
  Serial.print("Nivel de Toxicidad= ");
  Serial.println(analogSensor);

  if(analogSensor>sensorThres){
   //Serial.println("peligro");
  }

  else
  {
    //Serial.println("normal");
  }*/

  
}

//*********************************************************
//****************  CONEXION WIFI  ************************
//*********************************************************
void setupWiFi(){
   //Inicializamos WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  //Si aún no está establecida la conexión enviará puntos cada 500ms
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //Fin de la inicialización WiFi
}

//*********************************************************
//*****************  CONEXION MQTT  ***********************
//*********************************************************
void setupMQTT() {
  delay(10);
  // Iniciamos la conexion WiFi con la Red que colocamos
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid,password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

//servidor anteriormente mencionado
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(callback);
  
 Serial.println("Connected ");
 Serial.print("MQTT Server ");
 Serial.print(mqtt_server);
 Serial.print(":");
 Serial.println(String(mqtt_port)); 
}

//*********************************************************
//**************  FUNCION PUBLICAR DATOS  *****************
//*********************************************************
void publicarDatos(){
float temp = sensor.readTemperature();
float hume = sensor.readHumidity();
float analogSensor = analogRead(smokeA0);

Serial.println(temp);
Serial.println(hume);
Serial.println(analogSensor);
Serial.println(veloc1);

 dtostrf(temp,7, 3, temperaturenow); //// convert float to char ->temp
 dtostrf(hume,7, 3, humiditynow); //// convert float to char ->hume
 dtostrf(analogSensor,7, 3, gas); //// convert float to char ->gas
 dtostrf(veloc1,7, 3, velo1now); //// convert float to char ->viento
  mqtt_client.publish(getTopic("temp"), temperaturenow); //Publicando
  mqtt_client.publish(getTopic("hume"), humiditynow);
  mqtt_client.publish(getTopic("gas"), gas);
  mqtt_client.publish(getTopic("ane"), velo1now);
  publicar_flag=false;       
}

char *getTopic(char *topic)
{
  sprintf(topic_name, "%s/%s", TEAM_NAME, topic);
  Serial.println(topic_name); //Revisando el topic que se está mandando
  return topic_name;
}

//*********************************************************
//***************  FUNCION CALLBACK  **********************
//*********************************************************
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
 Serial.println();
}

void publish(char *topic, char *payload)
{
  mqtt_client.publish(topic, payload);
}

//*********************************************************
//*************  FUNCION RECONECTAR MQTT  *****************
//*********************************************************
void reconnect()
{
  // Loop until we're reconnected
  while (!mqtt_client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect

  if(mqtt_client.connect("ESP32Client6")){  //Colocar nombre Unico
      Serial.println("connected");
    } 
    else{
   Serial.print("failed, rc=");
   Serial.print(mqtt_client.state());
   Serial.println(" try again in 5 seconds");
   // Wait 5 seconds before retrying
   delay(5000);
    }
  }
}
