
/*
 Basic MQTT example with Authentication

  - connects to an MQTT server, providing username
    and password
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic"
*/

#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//169,254,103,61  192,168,2,100
//IPAddress server(169,254,103,61); //station IP (localhost)
IPAddress server(192,168,2,102); //station IP (localhost)

const char ssid[] = "RobLabStud";
const char pass[] = "12345678";

//const char ssid[] = "anton";
//const char pass[] = "12345678";

const char ID[] = "1";

// motor control consts and vars
const byte M1PWM = 2;
const byte M1PH = 3;
const byte M2PWM = 4;
const byte M2PH = 5;
const byte MODE_PIN = 7;

int throt_A = 0;
int throt_B = 0;
bool dir_A = LOW;
bool dir_B = LOW;


float battery_life;
const int ledPin =  LED_BUILTIN;// the number of the LED pin

//sampling rates
unsigned long previousMillis[2] = {0, 0}; //number of rates
const int sampling_interval = 1000;
const int battery_interval = 10000;


//--------------------------------------- callback -------------------------------

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("incoming command");
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload, length);
  int command_1 = doc["L"];
  int command_2 = doc["R"];
  Serial.println(command_1);
  Serial.println(command_2);
  execute(command_1, command_2);  
}


WiFiClient net;
PubSubClient client(server, 1883, callback, net);



//--------------------------------------- functions -----------------------------

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
           
      char  temp[3]; //temporal data
      sprintf(temp, ID);
      
      client.publish("setup/4", temp);
      // ... and resubscribe
      client.subscribe("command/4");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void send_message(int interval, byte idx)
{
  if ((millis() - previousMillis[idx]) >= interval){
    previousMillis[idx]= millis(); //stores the millis value in the selected array
    
    StaticJsonDocument<200> doc;
    doc["ID"] = ID;
    JsonArray sensors = doc.createNestedArray("sensors");
    sensors.add(48.756080);
    sensors.add(3.302038);
    sensors.add(5.302038);
    sensors.add(2.302038);
    sensors.add(7.302038);
    sensors.add(2.302038);
    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish("mkr/sensors", buffer, false); 
  } 
}



void battery(int interval, int idx)
{
  if ((millis() - previousMillis[idx]) >= interval){
    previousMillis[idx]= millis(); //stores the millis value in the selected array

    //prepare and publish battery status
    StaticJsonDocument<60> doc;
    doc["ID"] = ID;
    doc["battery"] = 70;
    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish("mkr/battery", buffer, false);    
  } 
}


void execute(int L, int R) //received command execution 
{
//  Serial.println("executing");
//  digitalWrite(ledPin, command1);
  if (L < 0) {
    dir_A = LOW;
  }
  else {
    dir_A = HIGH;
  }
  if (R < 0) {
    dir_B = LOW;
  }
  else {
    dir_B = HIGH;
  }
  digitalWrite(M1PH, dir_A);
  digitalWrite(M2PH, dir_B);
  analogWrite(M1PWM, abs(L));
  analogWrite(M2PWM, abs(R));
  //Serial.println("received command");
}
  

//--------------------------------------- setup ---------------------------------


void setup()
{
  
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  // Note - the default maximum packet size is 128 bytes. If the
  // combined length of clientId, username and password exceed this use the
  // following to increase the buffer size:
  client.setBufferSize(255);
  battery_life = 0;
  pinMode(ledPin, OUTPUT);

}


//--------------------------------------- main loop -----------------------------

void loop()
{
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //send_message(sampling_interval, 0);
  //battery(battery_interval, 1);

}
