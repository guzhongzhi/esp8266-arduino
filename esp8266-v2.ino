#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 2048
#endif

#include <Stepper.h>
#include <arduino.h>
#include "var.h";
#include "config.h";
#include "ArduinoJson.h"
#include <PubSubClient.h>
#include <IRsend.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <IRremoteESP8266.h>

#include <assert.h>
#include <IRtext.h>
#include <IRutils.h>

//红外发射头/接收文件
#include <IRac.h>
#include <IRrecv.h>

const char* registryTopic =  "/device/register";
WiFiClient espClient;
PubSubClient MQTTClient(espClient);

//
IRrecv* irReceiver = NULL;
IRrecv irrecv(4, 2048, 15, true);

//
Stepper* stepper = NULL;
//https://blog.csdn.net/weixin_42358937/article/details/107022433
//电机不能反转
Stepper myStepper(200, 0,0,0,0);
int stepperPins[4];
int stepperTotal = 0;
int stepperST = 1;

void MQTTConnect() {
  while (!MQTTClient.connected()) {
    #ifdef DEBUG
    Serial.print("Attempting MQTT connection...");
    #endif
    String clientId = "/client/" + WiFi.macAddress();
    if (MQTTClient.connect(clientId.c_str(),"mqtt","mqtt")) {
      #ifdef DEBUG
      Serial.println("connected");
      #endif
      MQTTClient.subscribe(clientId.c_str());
    } else {
      #ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(MQTTClient.state());
      Serial.println(" try again in 5 seconds");
      #endif
      delay(5000);
    }
  }
}


String formatIRData2(String m) {
    String n = "{";
    int isStarted = 0;
    for(int i=0;i<m.length();i++) {
       if(m[i] == '{' ) {
         isStarted = 1;
         continue;
       }
       if(isStarted != 1 || m[i] == ' ') {
        continue;
       }
       if(m[i] == '\r' || m[i] == '\n') {
         break;
       }
        n += String(m[i]);      
    }
    return n;
}

bool checkIrInput() {
  if(irReceiver == NULL){
    return false;
  }
  decode_results results;
  if (irrecv.decode(&results)) {
    String a = resultToSourceCode(&results);
    Serial.println(a.c_str());
    MQTTClient.publish(registryTopic, jsonDeviceInfo(a.c_str()).c_str());
    return true;
  }
  return false;
}

void callback(char* topic, byte* payload, unsigned int length) {
  char data[length + 1];
  for (int i = 0; i < length; i++) {
    data[i] = (char) payload[i];
  }
  data[length] = '\0';
  
  DynamicJsonDocument doc(8116);
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    MQTTClient.publish(registryTopic, jsonDeviceInfo(error.c_str()).c_str());
    Serial.println(error.c_str());
    return;
  }

  const char* cmd = doc["cmd"].as<char*>();
  Serial.println(cmd);
  
  DynamicJsonDocument cmdFeedBack(256);
  cmdFeedBack["c"] = cmd;

  if ( strcmp(cmd,"stepper") == 0 ) {
    //{"cmd":"stepper","d":[15,12,13,14],"v":100,"pr":200,"st":1}
    Serial.println("setup stepper");
    JsonArray data = doc["d"];
    for(int i=0;i<4;i++) {
      stepperPins[i] = data[i].as<int>();
      pinMode(stepperPins[i], OUTPUT);
      digitalWrite(stepperPins[i],HIGH);
    }
    stepperTotal = doc["v"].as<int>();
    stepperST = doc["st"].as<int>();
    int stepsPerRevolution = doc["pr"].as<int>(); // change this to fit the number of steps per revolution
    //stepper = new Stepper(stepsPerRevolution, stepperPins[0],stepperPins[1],stepperPins[2],stepperPins[3]);
    stepper->motor_pin_1 = stepperPins[0];
    stepper->motor_pin_2 = stepperPins[2]; //电机不能反转,需要调换两个的位置
    stepper->motor_pin_3 = stepperPins[1]; //电机不能反转,需要调换两个的位置
    stepper->motor_pin_4 = stepperPins[3];
    stepper->number_of_steps = stepsPerRevolution;
  }
  //读数据字信号
  if ( strcmp(cmd,"sir") == 0 ) {
    uint16_t p = doc["p"].as<uint16_t>();
    Serial.println("setup sir");
    Serial.println(p);
    if(p > 0) {
      pinMode(p, INPUT);
      IRrecv irrecv(p, 1024, 15, true);
      irrecv.setTolerance(15);  // Override the default tolerance.
      irrecv.enableIRIn();      // Start the receiver
      irReceiver = &irrecv;
    } else if(irReceiver != NULL) {
      irReceiver->disableIRIn();
      irReceiver = NULL;
    }
    cmdFeedBack["p"] = p; 
  }
  //读数据字信号
  if ( strcmp(cmd,"rd") == 0 ) {
    uint16_t p = doc["p"].as<uint16_t>();
    pinMode(p, INPUT);
    cmdFeedBack["p"] = p; 
    cmdFeedBack["v"] = digitalRead(p); 
  }
  //写数字信号
  if ( strcmp(cmd,"sd") == 0 ) {
    uint16_t p = doc["p"].as<uint16_t>();
    uint16_t v = doc["v"].as<uint16_t>();
    pinMode(p, OUTPUT);
    if(v > 0) {
      digitalWrite(p,1);    
    } else {
      digitalWrite(p,0);
    }
  }
  //读取模拟信号
  if(strcmp(cmd,"ra") == 0) {
    uint16_t p = doc["p"].as<uint16_t>();
    pinMode(p, INPUT);
    cmdFeedBack["p"] = p; 
    cmdFeedBack["v"] = analogRead(p); 
  }
  //写模拟信号
  if(strcmp(cmd,"ra") == 0) {
    uint16_t p = doc["p"].as<uint16_t>();
    uint16_t v = doc["v"].as<uint16_t>();
    pinMode(p, OUTPUT);
    cmdFeedBack["p"] = p; 
    analogWrite(p,v);
  }
  //红外发射
  if(strcmp(cmd,"irs") == 0) {
    uint16_t p = doc["p"].as<uint16_t>();
    JsonArray data = doc["d"];
   
    Serial.println("data.size()");
    Serial.println(data.size());
    
    pinMode(p, OUTPUT);
    IRsend irsend(p);

    uint16_t rawData[data.size()];
    for(int i=0;i<data.size();i++) {
      rawData[i] = data[i].as<uint16_t>();
    }
    Serial.println("send start");
    irsend.sendRaw(rawData, data.size(), 38);
    Serial.println("send end");
  }
  
  String output = "";
  serializeJson( cmdFeedBack,  output);
   
  MQTTClient.publish(registryTopic, jsonDeviceInfo(output).c_str());
}

void setup() {
  Serial.begin(115200);
  Serial.println("");
  irReceiver = NULL;
  
  irrecv.setUnknownThreshold(12);
  irrecv.setTolerance(15);  // Override the default tolerance.
  irrecv.enableIRIn();      // Start the receiver
  
  Serial.println(MQTT_MAX_PACKET_SIZE);
  if (!autoConfig()){
      smartConfig();
  }
  #ifdef DEBUG
  Serial.println(WiFi.macAddress());
  #endif
  MQTTClient.setServer("mqtt.gulusoft.com", 1883);
  MQTTClient.setCallback(callback);

  //
  stepper = &myStepper;
}

String jsonDeviceInfo(String data) {
   extern String versionCode;
   StaticJsonDocument<256> doc;
   doc["m"] = WiFi.macAddress();
   doc["i"] = WiFi.localIP().toString();
   doc["w"] = WiFi.SSID();
   doc["b"] = isNewBoot;
   doc["v"] = versionCode;
   doc["d"] = data;
   String output = "";
   serializeJson( doc,  output);
   isNewBoot = false;
   return output;
}

unsigned long lastMsg = 0;
void loop() {
  if( stepperTotal > 0 && stepper != NULL) {
    Serial.println(stepperTotal);
    Serial.println(stepper->motor_pin_1);
    stepperTotal--;
    stepper->step(stepperST > 0 ? 1 : -1);
    if(stepperTotal == 0) {
      for(int i=0;i<4;i++) {
          digitalWrite(stepperPins[i], LOW);
      }
    }
    delay(5);
    return;
  }
  if (!MQTTClient.connected()) {
    MQTTConnect();
  }
  MQTTClient.loop();
  checkIrInput();
  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    MQTTClient.publish(registryTopic, jsonDeviceInfo("").c_str());
  }
}
