#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 2048
#endif

#include <Stepper.h>
#include <Arduino.h>
#include "var.h";
#include "config.h";
#include "ArduinoJson.h"
#include <PubSubClient.h>
#include <IRsend.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <IRremoteESP8266.h>


//红外发射头/接收文件
#include <assert.h>
#include <IRac.h>
#include <IRrecv.h>
#include <IRtext.h>
#include <IRutils.h>
#include <IRac.h>




const char* registryTopic =  "/device/register";
WiFiClient espClient;
PubSubClient MQTTClient(espClient);

//

const uint16_t kRecvPin = 14;
const uint32_t kBaudRate = 115200;
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTolerancePercentage = kTolerance;  // kTolerance is normally 25%
#if DECODE_AC
const uint8_t kTimeout = 50;
#else   // DECODE_AC
const uint8_t kTimeout = 15;
#endif  // DECODE_AC
const uint16_t kMinUnknownSize = 12;
#define LEGACY_TIMING_INFO false
IRrecv* irReceiver = NULL;
IRrecv irrecv(14, 1024, 15, true);
decode_results results;  // Somewhere to store the results
String lastIRData = "";
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
   bool rs=irrecv.decode(&results);
   if(!rs) {
       return false;
   }
    String a = resultToSourceCode(&results);
    String b = formatIRData2(a);
    Serial.println("===========================================");

   int docLen = (int)(2048);
   DynamicJsonDocument  doc(docLen);
   doc["c"] = "rir";
   doc["d"] = b;
   String output = "";
   serializeJson( doc,  output);
   Serial.println(output);
   lastIRData=output;
   yield();
   return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  char data[length + 1];
  for (int i = 0; i < length; i++) {
    data[i] = (char) payload[i];
  }
  data[length] = '\0';
  Serial.println(length);
   int docLen = (int) (8102);
  DynamicJsonDocument doc(docLen);
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    MQTTClient.publish(registryTopic, jsonDeviceInfo(error.c_str()).c_str());
    Serial.println(error.c_str());
    return;
  }

  const char* cmd = doc["c"].as<char*>();
  Serial.println(cmd);
  
  DynamicJsonDocument cmdFeedBack(256);
  cmdFeedBack["c"] = cmd;

  if ( strcmp(cmd,"stepper") == 0 ) {
    //{"c":"stepper","d":[15,12,13,14],"v":100,"pr":200,"st":1}
    Serial.println("setup stepper");
    JsonArray data = doc["stepper"]["d"];
    for(int i=0;i<4;i++) {
      stepperPins[i] = data[i].as<int>();
      pinMode(stepperPins[i], OUTPUT);
      digitalWrite(stepperPins[i],HIGH);
    }
    stepperTotal = doc["stepper"]["v"].as<int>();
    stepperST = doc["stepper"]["st"].as<int>();
    int stepsPerRevolution = doc["stepper"]["pr"].as<int>(); // change this to fit the number of steps per revolution
    //stepper = new Stepper(stepsPerRevolution, stepperPins[0],stepperPins[1],stepperPins[2],stepperPins[3]);
    stepper->motor_pin_1 = stepperPins[0];
    stepper->motor_pin_2 = stepperPins[2]; //电机不能反转,需要调换两个的位置
    stepper->motor_pin_3 = stepperPins[1]; //电机不能反转,需要调换两个的位置
    stepper->motor_pin_4 = stepperPins[3];
    stepper->number_of_steps = stepsPerRevolution;
  }
  //读数红外设置
  if ( strcmp(cmd,"sir") == 0 ) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    Serial.println("setup sir");
    Serial.println(p);
    if(p > 0) {
      pinMode(p, INPUT);
      IRrecv irrecv(p, 1024, 15, true);
      irrecv.setTolerance(25);  // Override the default tolerance.
      irrecv.setUnknownThreshold(12);
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
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    pinMode(p, INPUT);
    cmdFeedBack["p"] = p; 
    cmdFeedBack["v"] = digitalRead(p); 
  }
  //写数字信号
  if ( strcmp(cmd,"sd") == 0 ) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    uint16_t v = doc["pin"]["v"].as<uint16_t>();
    pinMode(p, OUTPUT);
    if(v > 0) {
      digitalWrite(p,1);    
    } else {
      digitalWrite(p,0);
    }
  }
  //读取模拟信号
  if(strcmp(cmd,"ra") == 0) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    pinMode(p, INPUT);
    cmdFeedBack["p"] = p; 
    cmdFeedBack["v"] = analogRead(p); 
  }
  //写模拟信号
  if(strcmp(cmd,"ra") == 0) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    uint16_t v = doc["pin"]["v"].as<uint16_t>();
    pinMode(p, OUTPUT);
    cmdFeedBack["p"] = p; 
    analogWrite(p,v);
  }
  //红外发射
  if(strcmp(cmd,"irs") == 0) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    JsonArray data = doc["pin"]["d"];
    Serial.println("send irs");
    Serial.println(p);
    Serial.println("data.size()");
    Serial.println(data.size());
    
    IRsend irsend(p);
    irsend.begin();
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
  
  Serial.println(MQTT_MAX_PACKET_SIZE);
  if (!autoConfig()){
      smartConfig();
  }
  #ifdef DEBUG
  Serial.println(WiFi.macAddress());
  #endif
  MQTTClient.setServer("mqtt.home.gulusoft.com", 1883);
  MQTTClient.setCallback(callback);

  //
  stepper = &myStepper;

  
  irReceiver = NULL;
  
  irrecv.setUnknownThreshold(12);
  irrecv.setTolerance(25);  // Override the default tolerance.
  irrecv.enableIRIn();      // Start the receiver
}

String jsonDeviceInfo(String data) {
   extern String versionCode;

   DynamicJsonDocument  doc(data.length() + 256);
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

void cloop() {
 
  unsigned long now = millis();
 if( stepperTotal > 0 && stepper != NULL) {
    Serial.println(stepperTotal);
    stepperTotal--;
    stepper->step(stepperST > 0 ? 1 : -1);
    if(stepperTotal == 0) {
      Serial.println("set pins to low");
      for(int i=0;i<4;i++) {
          digitalWrite(stepperPins[i], LOW);
      }
    }
    delay(100);
    return;
  }
  if (!MQTTClient.connected()) {
    MQTTConnect();
  }
  if(lastIRData != "") {
    Serial.println("send ir data");
    Serial.println(lastIRData);
    MQTTClient.publish(registryTopic, jsonDeviceInfo(lastIRData.c_str()).c_str());
    lastIRData = "";
    lastMsg = now;
  }
  MQTTClient.loop();
  checkIrInput();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    MQTTClient.publish(registryTopic, jsonDeviceInfo("").c_str());
  }
  
}

void loop() {
  cloop();
}
