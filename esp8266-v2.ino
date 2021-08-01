#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 2048
#endif

#include <Arduino.h>
#include "var.h";
#include "config.h";
#include "ArduinoJson.h"
#include <PubSubClient.h>
#include <IRsend.h>
#include <iostream>
#include <sstream>
#include <vector>



const char* registryTopic =  "/device/register";
WiFiClient espClient;
PubSubClient MQTTClient(espClient);

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
  if (!MQTTClient.connected()) {
    MQTTConnect();
  }
  MQTTClient.loop();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    MQTTClient.publish(registryTopic, jsonDeviceInfo("").c_str());
  }
}

void loop() {
  cloop();
}
