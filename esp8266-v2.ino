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
#include <IRremoteESP8266.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>



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


void upgrade_started() {
  Serial.println("HTTP update process started");
}

void upgrade_finished() {
  Serial.println("HTTP update process finished");
}

void upgrade_progress(int cur, int total) {
  Serial.printf("HTTP update process at %d of %d bytes...\n", cur, total);
}

void upgrade_error(int err) {
  Serial.printf("HTTP update fatal error code %d\n", err);
}

void upgrade(String url) {
    WiFiClient client;

    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
    ESPhttpUpdate.onStart(upgrade_started);
    ESPhttpUpdate.onEnd(upgrade_finished);
    ESPhttpUpdate.onProgress(upgrade_progress);
    ESPhttpUpdate.onError(upgrade_error);

    t_httpUpdate_return ret = ESPhttpUpdate.update(client, url);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("OK");
        break;
    }
}
void callback(char* topic, byte* payload, unsigned int length) {
  char data[length + 1];
  for (int i = 0; i < length; i++) {
    data[i] = (char) payload[i];
  }
  data[length] = '\0';
  Serial.println(length);
   int docLen = (int) (1024);
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


  if(strcmp(cmd, "upg") == 0) {
    String url = doc["upg"]["u"].as<String>();
    Serial.println("upgrade");
    upgrade(url);
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
      digitalWrite(p,HIGH);    
    } else {
      digitalWrite(p,LOW);
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
  Serial.println(WiFi.macAddress());
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
    Serial.println(jsonDeviceInfo(""));
    MQTTClient.publish(registryTopic, jsonDeviceInfo("").c_str());
  }
}

void loop() {
  cloop();
}
