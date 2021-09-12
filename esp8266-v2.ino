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
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>


#include <OneWire.h>
// OneWire DS18S20, DS18B20, DS1822 Temperature Example
// http://www.pjrc.com/teensy/td_libs_OneWire.html
// The DallasTemperature library can do all this work for you!
// https://github.com/milesburton/Arduino-Temperature-Control-Library

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
bool IRReceiverEnabled = false;
//
Stepper* stepper = NULL;
//https://blog.csdn.net/weixin_42358937/article/details/107022433
//电机不能反转
Stepper myStepper(64, 0,0,0,0);
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



float readTemperature(int pin) {
  OneWire  ds(pin);  // on pin 10 (a 4.7K resistor is necessary)
  
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return 0;
  }
  

  if (OneWire::crc8(addr, 7) != addr[7]) {
      return 0 ;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return 0;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
  return celsius;
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
  if(strcmp(cmd, "upg") == 0) {
    String url = doc["upg"]["u"].as<String>();
    Serial.println("upgrade");
    upgrade(url);
  }
  if(strcmp(cmd, "rt") == 0) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    Serial.println("read temperture");
    Serial.println(p);
    cmdFeedBack["p"] = p; 
    cmdFeedBack["v"] = readTemperature(p);
  }
  //读数红外设置
  if ( strcmp(cmd,"sir") == 0 ) {
    uint16_t p = doc["pin"]["p"].as<uint16_t>();
    Serial.println("setup sir");
    Serial.println(p);
    if(p > 0) {
      IRReceiverEnabled = true;
    } else if(irReceiver != NULL) {
      IRReceiverEnabled = false;
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
    
    bool hasAfter = doc["pin"]["f"];
    if(hasAfter) {
       delay(50);
       Serial.println("has after send data need to send");
       JsonArray data = doc["pin"]["fd"];
       Serial.println(data.size());
       uint16_t rawData[data.size()];
       for(int i=0;i<data.size();i++) {
          rawData[i] = data[i].as<uint16_t>();
       }
       Serial.println("send start");
       irsend.sendRaw(rawData, data.size(), 38);
       Serial.println("has after send data need to send done");
    }
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
    delay(50);
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
  if(IRReceiverEnabled) {
    checkIrInput();
  }
  if (now - lastMsg > 10000) {
    lastMsg = now;
    MQTTClient.publish(registryTopic, jsonDeviceInfo("").c_str());
  }
  
}

void loop() {
  cloop();
}
