#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 2048
#endif

#include <Arduino.h>
#include "var.h";
#include "config.h";
#include "ArduinoJson.h"
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>

const char* registryTopic =  "/device/register";
WiFiClient espClient;
PubSubClient MQTTClient(espClient);

void MQTTConnect() {
  while (!MQTTClient.connected()) {
    #ifdef DEBUG
    Serial.print("MQTT connection...");
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

class Executor {
  public: 
    virtual void execute() = 0;
    virtual bool stopNext() = 0;
    virtual String getName() = 0;
    
};

//心跳
class HeadBeat :public Executor {
  public:
  String getName() {
    return String("heartBeat");
  };
  void execute()  {
    unsigned long now = millis();
    if((now - lastMsg) > duration) {
      Serial.println("heartBeat upgrade");
      lastMsg = now;
      MQTTClient.publish(registryTopic, jsonDeviceInfo("").c_str());
    }
  };
  bool stopNext() {
    return false;
  }
  HeadBeat(unsigned long d) {
    duration = d;
  };
  protected:
  unsigned long lastMsg;
  unsigned long duration;
};


void upgrade_started() {
  Serial.println("started");
};
void upgrade_finished() {
  Serial.println("finished");
};  
void upgrade_progress(int cur, int total) {
  Serial.printf("%d of %d bytes...\n", cur, total);
};  
void upgrade_error(int err) {
  Serial.printf("fatal error code %d\n", err);
};
//升级
class Upgrade: public Executor {
  public:
  String getName() {
    return String("Upgrade");
  };
  bool stopNext() {
    return true;
  }
  void execute() {
    this->upgrade();
  };
  Upgrade(String host, String path, int port){
    this->host = host;
    this->path = path;
    this->port = port;
  };

  protected:
    String host;
    String path;
    int port;
    void upgrade() {
      Serial.printf("execute upgrade");
  
      ESPhttpUpdate.setLedPin(LED_BUILTIN, HIGH);
      ESPhttpUpdate.onStart(upgrade_started);
      ESPhttpUpdate.onEnd(upgrade_finished);
      ESPhttpUpdate.onProgress(upgrade_progress);
      ESPhttpUpdate.onError(upgrade_error);

      t_httpUpdate_return ret = ESPhttpUpdate.update(host, port,path);  
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
    };
};

struct node{
  Executor *executor;
  struct node *next;
};

//执行链
class LinkedList {
  private:
  node *head, *last;
  public:
  LinkedList() {
    head = NULL;
    last = NULL;
  };
  void execute() {
      node *n;
      n = head;
      while(NULL != n) {
          if(NULL != n->executor) {
              n->executor->execute();
              if(n->executor->stopNext()) {
                return;
              }
          }
          n=n->next;
      }
  };
  void deleteExecutor(String name) {
    node *n = head;
    while(n != NULL) {
      if(n->executor != NULL && n->executor->getName() == name) {
        n->executor = NULL;
      }
      n = n->next;
    }
  };
  void append(Executor * exec) {
    node *n = head;
    bool isExisting = false;
    while(n != NULL) {
        if(n->executor == NULL) {
          n->executor = exec;
          isExisting = true;
          break;
        }
        if(n->executor->getName() == exec->getName()) {
          delete(n->executor);
          n->executor = exec;
          isExisting = true;
        }
        n = n->next;
    }
    if(isExisting == true) {
      return;
    }
    
    node *tmp = new node;
    tmp->executor = exec;
    Serial.println("add new node");
    Serial.println(tmp->executor->getName());
    //exec->execute();
    tmp->next = NULL;
    if(head == NULL){
      head = tmp;
      last = tmp;
    } else {
      last->next=tmp;
      last = tmp;
    }
  };
};

LinkedList *list = new LinkedList();

void sendData(DynamicJsonDocument cmdFeedBack) {
  String output = "";
  serializeJson( cmdFeedBack,  output);
  MQTTClient.publish(registryTopic, jsonDeviceInfo(output).c_str());
}


void callback(char* topic, byte* payload, unsigned int length) {
  char data[length + 1];
  for (int i = 0; i < length; i++) {
    data[i] = (char) payload[i];
  }
  data[length] = '\0';
  int docLen = (int) (1024);
  DynamicJsonDocument doc(docLen);
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    MQTTClient.publish(registryTopic, jsonDeviceInfo(error.c_str()).c_str());
    Serial.println(error.c_str());
    return;
  }

  const char* cmd = doc["c"].as<char*>();
  DynamicJsonDocument cmdFeedBack(512);
  cmdFeedBack["c"] = cmd;

  if(strcmp(cmd, "upg") == 0) {
    String host = doc["upg"]["host"].as<String>();
    int port = doc["upg"]["port"].as<int>();
    String path = doc["upg"]["path"].as<String>();
    Upgrade* u = new Upgrade(host,path,port);
    list->append(u);
  }
  sendData(cmdFeedBack);  
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
  if (!MQTTClient.connected()) {
    MQTTConnect();
  }
  list->append(new HeadBeat(10000));
}

String jsonDeviceInfo(String data) {
   extern String versionCode;

   DynamicJsonDocument  doc(data.length() + 256);
   doc["m"] = WiFi.macAddress();
   doc["i"] = WiFi.localIP().toString();
   doc["w"] = WiFi.SSID();
   doc["b"] = isNewBoot;
   doc["v"] = versionCode;
   doc["upg"] = true;
   doc["d"] = data;
   String output = "";
   serializeJson( doc,  output);
   isNewBoot = false;
   return output;
}

void loop() {
  list->execute();
  if (!MQTTClient.connected()) {
    MQTTConnect();
  }
  MQTTClient.loop();
}
