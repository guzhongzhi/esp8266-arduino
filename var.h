#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <iostream>
#include <sstream>

using namespace std;


//System
String versionCode = "1.2.15";
bool isNewBoot = true;

typedef struct Command {
  int pin;
  String type;
  String data;
};
typedef struct Device {
  String mac;
  String wifi;
  String ip;
};

Device device;
