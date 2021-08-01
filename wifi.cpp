#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "config.h"


void smartConfig()
{
    WiFi.mode( WIFI_STA );
    WiFi.beginSmartConfig();
    Serial.println("smartConfig");
    while ( WiFi.status() != WL_CONNECTED )
    {
        if (! WiFi.smartConfigDone() )
        {
            delay( 1000 );
            Serial.print(".");
            continue;
        }
    }
    WiFi.setAutoConnect( true ); 
    Serial.println( WiFi.localIP());
}

bool autoConfig()
{
    Serial.println( "\r\nWait for Smartconfig" );
    WiFi.mode(WIFI_STA);    
    //Serial.print( "connect wifi." );
    //WiFi.begin("10012503", "gd10012503");
    
    WiFi.begin();
    short int maxNum = 10;
    while ( maxNum  > 0)
    {
        maxNum--;
        #ifdef DEBUG
        Serial.print( "." );
        #endif
        delay( 1000 );
        if(WiFi.status() != WL_CONNECTED) {
          continue;
        }
    }
    #ifdef DEBUG
    Serial.println("");
    Serial.println(WiFi.localIP());
    Serial.println("");
    #endif
    return(WiFi.status() == WL_CONNECTED);
}
