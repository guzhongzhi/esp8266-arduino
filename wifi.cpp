#include <WiFi.h>
#include "config.h"


void smartConfig()
{
    WiFi.mode( WIFI_STA );
    WiFi.beginSmartConfig();
    while ( WiFi.status() != WL_CONNECTED )
    {
        #ifdef DEBUG
        Serial.println("smartConfig");
        #endif
         
        if (! WiFi.smartConfigDone() )
        {
            delay( 1000 );
            #ifdef DEBUG
            Serial.println("smconfig");
            #endif
            continue;
        }
        WiFi.setAutoConnect( true ); 
        break;
    }
    #ifdef DEBUG
    Serial.println( WiFi.localIP());
    #endif
}

bool autoConfig()
{
    WiFi.mode(WIFI_STA);
    
    #ifdef DEBUG
    Serial.print( "connect wifi." );
    WiFi.begin("10012503", "gd10012503");
    #else
    WiFi.begin();
    #endif
    
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
