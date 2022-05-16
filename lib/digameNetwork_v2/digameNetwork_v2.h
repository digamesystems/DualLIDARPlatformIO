/* digameNetwork_v2.h
 *
 *  Functions for logging into a wifi network and reporting the device's
 *  MAC address, and POSTING a JSON message to a server.
 *
 *  Copyright 2021, Digame Systems. All rights reserved.
 */

#ifndef __DIGAME_NETWORK_H__
#define __DIGAME_NETWORK_H__
#include "digameDebug.h"
#include "digamePowerMgt.h" 
#include <WiFi.h>       // WiFi stack
#include <HTTPClient.h> // To post to the ParkData Server

struct NetworkConfig
{
    String hostName  = "Digame Device";
    String ssid      = "YOUR_SSID";
    String password  = "YOUR_PASSSORD";
    // serverURL is the destination for the postJSON function below
    String serverURL = "YOUR_SERVER_URL";
};

// Globals
bool wifiConnected = false;
long msLastConnectionAttempt; // Timer value of the last time we tried to connect to the wifi.
HTTPClient http;              // The class we use to POST messages

//*****************************************************************************
// Return the device's MAC address as a String
String getMACAddress()
{
    byte mac[6];
    String retString;

    WiFi.macAddress(mac);

    char buffer[3];
    for (int i = 0; i<5; i++){
      sprintf(buffer, "%02x", mac[i]);
      retString = String(retString + buffer + ":");
    }
    sprintf(buffer, "%02x", mac[5]);
    retString = String(retString + buffer);

    return retString;
}

//*****************************************************************************
// Return the last four digits of the device's MAC address as a String
String getShortMACAddress()
{
    byte mac[6];
    String retString;

    WiFi.macAddress(mac);
    //DEBUG_PRINTLN(mac);
    char buffer[3]; 
    sprintf (buffer, "%02x", mac[4]);
    retString = String(retString + buffer);
    sprintf (buffer, "%02x", mac[5]);
    retString = String(retString + buffer);

    return retString;
}

//*****************************************************************************
// Enable WiFi and log into the network
bool enableWiFi(NetworkConfig config)
{
    String ssid     = config.ssid;
    String password = config.password;

    WiFi.disconnect();   // Disconnect the network
    WiFi.mode(WIFI_OFF); // Switch WiFi off
    DEBUG_PRINTLN(ssid);
    DEBUG_PRINTLN(password);
    DEBUG_PRINTLN(config.serverURL);
    delay(100); // Wait a bit...

    DEBUG_PRINT("  Starting WiFi");
    WiFi.mode(WIFI_STA); // Station mode
    delay(100);
    String hostName = config.hostName;
    hostName.replace(" ", "_");
    WiFi.setHostname(hostName.c_str());         // define hostname
    WiFi.begin(ssid.c_str(), password.c_str()); // Log in

    bool timedout = false;
    unsigned long wifiTimeout = 10000;
    unsigned long tstart = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        DEBUG_PRINT(".");
        if ((millis() - tstart) > wifiTimeout)
        {
            DEBUG_PRINTLN("Timeout attempting WiFi connection.");
            timedout = true;
            break;
        }
    }

    msLastConnectionAttempt = millis(); // Timer value of the last time we tried to connect to the wifi.

    if (timedout)
    {
        wifiConnected = false;
        return false;
    }
    else
    {
        DEBUG_PRINTLN();
        DEBUG_PRINTLN("    WiFi connected.");
        DEBUG_PRINT("    IP address: ");
        DEBUG_PRINTLN(WiFi.localIP());
        wifiConnected = true;

        // Your Domain name with URL path or IP address with path
        // http.begin(config.serverURL);

        return true;
    }
}

bool initWiFi(NetworkConfig config)
{
    return enableWiFi(config);
}

//*****************************************************************************
void disableWiFi()
{
    WiFi.disconnect(true); // Disconnect from the network
    WiFi.mode(WIFI_OFF);   // Switch WiFi off
    btStop();
    //adc_power_off();
    adc_power_release();
    esp_wifi_stop();
    esp_bt_controller_disable();
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("WiFi disconnected!");
    wifiConnected = false;
}

//*****************************************************************************
// Save a single JSON message to the server. TODO: Deal with retries, etc.
// in a smart way.
bool postJSON(String jsonPayload, NetworkConfig config)
{
    //DEBUG_PRINT("postJSON Running on Core #: ");
    //DEBUG_PRINTLN(xPortGetCoreID());
    // DEBUG_PRINT("Free Heap: ");
    // DEBUG_PRINTLN(ESP.getFreeHeap());

    
    if (WiFi.status() != WL_CONNECTED)
    {
        DEBUG_PRINTLN("WiFi Connection Lost.");
        if (enableWiFi(config) == false)
        {
            return false;
        };
    }

    unsigned long t1 = millis();

    // Your Domain name with URL path or IP address with path
    http.begin(config.serverURL);

    //http.setReuse(false);
    
    // http.begin("http://199.21.201.53/trailwaze/zion/lidar_sensor_import.php");

    DEBUG_PRINT("  JSON payload length: ");
    DEBUG_PRINTLN(jsonPayload.length());
    DEBUG_PRINT("  HTTP begin Time: ");
    DEBUG_PRINTLN(millis() - t1);

    // If you need an HTTP request with a content type: application/json, use the following:
    http.addHeader("Content-Type", "application/json");
    
    t1 = millis();
    int httpResponseCode = http.POST(jsonPayload);

    DEBUG_PRINT("  POST Time: ");
    DEBUG_PRINTLN(millis() - t1);
    DEBUG_PRINTLN("  POSTing to Server:");
    DEBUG_PRINT("    ");
    DEBUG_PRINTLN(jsonPayload);
    DEBUG_PRINT("  HTTP response code: ");
    DEBUG_PRINTLN(httpResponseCode);
    if (!(httpResponseCode == 200))
    {
        DEBUG_PRINTLN("  *****ERROR*****");
        DEBUG_PRINTLN(http.errorToString(httpResponseCode));
    }

    // Free resources
    http.end();
   // WiFi.disconnect(true);

    delay(500); // Give resources a bit to free... 

    if (
        (httpResponseCode == 200) || 
        (httpResponseCode == 303) 
       )
    {
        return true;
    }
    else
    {
        return false;
    }

}

#endif //__DIGAME_NETWORK_H__
