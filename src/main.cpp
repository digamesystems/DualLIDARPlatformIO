#include <Arduino.h>

/*
    This program uses a pair of TFMini-plus LIDARs to count people boarding /
    unboarding shuttle buses. 
    
    The two sensors are used in combination to determine direction of travel and 
    thereby if a person is boarding or exiting.
    
    Data is reported in JSON format via Bluetooth classic. 
    
    LIDAR Sensor:
    http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html
    (See manual in /docs folder.)

    Written for the ESP32 WROOM Dev board V4 (Incl. WiFi, Bluetooth, and stacks of I/O.)
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitc-02.html

    Copyright 2021, Digame Systems. All rights reserved.

    Testing the Sensor: 
    https://youtu.be/LPxUawpHUEk
    https://youtu.be/7mM1T-jgChU
    https://youtu.be/dQw4w9WgXcQ

*/

#define HARDWARE_PRESENT true // Flag so we can debug the code without LIDAR sensors.

//****************************************************************************************
// Digame Includes. (Usually found in ...Arduino\libraries\DigameUtils folder)
//****************************************************************************************
#include <digameDebug.h>      // Serial debugging defines. 
#include <digameFile.h>       // Read/Write Text files.
#include <digameNetwork_v2.h> // For MAC address functions
#include <credentials.h>      // network name, pw, etc.

#include <BluetoothSerial.h>  // Part of the ESP32 board package. 
                              // By Evandro Copercini - 2018
#include <DualLIDAR.h>
DualLIDAR dL;

#include <SPIFFS.h>           // FLASH file system support.

// For Over the Air (OTA) updates... 
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h> // A lovely little library that provides a web 
                             // Interface for over the air updates of the firmware. 
                             // See: https://github.com/ayushsharma82/ElegantOTA

bool useOTA = true; 

//****************************************************************************************
//****************************************************************************************                        
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

BluetoothSerial btUART; // Create a BlueTooth Serial Port Object


//****************************************************************************************
//****************************************************************************************
// We're tracking the visibility of the target on both sensors. Valid events go from only 
// visible on one sensor to being visible on both. Direction is determined by which sensor
// sees the target first. 
int previousState     = 0; 
int state             = 0; 

const int INBOUND     = 0;
const int OUTBOUND    = 1; 

unsigned int inCount  = 0;
unsigned int outCount = 0;

String deviceName        = "Entrance 1";
float  distanceThreshold = 160;
float  smoothingFactor   = 0.95;

bool streamingRawData = false; 
bool menuActive       = false;
bool clearDataFlag    = false; 

String jsonPayload;
String jsonPrefix;  

//****************************************************************************************
// Overloading print and println to send messages to both serial and bluetooth connections
// There must be a better way to do this with a #define... 
//****************************************************************************************
void   dualPrintln(String s="");
void   dualPrint(String s="");
void   dualPrintln(float f);
void   dualPrint(float f);
void   dualPrintln(int i);
void   dualPrint(int i);


void   loadDefaults();

void   showSplashScreen();
void   showMenu();

String getUserInput();
void   scanForUserInput();
void   handleEvent(int eventType);

void   configureWiFi();
void   configureBluetooth();
void   configureLIDARs();
void   configureOTA();

void   buildJSONPrefix();


//****************************************************************************************                            
void setup() // - Device initialization
//****************************************************************************************
{
  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize
  
  loadDefaults();
  showSplashScreen();
  
  DEBUG_PRINTLN("INITIALIZING HARDWARE...");
  
  configureWiFi();
  configureBluetooth();
  configureLIDARs();

  if (useOTA) {
    configureOTA();  
  }

  buildJSONPrefix();
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");

}



//****************************************************************************************
void loop()  // Main 
//****************************************************************************************
{ 
  scanForUserInput();
  
  if (clearDataFlag){
    inCount = 0; 
    outCount = 0;
    clearDataFlag = false;    
  }

  int16_t dist1, dist2;
  
  dL.getRanges(dist1, dist2);
  
  state = dL.getVisibility();
  
  if (streamingRawData) {
    dualPrint(dist1);
    dualPrint(" ");
    dualPrint(dist2);
    dualPrint(" ");
    dualPrintln(100*state);
  }
  
  if (state == BOTH){ // Visible on both Sensors
  
    if ((previousState == SENSOR1)){ // INBOUND event
      handleEvent(INBOUND); 
    }
  
    if ((previousState == SENSOR2)){ // OUTBOUND event
      handleEvent(OUTBOUND);
    }
    
  }

  previousState = state;
}


//****************************************************************************************                            
void buildJSONPrefix() // The first part of all of our JSON messages
//****************************************************************************************                            
{ 
  jsonPrefix = "{\"deviceName\":\"" + deviceName + "\",\"deviceMAC\":\"" + WiFi.macAddress();
}


//****************************************************************************************
void configureWiFi(){
//****************************************************************************************
  DEBUG_PRINTLN("  WiFi...");
}


//****************************************************************************************
void configureBluetooth(){
//****************************************************************************************
  DEBUG_PRINTLN("  Bluetooth...");
  btUART.begin("Counter_" + getShortMACAddress()); // My Bluetooth device name 
                                  //  TODO: Provide opportunity to change names. 
  delay(1000);                    // Give port time to initalize
    
}


//****************************************************************************************
void configureLIDARs(){
//**************************************************************************************** 
  DEBUG_PRINTLN("  Configuring LIDARS...");
  
  #if HARDWARE_PRESENT
    dL.begin(25,33,27,26); // TX/RX pin numbers for the two LIDARs
  #endif

}


//***************************************************************************************
String processor(const String& var)
//***************************************************************************************
{
  if(var == "config.deviceName") return deviceName;
  return "";
}


//****************************************************************************************
void configureOTA(){
//****************************************************************************************

  DEBUG_PRINTLN("  Stand-Alone Mode. Setting AP (Access Point)…");  
  WiFi.mode(WIFI_AP);
  
  String netName = "Counter_" + getShortMACAddress();
  const char* ssid = netName.c_str();
  WiFi.softAP(ssid);
  
  IPAddress IP = WiFi.softAPIP();
  DEBUG_PRINT("    AP IP address: ");
  DEBUG_PRINTLN(IP);   
  
  //delay(3000);  
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate("admin", "admin"))
      return request->requestAuthentication();
      
    request->send(SPIFFS, "/index.html", String(), false, processor);
  
  });

  server.serveStatic("/", SPIFFS, "/"); // sets the base path for the web server
  AsyncElegantOTA.begin(&server);   
  server.begin();
}

  
//****************************************************************************************
// Simultaneous Print functions. Bluetooth and Serial. 
// TODO: find a better way to do this with #define macro... 
//****************************************************************************************
void dualPrintln(String s){
  DEBUG_PRINTLN(s);
  btUART.println(s);  
}

void dualPrint(String s){
  DEBUG_PRINT(s);
  btUART.print(s);  
}

void dualPrintln(float f){
  DEBUG_PRINTLN(f);
  btUART.println(f);  
}

void dualPrint(float f){
  DEBUG_PRINT(f);
  btUART.print(f);  
}

void dualPrintln(int i){
  DEBUG_PRINTLN(i);
  btUART.println(i);  
}

void dualPrint(int i){
  DEBUG_PRINT(i);
  btUART.print(i);  
}


//****************************************************************************************
void showSplashScreen(){
//****************************************************************************************
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  dualPrintln();
  dualPrintln("*******************************************");
  dualPrintln("ParkData Directional LIDAR Sensor");
  dualPrintln("Version 1.0");
  dualPrintln("");
  dualPrintln("Compiled: " + compileDate + " at " + compileTime); 
  dualPrint("Device Name: ");
  dualPrintln(deviceName);
  dualPrint("Bluetooth Address: Counter_");
  dualPrintln(String(getShortMACAddress()));
  dualPrintln();
  dualPrintln("Copyright 2022, Digame Systems.");
  dualPrintln("All rights reserved.");
  dualPrintln("*******************************************");
  dualPrintln();   
}


//****************************************************************************************
void showMenu()
//****************************************************************************************
{
  if (menuActive){
    showSplashScreen();   
    dualPrintln("MENU: ");
    dualPrintln("  [+][-]Menu Active");
    dualPrintln("  [n]ame               (" + deviceName +")");
    dualPrintln("  [d]istance threshold (" + String(distanceThreshold) + ")");
    dualPrintln("  [s]moothing factor   (" + String(smoothingFactor) + ")");
    dualPrintln("  [g]et count data");
    dualPrintln("  [c]lear count data");
    dualPrintln("  [r]aw data stream    (" + String(streamingRawData) + ")");
    dualPrintln("  [x]eXit and reboot");  
    dualPrintln();
  }
}


//****************************************************************************************                            
void loadDefaults(){
//****************************************************************************************                            
  if(!SPIFFS.begin()){
    DEBUG_PRINTLN("    File System Mount Failed");
  } else {
    //DEBUG_PRINTLN("    SPIFFS up!");
    String temp;
    
    temp = readFile(SPIFFS, "/name.txt");
    if (temp.length() >0) deviceName  = temp;
    
    temp = readFile(SPIFFS, "/smooth.txt");
    if (temp.length() > 0) smoothingFactor = temp.toFloat();
    dL.setSmoothingFactor(smoothingFactor);
    
    temp = readFile(SPIFFS, "/threshold.txt");
    if (temp.length() > 0) distanceThreshold = temp.toFloat();
    dL.setZone(0,distanceThreshold);
  }
}



//****************************************************************************************
String getUserInput() { // -- No range checking!
//****************************************************************************************
  String inString;

  while (!(Serial.available()) && !(btUART.available())){
    delay(10);
    //lightSleepMSec(10);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
  }  

  if ( Serial.available() ) inString = Serial.readStringUntil('\n');
  if ( btUART.available() ) inString = btUART.readStringUntil('\n');
  
  inString.trim();
  dualPrint(" You entered: ");
  dualPrintln(inString);
  return inString;  

}


//****************************************************************************************
void scanForUserInput()
//****************************************************************************************
{
  String inString;
  bool inputReceived=false;
 
  inputReceived = false;
  
  if (Serial.available()){
    inString = Serial.readStringUntil('\n');
    inputReceived = true;
  }  

  if (btUART.available()){
    inString = btUART.readStringUntil('\n');
    inputReceived = true;
  }
  
  if (inputReceived) {  
    inString.trim();
   
     if(inString == "g"){
      String jsonPayload = jsonPrefix + "\",\"inbound\":\""  + inCount  + "\"" + 
                                          ",\"outbound\":\"" + outCount + "\"" + "}";
      dualPrintln(jsonPayload);
      return;
    }

    if (inString == "+") {
      dualPrintln("OK");
      menuActive = true;
    }

    if (inString == "-") {
      dualPrintln("OK");
      menuActive = false;
    }
    
    if (inString == "n") {
      dualPrintln(" Enter New Device Name. (" + deviceName +")");
      deviceName = getUserInput();
      buildJSONPrefix(); // The device name is part of the prefix.
      dualPrint(" New Device Name: ");
      dualPrintln(deviceName);
      writeFile(SPIFFS, "/name.txt", deviceName.c_str());
    } 
    
    if(inString == "d"){
      dualPrintln(" Enter New Distance Threshold. (" + String(distanceThreshold) +")");
      distanceThreshold = getUserInput().toFloat();
      dualPrint(" New distanceThreshold: ");
      dualPrintln(distanceThreshold);
      dL.setZone(0,distanceThreshold);
      writeFile(SPIFFS, "/threshold.txt", String(distanceThreshold).c_str());
    } 
    
    if(inString == "s"){
      dualPrintln(" Enter New Smoothing Factor. (" + String(smoothingFactor) + ")");
      smoothingFactor = getUserInput().toFloat();
      dualPrint(" New Smoothing Factor: ");
      dualPrintln(smoothingFactor);
      dL.setSmoothingFactor(smoothingFactor);
      writeFile(SPIFFS, "/smooth.txt", String(smoothingFactor).c_str());
    } 

    if(inString == "c"){
      dualPrintln("OK");
      clearDataFlag = true;
    }      

    if(inString == "r"){
      streamingRawData = (!streamingRawData);
    } 

    if(inString =="x"){
      dualPrintln();
      dualPrintln("Rebooting NOW...");
      dualPrintln();
      
      ESP.restart();  
    }
    
    if (!streamingRawData) showMenu(); 
  }   
}

//****************************************************************************************
void handleEvent(int eventType)
//****************************************************************************************
{
  String eventString; 
  
  jsonPayload = jsonPrefix; 
  
  if (eventType == INBOUND) {
    inCount += 1;
    jsonPayload = jsonPayload + "\",\"eventType\":\"inbound" +
                 "\",\"count\":\"" + inCount + "\"" +
                 "}";
  } 
  if (eventType == OUTBOUND) {
    outCount += 1;
    jsonPayload = jsonPayload + "\",\"eventType\":\"outbound" +
                 "\",\"count\":\"" + outCount + "\"" +
                 "}";
  }
  
  if ((!streamingRawData)&&(menuActive)) dualPrintln(jsonPayload);  
  
}

