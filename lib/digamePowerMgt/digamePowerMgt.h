/*
  Power management functions for the ESP32 platform
*/
#ifndef __DIGAME_POWER_MGT_H__
#define __DIGAME_POWER_MGT_H__

#include <digameDebug.h>  // DEBUG_PRINT functions
#include <WiFi.h>         // WiFi stack
#include "driver/adc.h"   // ADC functions. (Allows us to turn off to save power.)
#include <esp_bt.h>       // Bluetooth control functions
#include <esp_wifi.h>

#define LOW_POWER    1
#define MEDIUM_POWER 2
#define FULL_POWER   3

int powerMode = FULL_POWER;
//*****************************************************************************
// Sleep the ESP32 for a while
void lightSleepMSec(unsigned long ms){
  unsigned long mS_TO_S_FACTOR = 1000;
  esp_sleep_enable_timer_wakeup(ms * mS_TO_S_FACTOR);
  esp_light_sleep_start(); 
}

//*****************************************************************************
// Disable Bluetooth, the ADC sub-system, WiFi and drop the CPU down to 40Mhz.
void setLowPowerMode() {
    DEBUG_PRINT("  Switching to Low Power Mode... ");
    delay(500);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    // adc_power_off(); Depricated
    adc_power_release();
    esp_wifi_stop();
    esp_bt_controller_disable();
    delay(500);
    DEBUG_PRINTLN("(40 MHz)");
    delay(500);
    setCpuFrequencyMhz(40); // Slow down the CPU
    delay(500);
    /*DEBUG_PRINTLN("    Done. Low Power Mode Enabled.");
    delay(500);
    */
    powerMode = LOW_POWER;
}

//*****************************************************************************
// Disable Bluetooth, the ADC sub-system, and drop the CPU down to 80MHz.
// Leave WiFi active. 
void setMediumPowerMode() {
    DEBUG_PRINT("  Switching to Medium Power Mode... ");
    delay(500);
    btStop();
    //adc_power_off();
    esp_bt_controller_disable();
    setCpuFrequencyMhz(80); // Slow down the CPU
    //delay(500);
    //DEBUG_PRINTLN("    Done. Medium Power Mode Enabled.");
    delay(500);
    powerMode = MEDIUM_POWER;
}

//***************************************************************************** 
// Run the CPU Flat out at 240MHz w/ WiFi active. Turn off and Bluetooth.
void setFullPowerMode() {
    DEBUG_PRINTLN("  Switching to Full Power Mode... ");
    delay(500);
    setCpuFrequencyMhz(240); // Speed up the CPU
    btStop();
    esp_bt_controller_disable();
    //adc_power_on();          // Turn on the ADCs for WiFi (Depricated)
    adc_power_acquire();
    delay(500);
    DEBUG_PRINTLN("    Done. Full Power Mode Enabled. ");
    delay(500);
    powerMode = FULL_POWER;
}


#endif 

