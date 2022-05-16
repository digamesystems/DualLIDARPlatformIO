#ifndef __DUAL_LIDAR_H__
#define __DUAL_LIDAR_H__


#include <digameDebug.h>  // debug defines
#include <TFMPlus.h>      // Include TFMini Plus LIDAR Library v1.5.0
                          // https://github.com/budryerson/TFMini-Plus

#define NEITHER 0
#define SENSOR1 1
#define SENSOR2 2 
#define BOTH    3

class DualLIDAR{

  public: 
    DualLIDAR();
    ~DualLIDAR();

    uint8_t version[3]; // FW version
    uint8_t status;     // Error status

    bool begin(int t1, int r1, int t2, int r2); // Specify Pins
    bool begin();                               // Use Default Pins 
    
    bool getRanges(int16_t &dist1, int16_t &dist2);
    
    void setZone(int zoneMin, int zoneMax);
    void getZone(int &zoneMin, int &zoneMax);

    void  setSmoothingFactor(float newSmoothingFactor);
    float getSmoothingFactor();


    int  getVisibility();

  private: 
    TFMPlus tfmP_1;
    TFMPlus tfmP_2;

    float smoothingFactor = 0.95;
    float smoothedDist1 = 0;
    float smoothedDist2 = 0;

    int tx1=25, rx1=33, tx2=27, rx2=26; // Default pins for tx and rx
    int zoneMin = 0; 
    int zoneMax = 100;
    int visibility = NEITHER;
  
    void initLIDAR(TFMPlus &tfmP, int port=1);


};

#endif //__DUAL_LIDAR_H__
