#include <digameDebug.h>  // debug defines
#include <TFMPlus.h>      // Include TFMini Plus LIDAR Library v1.5.0
                          // https://github.com/budryerson/TFMini-Plus

#include <DualLIDAR.h> 


#define tfMiniUART_1 Serial1
#define tfMiniUART_2 Serial2

DualLIDAR::DualLIDAR(){}
DualLIDAR::~DualLIDAR(){}


//****************************************************************************************
bool DualLIDAR::begin(int t1, int r1, int t2, int r2)
//****************************************************************************************
{
  tx1 = t1;
  rx1 = r1;
  tx2 = t2;
  rx2 = r2;

  tfMiniUART_1.begin(115200,SERIAL_8N1,tx1,rx1);
  delay(100);
  initLIDAR(tfmP_1, 1);

  tfMiniUART_2.begin(115200,SERIAL_8N1,tx2,rx2);
  delay(100);
  initLIDAR(tfmP_2, 2);

  int16_t d1, d2;

  getRanges(d1,d2);
  smoothedDist1 = d1;
  smoothedDist2 = d2;

  return true; //fix.
}


//****************************************************************************************
bool DualLIDAR::begin()
//****************************************************************************************
{
  return begin(tx1,rx1,tx2,rx2);
}
    

//****************************************************************************************
bool DualLIDAR::getRanges( int16_t &dist1, int16_t &dist2)
//****************************************************************************************
{
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  
  // Read the LIDAR Sensor
  if( tfmP_1.getData( tfDist, tfFlux, tfTemp)) { 
    dist1 = tfDist;
  } else { return false; }
 
  if( tfmP_2.getData( tfDist, tfFlux, tfTemp)) { 
    dist2 = tfDist;
  } else { return false; }

  smoothedDist1 = smoothedDist1 * smoothingFactor + (float)dist1 * (1-smoothingFactor);
  dist1 = smoothedDist1;

  smoothedDist2 = smoothedDist2 * smoothingFactor + (float)dist2 * (1-smoothingFactor);
  dist2 = smoothedDist2;

  visibility = 0; 

  if ((dist1>=zoneMin)&&(dist1<=zoneMax)){
    visibility +=1;
  }

  if ((dist2>=zoneMin)&&(dist2<=zoneMax)){
    visibility +=2;
  }

  return true;
}



void DualLIDAR::setZone(int zMin, int zMax)
{
  zoneMin = zMin;
  zoneMax = zMax;
}

void DualLIDAR::getZone(int &zMin, int &zMax)
{
  zMin = zoneMin;
  zMax = zoneMax;
}

void DualLIDAR::setSmoothingFactor(float newSmoothingFactor)
{
  smoothingFactor = newSmoothingFactor;
}

float DualLIDAR::getSmoothingFactor()
{
  return smoothingFactor;
}

int DualLIDAR::getVisibility()
{
  return visibility;
}


//**************************************************************************************** 
void DualLIDAR::initLIDAR(TFMPlus &tfmP, int port) // Initialize a LIDAR sensor on a 
                                                   // serial port.
//****************************************************************************************
{
  // Initialize a TFminiPlus class and pass a serial port to the object.
  if (port == 1) {
    tfmP.begin(&tfMiniUART_1);   
  }
  else if (port == 2) {
    tfmP.begin(&tfMiniUART_2);   
  }
  else {
    DEBUG_PRINTLN("Unknown Port. I give up.");
    while (1){} // Nothing to do if we don't have a valid port.  
    return;
  }

  // Send some commands to configure the TFMini-Plus
  // Perform a system reset
  DEBUG_PRINT( "    Activating LIDAR Sensor... ");
  if( tfmP.sendCommand(SOFT_RESET, 0)) {
      DEBUG_PRINTLN("Sensor Active.");
  }
  else{
    DEBUG_PRINTLN("     TROUBLE ACTIVATING LIDAR!");                    
    tfmP.printReply();
  }

  delay(1000);

  // Set the acquisition rate to 100 Hz.
  DEBUG_PRINT( "    Adjusting Frame Rate... ");
  if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_100)){
      DEBUG_PRINTLN("Frame Rate Adjusted.");
  }
  else tfmP.printReply();

}


