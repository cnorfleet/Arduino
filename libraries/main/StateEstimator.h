#ifndef __STATE_ESTIMATOR_H__
#define __STATE_ESTIMATOR_H__

#include <Arduino.h>

#include <SensorGPS.h>
#include <SensorIMU.h>
#include <ADCSampler.h>
#include "DataSource.h"

#define RADIUS_OF_EARTH_M 6371000 // [m]

// teensyUnits to depth in meters calibration curve values
#define depthConvertIntercept -210.13
#define depthConvertSlope     1.5241

// using light for depth calibration curve values:
// depth [cm] = a * ln (b * (IR [teensy]) / (Vis [teensy]))
#define lightRatio_a 1 ///////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define lightRatio_b 1

typedef struct {
  float x = 0; // x position in global frame [m]
  float y = 0; // y position in global frame [m]
  float depth = 0; // below water surface [m]
  float yaw = 0; // yaw in global frame [rad] CCW from magnetic east
} state_t;

/*
 * StateEstimator class keeps track of the robot's state, incorporating
 * measurements of the system outputs from the various sensors like IMU or
 * GPS as well as the control inputs to the system.
 */
class StateEstimator : public DataSource
{
public:
  StateEstimator(void);

  // init
  void init();

  // State Access
  state_t state;
  void updateState(imu_state_t * imu_state_p, gps_state_t * gps_state_p, ADCSampler * adc_sampler);
  String printState(void);

  void latlonToXY(double lat, double lon, float* x_p, float* y_p);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  // set coordinates of chosen origin below
  const float origin_lat = 34.1093063; // 34.1095009;   // 34.106465;
  const float origin_lon = -117.7127151; // -117.7128448; // -117.712488;
  bool gpsAcquired;
  
  float lightRatio = 0; //ratio of green to IR light (in terms of irradiance)
};

#endif
