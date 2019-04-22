#ifndef __PCONTROL_H__
#define __PCONTROL_H__

#define SUCCESS_RADIUS 6.0 // success radius in meters
#define SUCCESS_RADIUS_DIVE 10
#define UP_MOTOR_DEFAULT 100

#define ABORT_TIME 300000//in ms

#include <Arduino.h>
#include "MotorDriver.h"
#include "StateEstimator.h"
extern MotorDriver motorDriver;

class PControl : public DataSource
{
public:
  PControl(void);

  // defines the waypoints used for pControl
  void init(const int totalWayPoints_in, const int stateDims_in, double * wayPoints_in, unsigned long startT);

  // sets the motor speeds using P-Control
  void calculateControl(state_t * state, gps_state_t * gps_state_p, unsigned long currentTime);

  String printString(void);
  String printString2(void);

  String printWaypointUpdate(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

  // control fields
  float yaw_des;         // desired yaw
  float yaw;             // current yaw
  float dist;            // distance to waypoint
  float u;               // control effort
  float Kp=10.0;         // proportional control gain
  float Kr=1.0;          // right motor gain correction
  float Kl=1.0;          // left motor gain correction
  float Kv=20.0;
  float avgPower = 20.0; // average forward thrust
  float uR;             // right motor effort
  float uL;             // left motor effort
  float depth;
  float depth_des;
  float uV;              // up/down motor effort


private:

  // updates the current waypoint if necessary
  void updatePoint(float x, float y);
  unsigned long startTime;

  int getWayPoint(int dim);

  int totalWayPoints, stateDims;
  double * wayPoints;
  int currentWayPoint = 0;
  bool gpsAcquired;
};

#endif
