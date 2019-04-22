#ifndef __ADCSAMPLER_h__
#define __ADCSAMPLER_h__

#include <Arduino.h>
#include "MotorDriver.h"
#include "DataSource.h"
#include "Pinouts.h"

/*
 * ADCSampler implements SD logging for the ADC channels
 */

#define NUM_PINS 16

#define numberOfTurbidityPoints 50
#define turbiditySlope   14471.780
#define turbidityIntercept -60.492

class ADCSampler : public DataSource
{
public:
  ADCSampler(void);

  void init();

  // Managing state
  int sample [NUM_PINS];
  int readDepth;
  int IRVolts;
  int greenVolts;
  float turbidity; // in ntu
  void updateSample(void);
  String printSample(void);
  String printEnvironmentalInfo(void);

  // Write out
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:

  const int pinMap[NUM_PINS] =  {21,14,15,16,17,34,35,36,37,40,26,27,28,29,30,31};
  
  float calcTurbAvg(float * lastVals);
  float calcTurb();
  
  int currentTurbidityIdx = 0;
  float lastTurb90[numberOfTurbidityPoints]  = {0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0};
  float lastTurb180[numberOfTurbidityPoints] = {0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0,
												0,0,0,0,0,0,0,0,0,0};
};
#endif