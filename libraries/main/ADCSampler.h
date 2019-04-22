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

#define teensyUToVisLight 110.6623 // (200 * 1000000/5830 * 3.3/1023)
#define teensyUToIRLight  3.2258   // (80  *    1000/80   * 3.3/1023)

class ADCSampler : public DataSource
{
public:
  ADCSampler(void);

  void init();

  // Managing state
  int sample [NUM_PINS];
  int readDepth;
  int IRVolts;      // in teensy units
  float IRLight;    // in lux
  int greenVolts;   // in teensy units
  float greenLight; // in lux
  float turbidity;  // in ntu
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