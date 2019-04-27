/********
Default E80 Code
Current Author:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
Previous Contributors:
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)                    
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <StateEstimator.h>
#include <ADCSampler.h>
#include <ErrorFlagSampler.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <PControl.h>
#define UartSerial Serial1
#include <GPSLockLED.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
StateEstimator state_estimator;
PControl pcontrol;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ADCSampler adc;
ErrorFlagSampler ef;
ButtonSampler button_sampler;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
volatile bool EF_States[NUM_FLAGS] = {1,1,1};


const int number_of_waypoints = 11;
const int waypoint_dimensions = 4;       // waypoints are set to have 4 pieces of information, x then y then depth then time to stay at waypoint.
double waypoints [] = { 10,    0,   10,    5000,
                        10,    0,    0,    0,
                       -10,    0,   -1,    0};
/*double waypoints [] = { 10,    0,   -1,    0,
                        10,    0,   50,    1000,
                        10,    0,    0,    1000,
                        10,    0,  100,    1000,
                        10,    0,    0,    0,
                        15,    0,   -1,    0,
                        15,    0,  100,    2000,
                        15,    0,    0,    0,
                       -10,    0,   -1,    0};*/
// ^ listed as x0,y0,z0,t0,x1,y1,z1,t1, ... etc.
// x y in meters, depth in cm, time in milliseconds (# seconds * 1000)

////////////////////////* Setup *////////////////////////////////

void setup() {
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&pcontrol);
  logger.include(&motor_driver);
  logger.include(&adc);
  logger.include(&ef);
  logger.include(&button_sampler);
  logger.init();

  printer.init();
  ef.init();
  button_sampler.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();
  pcontrol.init(number_of_waypoints, waypoint_dimensions, waypoints, millis());
  
  state_estimator.init(); 

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  ef.lastExecutionTime              = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
  button_sampler.lastExecutionTime  = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + STATE_ESTIMATOR_LOOP_OFFSET;
  pcontrol.lastExecutionTime        = loopStartTime - LOOP_PERIOD + P_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}

//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();
  
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,adc.printSample());
    printer.printValue(1,adc.printEnvironmentalInfo());
    printer.printValue(2,ef.printStates());
    printer.printValue(3,logger.printState());
    printer.printValue(4,gps.printState());   
    printer.printValue(5,state_estimator.printState());
    printer.printValue(6,pcontrol.printWaypointUpdate());
    printer.printValue(7,pcontrol.printString());
    printer.printValue(8,pcontrol.printString2());
    printer.printValue(9,motor_driver.printState());
    printer.printValue(10,imu.printRollPitchHeading());        
    printer.printValue(11,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-pcontrol.lastExecutionTime > LOOP_PERIOD ) {
    pcontrol.lastExecutionTime = currentTime;
    pcontrol.calculateControl(&state_estimator.state, &gps.state, millis());
    motor_driver.drive(pcontrol.uV,pcontrol.uL,pcontrol.uR);
  }

  if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }

  if ( currentTime-ef.lastExecutionTime > LOOP_PERIOD ) {
    ef.lastExecutionTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
    delay(5);
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
    ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
    EF_States[0] = 1;
    EF_States[1] = 1;
    EF_States[2] = 1;
  }

 // uses the ButtonSampler library to read a button -- use this as a template for new libraries!
  if ( currentTime-button_sampler.lastExecutionTime > LOOP_PERIOD ) {
    button_sampler.lastExecutionTime = currentTime;
    button_sampler.updateState();
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }
 
  if (true){//if ( currentTime-gps.lastExecutionTime > LOOP_PERIOD ) {
    gps.lastExecutionTime = currentTime;
    gps.read(&GPS); // blocking UART calls
  }

  if ( currentTime-state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    state_estimator.lastExecutionTime = currentTime;
    state_estimator.updateState(&imu.state, &gps.state, &adc);
  }
  
  if ( currentTime-led.lastExecutionTime > LOOP_PERIOD ) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if ( currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}

void EFA_Detected(void){
  EF_States[0] = 0;
}

void EFB_Detected(void){
  EF_States[1] = 0;
}

void EFC_Detected(void){
  EF_States[2] = 0;
}
