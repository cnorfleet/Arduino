#include "PControl.h"
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

PControl::PControl(void) 
: DataSource("u,uL,uR,yaw,yaw_des","float,float,float,float,float"){}


void PControl::init(const int totalWayPoints_in, const int stateDims_in, double * wayPoints_in, unsigned long startT) {
  totalWayPoints = totalWayPoints_in;
  stateDims = stateDims_in;
  wayPoints = wayPoints_in;
  startTime = startT;
}

int PControl::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}

void PControl::calculateControl(state_t * state, gps_state_t * gps_state_p, unsigned long currentTime) {
	int x_des = getWayPoint(0);
	int y_des = getWayPoint(1);
	depth_des = getWayPoint(2);
	
	depth = state->depth;
	
	if(getWayPoint(2) == -1) {
	  uV = UP_MOTOR_DEFAULT;
	  if (gps_state_p->num_sat >= N_SATS_THRESHOLD){
		gpsAcquired = 1;

		updatePoint(state->x, state->y);
		if (currentWayPoint == totalWayPoints) return; // stops motors at final point

		// Set the values of yaw_des, yaw, control effort (u), uL, and uR appropriately
		// You can use trig functions (atan2 might be useful)
		// You can access the x and y coordinates calculated in StateEstimator.cpp using state->x and state->y respectively
		// You can access the heading calculated in StateEstimator.cpp using state->heading
		
		yaw_des = atan2(y_des - state->y, x_des - state->x);
		yaw = state->yaw;
		u = Kp*angleDiff(yaw_des - yaw);

		uL = max(0.0,min(255.0,(avgPower - u)*Kl));
		uR = max(0.0,min(255.0,(avgPower + u)*Kr));
	  }
	  else{
		gpsAcquired = 0;
	  }
	}
	else {
		uL = 0;
		uR = 0;
		uV = Kv * (depth - depth_des);
	}
	
	if(currentTime > startTime + ABORT_TIME) {
		uV = 255;
	}
}

String PControl::printString(void) {
	String printString = "";
	printString += "PControl: ";
	printString += "Yaw: ";
	printString += String(yaw*180.0/PI);
	printString += "[deg] ";
	printString += "(Des: ";
	printString += String(yaw_des*180.0/PI);
	printString += "[deg]), ";
	printString += "Depth: ";
	printString += String(depth);
	printString += "[cm] ";
	printString += "(Des: ";
	printString += String(depth_des);
	printString += "[cm])";
	return printString;
}

String PControl::printString2(void) {
  String printString = "";
  //if(!gpsAcquired && (getWayPoint(2) == -1)){
  //  printString += "PControl: Waiting to acquire more satellites...";
  //}
  //else{
    printString += "PControl: ";
    printString += "u: ";
    printString += String(u);
    printString += ", u_L: ";
    printString += String(uL);
    printString += ", u_R: ";
    printString += String(uR);
    printString += ", u_V: ";
    printString += String(uV);
  //}
  return printString;
}

String PControl::printWaypointUpdate(void) {
  String wayPointUpdate = "";
  if(!gpsAcquired && (getWayPoint(2) == -1)){
    wayPointUpdate += "PControl: Waiting to acquire more satellites...";
  }
  else{
    wayPointUpdate += "PControl: ";
    wayPointUpdate += "Current Waypoint: ";
    wayPointUpdate += String(currentWayPoint);
    wayPointUpdate += " = ";
    wayPointUpdate += String(getWayPoint(0));
    wayPointUpdate += ",";
    wayPointUpdate += String(getWayPoint(1));
    wayPointUpdate += ",";
    wayPointUpdate += String(getWayPoint(2));
    wayPointUpdate += ". Distance from Waypoint: ";
    wayPointUpdate += String(dist);
    wayPointUpdate += "[m]";
  }
  return wayPointUpdate;
}

void PControl::updatePoint(float x, float y) {
	if (currentWayPoint == totalWayPoints) return; // don't check if finished

	int x_des = getWayPoint(0);
	int y_des = getWayPoint(1);
	int depth_des = getWayPoint(2);
	
	if(currentWayPoint < totalWayPoints) {
		if(depth_des == -1) {
			dist = sqrt(pow(x-x_des,2) + pow(y-y_des,2));
		} else {
			dist = abs(depth-depth_des);
		}
		
		if((depth_des == -1 && dist < SUCCESS_RADIUS) ||
		   (depth_des != -1 && dist < SUCCESS_RADIUS_DIVE)) {
		String changingWPMessage = "Got to waypoint " + String(currentWayPoint)
		  + ", now directing to next point";
		int cwpmTime = 20;
		currentWayPoint++;
		if (currentWayPoint == totalWayPoints) {
		  changingWPMessage = "Congratulations! You completed the path! Stopping motors.";
		  uR=0;
		  uL=0;
		  uV=UP_MOTOR_DEFAULT;
		  cwpmTime = 0;
		}
		printer.printMessage(changingWPMessage,cwpmTime);
	}
	}
}

size_t PControl::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = u;
  data_slot[1] = uL;
  data_slot[2] = uR;
  data_slot[3] = yaw;
  data_slot[4] = yaw_des;
  return idx + 5*sizeof(float);
}
