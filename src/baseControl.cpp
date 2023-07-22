#include "main.h"
#include "baseControl.hpp"

#define defaultMoveKP 15 //12 at 20 inches
#define defaultMoveKD 2
#define defaultTurnKP 2 //2.25 for 45, 2 for 60, 1.3 for 90
#define defaultTurnKD 0.1
#define distanceLeeway 0.5
#define bearingLeeway 0.5
#define velocityLeeway 2
#define rampPower 3 //5
#define defaultMaxPower 110

double abscap(double x, double abscap){return x>abscap? abscap:x<-abscap? -abscap:x;}

double targL = 0, targR = 0, targBearing = 0;
double errorL = 0, errorR = 0, errorBearing = 0;
double powerL = 0, powerR = 0;
double targPowerL = 0, targPowerR = 0;
double kP = defaultMoveKP, kD = defaultMoveKD;
double maxPower = defaultMaxPower;

bool turnMode = false, pauseBase = false;
bool auton = true;
bool poweredBase = false;

void waitBase(double cutoff){
	Motor frontLeft(frontLeftPort);
	Motor midLeft(midLeftPort);
	Motor backLeft(backLeftPort);
	Motor frontRight(frontRightPort);
	Motor midRight(midRightPort);
	Motor backRight(backRightPort);
	double start = millis();
	delay(20);
  if(turnMode) {while(fabs(errorBearing) > bearingLeeway && (millis()-start) < cutoff) delay(20);}
  else{while((fabs(errorL) > distanceLeeway || fabs(errorR) > distanceLeeway||fabs(frontLeft.get_actual_velocity())>velocityLeeway||fabs(frontRight.get_actual_velocity())>velocityLeeway||fabs(backLeft.get_actual_velocity())>velocityLeeway||fabs(backRight.get_actual_velocity())>velocityLeeway||fabs(midLeft.get_actual_velocity())>velocityLeeway||fabs(midRight.get_actual_velocity())>velocityLeeway) && (millis()-start) < cutoff)delay(20);}
	double end = millis();
	printf("end: errorL %.2f, errorR %.2f, time %.2f\n",errorL,errorR,end-start);
  targL = posL;
  targR = posR;
	pauseBase = true;
}

void baseMove(double dist, double cutoff, double moveP, double moveD){
	poweredBase = false;
  turnMode = false;
  targL += dist;
  targR += dist;

  kP = moveP;
  kD = moveD;

	printf("START: baseMove distance: %.2f cutoff %.2f moveP %.2f moveD %.2f, targL %.2f, targR %.2f \n",dist, cutoff, moveP, moveD, targL, targR);
	pauseBase = false;
  waitBase(cutoff);
	printf("END: baseMove distance: %.2f cutoff %.2f moveP %.2f moveD %.2f, targL %.2f, targR %.2f \n",dist, cutoff, moveP, moveD, targL, targR);
	pauseBase = true;
}

void baseMove(double dist, double cutoff){baseMove(dist, cutoff, defaultMoveKP, defaultMoveKD);}

void baseTurn(double angle, double cutoff, double turnP, double turnD){
	poweredBase = false;
  turnMode = true;
  targBearing = angle;

  kP = turnP;
  kD = turnD;

	printf("START: %.2f cutoff %.2f turnP %.2f turnD %.2f \n", bearing, cutoff, turnP, turnD);
	pauseBase = false;
  waitBase(cutoff);
	printf("END: %.2f cutoff %.2f turnP %.2f turnD %.2f \n", bearing, cutoff, turnP, turnD);
	pauseBase = true;
}

void baseTurn(double angle, double cutoff){baseTurn(angle, cutoff, defaultTurnKP, defaultTurnKD);}

void baseControl(void * ignore){
  Motor frontLeft (frontLeftPort);
  Motor midLeft (midLeftPort);
  Motor backLeft (backLeftPort);
  Motor frontRight (frontRightPort);
  Motor midRight (midRightPort);
  Motor backRight (backRightPort);
  Imu imu (imuPort);
  double prevErrorL = 0, prevErrorR = 0, prevErrorBearing = 0;
  double deltaErrorBearing, deltaErrorL, deltaErrorR;
  double deltaPowerL = 0, deltaPowerR = 0;
	poweredBase = false;
  while(true){
    if(!imu.is_calibrating()) {
			if(!pauseBase){
	      if(turnMode){
	        errorBearing = targBearing - bearing;
	        deltaErrorBearing = errorBearing - prevErrorBearing;
	        targPowerL = errorBearing * kP + deltaErrorBearing * kD;
	        targPowerR = -targPowerL;
	        prevErrorBearing = errorBearing;
	      }
	      else{
	        errorL = targL - posL;
	        errorR = targR - posR;
	        deltaErrorL = errorL - prevErrorL;
	        deltaErrorR = errorR - prevErrorR;
	        targPowerL = errorL * kP + deltaErrorL * kD;
	        targPowerR = errorR * kP + deltaErrorR * kD;
	        prevErrorL = errorL;
	        prevErrorR = errorR;
	      }
	      deltaPowerL = targPowerL - powerL;
	      powerL += abscap(deltaPowerL, rampPower);
	      deltaPowerR = targPowerR - powerR;
	      powerR += abscap(deltaPowerR, rampPower);
	      powerL = abscap(powerL, maxPower);
	      powerR = abscap(powerR, maxPower);

				printf("time: %.d\t",millis());
				printf("targL %.2f, targR %.2f\n",targL,targR);
				printf("errorL %.2f ErrorR %.2f\t",errorL,errorR);
				printf("prevErrorL %.2f prevErrorR %.2f\t",prevErrorL,prevErrorR);
				printf("posL %.2f, posR %.2f\td",posL,posR);
				printf("powerL %.2f, powerR %.2f\n",powerL,powerR);

				//printf("targBearing %.2f, errorBearing %.2f, prevErrorBearing %.2f, bearing %.2f, targPowerL %.2f, powerL %.2f\n",targBearing,errorBearing,prevErrorBearing,bearing,targPowerL,powerL);
    	}
			else{
				powerL = 0;
				powerR = 0;
			}
		}
		if(!poweredBase){
	    frontLeft.move(powerL);
	    midLeft.move(powerL);
	    backLeft.move(powerL);
	    frontRight.move(powerR);
	    midRight.move(powerR);
	    backRight.move(powerR);
		}
		//printf("powerL %.2f, powerR %.2f\n",powerL,powerR);
    delay(5);
  }
}

void printingSensors(void * ignore){
	Imu imu(imuPort);
	while(competition::is_autonomous()){
		double time = millis();
		printf("time %.2f\t",time);
		if(!imu.is_calibrating()){
			if(turnMode){printf("errorBearing %.2f\t",errorBearing);}
			else{printf("errorL %.2f, errorR %.2f\t",errorL,errorR);}
			printf("left %.2f, right %.2f, bearing %.2f\t",posL,posR,bearing);
		}
		else{printf("imu is calibrating...\n");}
		printf("\n");
		delay(20);
	}
}

void powerBase(double power){
	Motor frontLeft (frontLeftPort);
	Motor midLeft (midLeftPort);
	Motor backLeft (backLeftPort);
	Motor frontRight (frontRightPort);
	Motor midRight (midRightPort);
	Motor backRight (backRightPort);
	poweredBase = true;
	frontLeft.move(power);
	midLeft.move(power);
	backLeft.move(power);
	frontRight.move(power);
	midRight.move(power);
	backRight.move(power);
}

void changeMaxPower(double power){maxPower = power;}
