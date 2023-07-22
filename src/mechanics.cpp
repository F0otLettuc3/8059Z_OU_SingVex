#include "main.h"
#define catapultP 0.027 //0.023 //0.017
#define catapultD 0.012 //0.001

Motor catapultLeft(catapultLeftPort);
Motor catapultRight(catapultRightPort);
Rotation catapultSensor(catapultSensorPort);
Controller master(E_CONTROLLER_MASTER);

double catapultTarg = downTarg, catapultPower = 0, prevError = 0;
bool catapultManual = false, catapultMoving;
void catapultControl(void*ignore){
  while(true){
    if(catapultManual==false){
      double catapultPosition = catapultSensor.get_angle();
      double catapultError = catapultTarg - catapultPosition;
      if((prevError-fabs(catapultError))>(35000)||fabs(catapultError)<50){catapultMoving = false;}
      else{catapultMoving=true;}
      if(catapultError<0){catapultError = (36000)-fabs(catapultError);}
      if(catapultMoving==true){
        catapultPower = catapultError*catapultP + (catapultError-prevError)*catapultD;
        if(catapultPower>127){catapultPower=127;}
        else if(catapultPower<0){catapultPower = 0;}
        catapultLeft.move(catapultPower);
        catapultRight.move(catapultPower);
      }
      else{
        catapultLeft.move(0);
        catapultRight.move(0);
      }
      double time = millis();
      prevError = catapultError;
      //printf(":%.2f s, pos:%.2f, targ:%.2f, err:%.2f, power:%.2f, \n",time, catapultPosition,catapultTarg,catapultError,catapultPower);
      //printf("%s", catapultMoving ? "true\n" : "false\n");
    }
    delay(10);
  }
}

void shootCatapult(){
  catapultTarg = 0;
  delay(500);
  catapultTarg = downTarg;
}
