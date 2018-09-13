#include "src/coders.hpp"
#include "src/odometry.hpp"
#include "src/DifferentialController.hpp"
#include "src/Motor.hpp"
#include <Arduino.h>

IntervalTimer controlTimer;
Coders coders(33,34,35,36);
Odometry odometry(1000,1000,0,265.0,16.0,20000);
DifferentialController controller(10,0,0,100,0,0);
Motor leftMotor(2,3,4);
Motor rightMotor(5,6,7);
bool forward = true;

void setup(){
  Serial.begin(250000);
  delay(1000);

  controlTimer.begin(mainLoop, 4166);
  controlTimer.priority(129);

  controller.update(odometry.getX(),odometry.getY(),odometry.getA());
  controller.setTarget(odometry.getX(),odometry.getY(),odometry.getA());
}

void loop(){
  delay(2000);
  /*
  if(odometry.getX()>100){
    forward = false;
  }
  if(odometry.getX()<-100){
    forward = true;
  }*/
}

void mainLoop(){
    odometry.move(coders.left(),coders.right());
    controller.update(odometry.getX(),odometry.getY(),odometry.getA());
    #if 0
    leftMotor.setSpeed(forward?40:-40);
    rightMotor.setSpeed(forward?40:-40);
    #else
    leftMotor.setSpeed(controller.getLeft());
    rightMotor.setSpeed(controller.getRight());
    #endif
}
