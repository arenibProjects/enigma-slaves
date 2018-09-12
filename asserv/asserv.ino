#include <Arduino.h>

#include "src/odometry.hpp"
#include "src/coders.hpp"

IntervalTimer controlTimer;
Coders coders(33,34,35,36);
Odometry odometry(0,0,0,265.0,16.0,20000);

void setup(){
  Serial.begin(250000);
  delay(1000);
  controlTimer.begin(mainLoop, 4166);
  controlTimer.priority(129);
}

void mainLoop(){
    int cl=coders.left();
    int cr=coders.right();
    Serial.println(cl+' '+cr);
    odometry.move(cl,cr);
    //controller.update(odometry.getX(),odometry.getY(),odometry.getA());
}
