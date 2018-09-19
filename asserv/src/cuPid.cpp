#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "cuPid.hpp"

const double pi = 3.141592f;

double circstrain(double c){
  while(c<-pi)c+=2*pi;
  while(c>pi)c-=2*pi;
  return c;
}

double getError(double current, double objective){
  while(objective<=-pi)objective+=2*pi;
  while(objective>pi)objective-=2*pi;

  while(current<=objective-pi)current+=2*pi;
  while(current>objective+pi)current-=2*pi;
  return objective-current;
}

CuPID::CuPID(double * iinput, double * isetpoint, double * ioutput,double ikp,double iki,double ikd){
  while(*iinput<pi)*iinput+=2*pi;
  while(*iinput>3*pi)*iinput-=2*pi;
  kp=ikp;
  ki=iki;
  kd=ikd;
  input=iinput;
  setpoint=isetpoint;
  output=ioutput;
  prevT=-1;
  prevE=0;
  integ=0;
}

double deriv;
void CuPID::compute(){
  double E = getError(*input, *setpoint);
  long T=micros();
  double dt=T-prevT;
  if(prevT==-1) dt=0;
  integ+=E*dt;
  if(dt==0)deriv=0;
  else deriv=(E-prevE)/dt;
  prevT=T;
  prevE=E;
  *output=kp*E+ki*integ+kd*deriv;
  // Zone morte
  if(abs(*output) >= MIN_ZERO_POWER && abs(*output)<MAX_ZERO_POWER){
    if(*output>0){
      *output=MAX_ZERO_POWER;
    }else{
      *output=-MAX_ZERO_POWER;
    }
  }
  if(debug){
    Serial.println(String(kp*E) + " " + String(*input) + " " + String(*setpoint) + " " + String(kd*deriv));
    debug = false;
  }
  // stabilisateur expérimental
  /*if(deriv<5){
    *output*=max(0.7,deriv/5);
  }*/
}

void CuPID::printDebug(){
  debug = true;
}

bool CuPID::isFacingFront(){
  return abs(*setpoint-*input)<pi;
}
void CuPID::set(double ikp,double iki,double ikd){
  kp=ikp;
  ki=iki;
  kd=ikd;
}
double CuPID::getP(){
  return kp;
}
double CuPID::getI(){
  return ki;
}
double CuPID::getD(){
  return kd;
}
void CuPID::reset(){
  prevT=-1;
  prevE=0;
  integ=0;
}
