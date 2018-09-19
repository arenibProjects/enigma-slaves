#ifndef DIFFERENTIAL_CONTROLLER_H
#define DIFFERENTIAL_CONTROLLER_H
#include "stuPid.hpp"
#include "cuPid.hpp"
class DifferentialController{
  public:
    DifferentialController(double dp,double di,double dd,double ap,double ai,double ad);
    void update(double x,double y, double z);
    void setTarget(double x,double y, double z);
    int getLeft();
    int getRight();
    void setFactors(double dp,double di,double dd,double ap,double ai,double ad);
    double getFactor(int i);
    void debug();
    void reset();
  private:
    PID *distancePID;
    CuPID *anglePID;
    double din,dtar,dout;
    double ain,atar,aout;
    double tx,ty,ta;

};
#endif
