/* simple timestamp useful Pid */
#ifndef CUPID_H
#define CUPID_H

class CuPID
{
  public:
    CuPID(double * iinput, double * isetpoint, double * ioutput, double ikp,double iki,double ikd);
    void compute();
    void set(double kp,double ki,double kd);
    double getP();
    double getI();
    double getD();
    void reset();
    bool isFacingFront();
  private:
    double *input,*setpoint,*output;
    double kp,ki,kd,integ,prevE;
    long prevT;
};
#endif
