/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */

#ifndef PID_H_
#define PID_H_

class PID {

public:
PID();
int maxVal;
double sampleFrequency;
double Kp;
double Ki;
double Kd;
double setPoint;
double prevError = 0;
double sumError =0;
//set PID constants
void setpid(double P, double I, double D, double sampleFreq, int max);
//calculate the output control signal
double calc(double setVal, double curVal);

private:

};



#endif
