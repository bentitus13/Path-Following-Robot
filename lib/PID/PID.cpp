/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "PID.h"
#include "Arduino.h"

//Class constructor
PID::PID(){

}

//Function to set PID gain values
void PID::setpid(double P, double I, double D, double sampleFreq, int max){
  kp=P;
  ki=I;
  kd=D;
  sampleFrequency = sampleFreq;
  maxVal = max;
}

//Write this function to calculate a control signal from the set velocity
//and the current velocity
double PID::calc(double setVal, double curVal){
    if (setPoint != setVal) {
        setPoint = setVal;
        errorSum = 0;
    }

    error = setPoint - curVal;
    sumError += error;

    double P_Val = Kp * error;
    double I_Val = Ki * sumError / sampleFrequency;
    double D_Val = Kd * (error - prevError) * sampleFrequency;

    double motorVal = P_Val + I_Val + D_Val;

    if (motorVal > 255) motorVal = 255;
    if (motorVal < -255) motorVal = -255;

    prevError = error;

    return motorVal;
}
