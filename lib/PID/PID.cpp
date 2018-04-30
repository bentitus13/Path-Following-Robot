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
    Kp=P;
    Ki=I;
    Kd=D;
    sampleFrequency = sampleFreq;
    maxVal = max;
    for (int i = 0; i < 32; i++) {
        pastErrors[i] = 0.0;
    }
}

//Write this function to calculate a control signal from the set velocity
//and the current velocity
double PID::calc(double setVal, double curVal){
    if (setPoint != setVal) {
        setPoint = setVal;
        sumError = 0;
    }

    // Serial.print("pastErrorIndex: ");
    // Serial.println(pastErrorIndex);

    pastErrorIndex = 31 & (pastErrorIndex + 1);
    double error = setPoint - curVal;
    sumError += error;
    sumError -= pastErrors[pastErrorIndex];

    double P_Val = Kp * error;
    double I_Val = Ki * sumError / sampleFrequency;
    double D_Val = Kd * (error - prevError) * sampleFrequency;

    double motorVal = P_Val + I_Val + D_Val;

    if (motorVal > 255) motorVal = 255;
    if (motorVal < -255) motorVal = -255;

    prevError = error;
    pastErrors[pastErrorIndex] = error;

    // if (motorVal > 100) motorVal = 100;
    // Serial.println(motorVal);

    return motorVal;
}
