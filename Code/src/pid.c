#include "pid.h"
#include <stdio.h>
#include <stdint.h>

uint16_t Kp = 100;
uint16_t Ki = 0;
uint16_t Kd = 0;
double Ts = 1;
double upperBoundU_s = 255.0;
double lowerBoundU_s = -255.0;

double xStorage[3] = {0, 0, 0};
double yStorage[3] = {0,0,0};

/*
  * Desc.: Sets the proportional gain of the PID controller.
  * @param: (uint16_t) newKp: proportional gain
  * @return: none
  */
void setKp(uint16_t newKp){
    Kp = newKp;
}

/*
  * Desc.: Sets the integral gain of the PID controller
  * @param: (uint16_t) newKi: integral gain
  * @return: none
  */
void setKi(uint16_t newKi){
    Ki = newKi;
}

/*
  * Desc.: Sets the derivative gain of the PID controller.
  * @param: (uint16_t) newKp: proportional gain [percentage]
  * @return: none
  */
void setKd(uint16_t newKd){
    Kd = newKd;
}

/*
  * Desc.: Sets the upper bound of the control variable.
  * @param: (double) upperBound: maximal value of the control variable [percentage]
  * @return: none
  */
void setUpperBoundSystemInput(double upperBound){
    upperBoundU_s = upperBound;
}

/*
  * Desc.: Sets the lower bound of the control variable.
  * @param: (double) upperBound: minimal value of the control variable [percentage]
  * @return: none
  */
void setLowerBoundSystemInput(double lowerBound){
    lowerBoundU_s = lowerBound;
}

/*
  * Desc.: Sets the sampling time for the controller.
  * @param: (double) sampleTime: sampling time [seconds]
  * @return: none
  */
void setSamplingTime(double sampleTime){
    Ts = sampleTime;
}

/*
  * Desc.: A getter for all of the current controller parameters.
  * @param: (uint16_t*) currKp
  * @param: (uint16_t*) currKi
  * @param: (uint16_t*) currKd
  * @param: (uint16_t*) currUpperBound
  * @param: (uint16_t*) currLowerBound
  * @param: (uint16_t*) currTs
  * @return: none
  */
void getCurrentParameters(uint16_t* currKp, uint16_t* currKi, uint16_t* currKd, double* currUpperBound, double* currLowerBound, double* currTs){
    *currKp = Kp;
    *currKi = Ki;
    *currKd = Kd;
    *currUpperBound = upperBoundU_s;
    *currLowerBound = lowerBoundU_s;
    *currTs = Ts;
}

/*
  * Desc.: This is the main function of the controller. 
            This function is called during the setting of the sampling interval. 
            It calculates the error between the input and the feedback value, performs the necessary value shifting, and computes the new control value. 
            Essentially, it represents a continuous-time PID controller transformed into the Z-domain using the bilinear transformation.
  * @param: (double) current: current input
  * @param: (double) setPoint: desired setpoint
  * @return: none
  */
double procPID(double current, double setPoint){
    double u_s = 0.0;
    double error = setPoint-current;
    double xTempStorage[3] = {0,0,0};
    double yTempStorage[3] = {0,0,0};
    xTempStorage[0] = xStorage[0];
    xTempStorage[1] = xStorage[1];
    xTempStorage[2] = xStorage[2];
    yTempStorage[0] = yStorage[0];
    yTempStorage[1] = yStorage[1];
    yTempStorage[2] = yStorage[2];
    xStorage[0] = error;
    xStorage[1] = xTempStorage[0];
    xStorage[2] = xTempStorage[1];
    u_s = ((xStorage[0]*(2*(Kp/100)*Ts + (Ki/100)*(Ts*Ts) + 4*(Kd/100))) + (xStorage[1] * (2*(Ki/100)*(Ts*Ts) - 8*(Kd/100))) + (xStorage[2]*((Ki/100)*(Ts*Ts) + 4*(Kd/100) - 2*(Kp/100)*Ts)) + (2*Ts*yStorage[1]))/(2*Ts);
    yStorage[0] = u_s;
    yStorage[1] = yTempStorage[0]; 
    yStorage[2] = yTempStorage[1];
    return u_s;
}