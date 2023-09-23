#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

extern void setKp(uint16_t newKp);
extern void setKi(uint16_t newKi);
extern void setKd(uint16_t newKd);
extern void setSamplingTime(double sampleTime);
extern void setUpperBoundSystemInput(double upperBound);
extern void setLowerBoundSystemInput(double lowerBound);
extern void getCurrentParameters(uint16_t* currKp, uint16_t* currKi, uint16_t* currKd, double* currUpperBound, double* currLowerBound, double* currTs);

extern double procPID(double current, double setPoint);

#endif // !PID_H


