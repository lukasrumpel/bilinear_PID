/*
bilinear_PID
Copyright (C) 2023  Lukas Rumpel

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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


