/*
 * PIDHorizontal.h
 *
 *  Created on: Aug 18, 2025
 *      Author: DELL
 */


#ifndef PIDHORIZONTAL_H_
#define PIDHORIZONTAL_H_

#include "main.h"
#include <cmath>

typedef struct {
	float Pgain, Igain, Dgain;
	float outputValue;
	float maxOutputValue;
	float inputValue;
	float ImemValue;
	float setPoint;
	float previousError;
	float momentError;
	float levelAdjust;
	int32_t setPointBase;
	float prevOutput;
} rpyObject;
class PID_Horizontal {
public:
	rpyObject roll;
	rpyObject pitch;
	rpyObject yaw;
	int32_t throttleSetpoint;
	PID_Horizontal();
	void setKgainConstantRoll(float thePgainRoll, float theIgainRoll, float theDgainRoll, float theMaxOutputValue);
	void setKgainConstantPitch(float thePgainPitch, float theIgainPitch, float theDgainPitch, float theMaxOutputValue);
	void setKgainConstantYaw(float thePgainYaw, float theIgainYaw, float theDgainYaw, float theMaxOutputValue);
	void setLevelAdjust(float theLevelAdjustRoll, float theLevelAdjustPitch);
	void calculatePID(float dt);
//	void calculatePID();
//	void calculateOuterLoop(float measuredRollAngle, float measuredPitchAngle);
	void calculateOuterLoop(float measuredRollAngle, float measuredPitchAngle, float dt);


	virtual ~PID_Horizontal();
};

#endif /* PIDHORIZONTAL_H_ */


