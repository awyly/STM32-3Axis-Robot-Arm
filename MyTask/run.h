#pragma once
#include "motor.h"
void ForwardKinematics(Motor* motor, float* current_postion);

void MotorControlTask(void *argument);