#pragma once
#include "motor.h"
void ForwardKinematics(Motor* motor, float* current_postion);
int InverseKinematics(const float* target_position, float* joint_angle);

void MotorControlTask(void *argument);
void Kinematics(void *arugument);