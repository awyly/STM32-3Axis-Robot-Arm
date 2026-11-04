#pragma once
#include "motor.h"
void ForwardKinematics(Motor* motor, float* current_postion);
int InverseKinematics(const float* target_position, float* joint_angle);
int IK_CoordinateDescent(const float* target_position, Motor* motor);

void MotorControlTask(void *argument);
void Kinematics(void *arugument);