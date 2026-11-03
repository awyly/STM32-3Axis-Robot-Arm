#pragma once

#include "stdint.h"
#include "stm32f4xx_hal.h"

//上电后要给电机一定的启动时间，使用延时函数等待两秒再去执行任务

typedef struct{
    uint8_t motor_id;
    UART_HandleTypeDef *huart;
    float expected_angle;
    float current_angle;
    uint8_t RealTimePositionRxBuffer[8];
}Motor;


void SetMotorZeroPosition(Motor* motor);
void ReachMotorPosition(Motor* motor, uint8_t mode);
void QuickPositionModeInit(Motor* motor, uint16_t speed, uint8_t acceleration, uint8_t mode, uint8_t flag);
int QuickPositionModeControl(Motor* motor, float angle);
void StartAllMotors(Motor* motor);
void ReadMotorPosition(Motor* motor);
void RealTimePositionHandle(Motor* motor, uint8_t* RxBuffer, uint16_t Size);