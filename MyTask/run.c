#include "cmsis_os.h"
#include "usart.h"
#include "math.h"
#include "run.h"

extern SemaphoreHandle_t xMotorControlSemaphore;

uint8_t RxBuffer[4];
float current_position[3];
Motor motor[3] = {
    {0x01, &huart2},
    {0x02, &huart3}, 
    {0x03, &huart3}
};


void MotorControlTask(void *argument)
{
    //上电回零
    for(int i = 0; i < 3; i++)
    {
        ReachMotorPosition(&motor[i], 0x00);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  
    //初始化电机
    for(int i = 0; i < 3; i++)
    {
        QuickPositionModeInit(&motor[i], 500, 50, 0x01, 0x01);
        HAL_UARTEx_ReceiveToIdle_DMA(motor[i].huart, RxBuffer, 4);
        __HAL_DMA_DISABLE_IT(motor[i].huart->hdmarx, DMA_IT_HT);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    
    while(1)
    {
        for(int i = 0; i < 3; i++)
        {
            ReadMotorPosition(&motor[i]);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        ForwardKinematics(motor, current_position);
        for(int i = 0; i < 3; i++)
        {
            if (i == 1 || i == 2) QuickPositionModeControl(&motor[i], -motor[i].expected_angle);
            else QuickPositionModeControl(&motor[i], motor[i].expected_angle * 10/3.0f);
            vTaskDelay(pdMS_TO_TICKS(3));
        }
            StartAllMotors(&motor[0]);
            vTaskDelay(pdMS_TO_TICKS(4));
            StartAllMotors(&motor[1]);
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 || huart->Instance == USART3)
    {
      
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 || huart->Instance == USART3)
    {
        for(int i = 0; i < 3; i++)
        {
            RealTimePositionHandle(&motor[i], motor[i].RealTimePositionRxBuffer, Size);
        } 
    }  
}

void ForwardKinematics(Motor* motor, float* current_position) 
{
    float theta1 = motor[0].current_angle;
    float theta2 = motor[1].current_angle;
    float theta3 = motor[2].current_angle;

    float l1 = 9.0f; // 第一段连杆长度(cm)
    float l2 = 15.4f; // 第二段连杆长度(cm)

    float x_y = l1 * cosf(theta2) + l2 * cosf(theta2 + theta3);
    current_position[0] = x_y * cosf(theta1); // x坐标
    current_position[1] = x_y * sinf(theta1) + 1.5f; // y坐标
    current_position[2] = l1 * sinf(theta2) + l2 * sinf(theta2 + theta3) + 2.0f; // z坐标
}