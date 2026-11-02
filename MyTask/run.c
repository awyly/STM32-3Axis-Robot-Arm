#include "cmsis_os.h"
#include "motor.h"
#include "usart.h"
#include "run.h"

extern SemaphoreHandle_t xMotorControlSemaphore;

uint8_t RxBuffer[4];
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
        HAL_UARTEx_ReceiveToIdle_DMA(motor[i].huart, RxBuffer, 8);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {

        for(int i = 0; i < 3; i++)
        {
            if (i == 1 || i == 2) QuickPositionModeControl(&motor[i], -motor[i].expected_angle);
            else QuickPositionModeControl(&motor[i], motor[i].expected_angle * 10/3.0f);
            vTaskDelay(pdMS_TO_TICKS(4));
        }
            StartAllMotors(&motor[0]);
            vTaskDelay(pdMS_TO_TICKS(4));
            StartAllMotors(&motor[1]);
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(12));
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 || huart->Instance == USART3)
    {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xMotorControlSemaphore, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 )
    {
     2;
    }
    else if (huart->Instance == USART3)
    {
        3;
    }
}