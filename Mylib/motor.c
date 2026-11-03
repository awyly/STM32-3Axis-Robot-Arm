#include "motor.h"
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)

/**
 * @brief 使能电机
 * @param motor: 指向Motor结构体的指针，包含电机ID和UART句柄
 * @param enable: 0表示不使能，1表示使能
 * @retval NULL
 */
void EnableMotorControl(Motor* motor, MotorEnableState enable_state)
{
    static uint8_t SendBuffer[6];
    SendBuffer[0] = motor->motor_id;
    SendBuffer[1] = 0xF3;
    SendBuffer[2] = 0xAB;   
    SendBuffer[3] = enable_state;   
    SendBuffer[4] = 0x00;
    SendBuffer[5] = 0x6B;
    HAL_UART_Transmit(motor->huart, SendBuffer, sizeof(SendBuffer), 30);
}

/**
 * @brief 设置零位(记忆零位)
 * @param motor: 指向Motor结构体的指针，包含电机ID和UART句柄
 * @retval NULL
 */
void SetMotorZeroPosition(Motor* motor)
{
    static uint8_t SendBuffer[5];
    SendBuffer[0] = motor->motor_id;
    SendBuffer[1] = 0x93;
    SendBuffer[2] = 0x88;   
    SendBuffer[3] = 0x01;   
    SendBuffer[4] = 0x6B;
    HAL_UART_Transmit(motor->huart, SendBuffer, sizeof(SendBuffer), 30);
}

/**
 * @brief 回到零位
 * @param motor: 指向Motor结构体的指针，包含电机ID和UART句柄
 * @param mode: 回零模式
 * @retval NULL
 */
void ReachMotorPosition(Motor* motor, uint8_t mode)
{
    static uint8_t SendBuffer[5];
    SendBuffer[0] = motor->motor_id;
    SendBuffer[1] = 0x9A;
    SendBuffer[2] = mode;   
    SendBuffer[3] = 0x00;   
    SendBuffer[4] = 0x6B;
    HAL_UART_Transmit(motor->huart, SendBuffer,sizeof(SendBuffer), 30);
}

/**
 * @brief 快速位置模式初始化
 * @param motor: 指向Motor结构体的指针，包含电机ID和UART句柄
 * @param speed: 速度参数(0-3000RPM)
 * @param acceleration: 加速度参数(0-255)
 * @param mode: 模式参数 01是相对零点进行运动
 * @param flag: 同步标志参数
 * @retval NULL
 */
void QuickPositionModeInit(Motor* motor, uint16_t speed, uint8_t acceleration, uint8_t mode, uint8_t flag)
{
    static uint8_t SendBuffer[8];
    SendBuffer[0] = motor->motor_id;
    SendBuffer[1] = 0xF1;
    SendBuffer[2] = speed >> 8;
    SendBuffer[3] = speed & 0xFF;
    SendBuffer[4] = acceleration;
    SendBuffer[5] = mode;
    SendBuffer[6] = flag;
    SendBuffer[7] = 0x6B;
    HAL_UART_Transmit(motor->huart, SendBuffer, sizeof(SendBuffer), 100);
}

/**
 * @brief 快速位置模式控制
 * @param motor: 指向Motor结构体的指针，包含电机ID和UART句柄
 * @param angle: 目标角度参数
 * @retval NULL
 */
int QuickPositionModeControl(Motor* motor, float angle)
{
    static uint8_t SendBuffer[7];
    static int32_t pulse;
    pulse = (int32_t)(angle / (1.8/16));
    SendBuffer[0] = motor->motor_id;
    SendBuffer[1] = 0xFC;
    SendBuffer[2] = pulse >> 24;
    SendBuffer[3] = pulse >> 16;
    SendBuffer[4] = pulse >> 8;
    SendBuffer[5] = pulse & 0xFF;
    SendBuffer[6] = 0x6B;
    return HAL_UART_Transmit_DMA(motor->huart, SendBuffer, sizeof(SendBuffer));
}
void StartAllMotors(Motor* motor)
{
    static uint8_t SendBuffer[4] = {0x00, 0xFF, 0x66, 0x6B};
    HAL_UART_Transmit_DMA(motor->huart, SendBuffer, sizeof(SendBuffer));
}

void ReadMotorPosition(Motor* motor)
{
    static uint8_t SendBuffer[3];
    SendBuffer[0] = motor->motor_id;
    SendBuffer[1] = 0x36;
    SendBuffer[2] = 0x6B;
    // 上一次应答丢失时RX会一直处于BUSY，先终止，否则本次接收无法启动
    if (motor->huart->RxState != HAL_UART_STATE_READY)
        HAL_UART_AbortReceive(motor->huart);
    HAL_UARTEx_ReceiveToIdle_DMA(motor->huart, motor->RealTimePositionRxBuffer, 8);
    __HAL_DMA_DISABLE_IT(motor->huart->hdmarx, DMA_IT_HT);
    HAL_UART_Transmit_DMA(motor->huart, SendBuffer, sizeof(SendBuffer));
}

void RealTimePositionHandle(Motor* motor, uint8_t* RxBuffer, uint16_t Size)
{
    // 应答帧: 地址 0x36 符号 位置[4](大端) 0x6B
    if (Size == 8 && RxBuffer[0] == motor->motor_id && RxBuffer[1] == 0x36 &&
        RxBuffer[7] == 0x6B && RxBuffer[2] <= 0x01)
    {
        int8_t direction = (RxBuffer[2] == 0) ? 1 : -1;
        uint32_t magnitude = ((uint32_t)RxBuffer[3] << 24) | ((uint32_t)RxBuffer[4] << 16) |
                             ((uint32_t)RxBuffer[5] << 8) | (uint32_t)RxBuffer[6];
        // 各关节电机轴实际转动不会超过4圈，超出说明是错误数据，丢弃以免污染当前角度
        if (magnitude > 4u * 65536u)
            return;
        int32_t ReadPosition = (int32_t)magnitude;
        // 云台：电机轴与大齿轮转向相反，乘3/10换算成大齿轮角度(俯视逆时针为正)
        if(motor->motor_id == 0x01)
            motor->current_angle = -direction*ReadPosition* (360/65536.0f) * 3/10.0f*DEG_TO_RAD;
        else if(motor->motor_id == 0x02)
            motor->current_angle = ReadPosition * (360/65536.0f) * direction*DEG_TO_RAD;
        else if(motor->motor_id == 0x03)
            motor->current_angle = ReadPosition * (360/65536.0f) * -direction*DEG_TO_RAD;
    }
}