#include "cmsis_os.h"
#include "usart.h"
#include "math.h"
#include "run.h"

extern SemaphoreHandle_t xMotorControlSemaphore;
extern SemaphoreHandle_t semaphore;
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define MOTOR_STEP_RAD    0.05f
uint8_t RxBuffer[4];
float target_position[3] = {16.8, -6, 5.5};
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
    xSemaphoreGive(semaphore);
    while(1)
    {
        xSemaphoreTake(xMotorControlSemaphore,portMAX_DELAY);
        // 用本轮刚读回的关节角迭代求解，收敛后才写expected_angle，未收敛则保持上一次的值
        IK_CoordinateDescent(target_position, motor);
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
        // 控制指令发完后再允许Kinematics读位置，两任务共用huart3，不能同时收发
        xSemaphoreGive(semaphore);
    }
}


float joint_angle[3];
int flag = 0;
void Kinematics(void *arugument)
{
    while(1)
    {
        xSemaphoreTake(semaphore,portMAX_DELAY);
        for(int i = 0; i < 3; i++)
        {
            ReadMotorPosition(&motor[i]);
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        ForwardKinematics(motor, current_position);

        if(flag == 1)
        {
            target_position[0] = 1.768;
            target_position[1] = 1.4;
            target_position[2] = 26.3;
        }
        else if(flag == 2)
        {
            target_position[0] = 21.59;
            target_position[1] = -1.75;
            target_position[2] = 12.7;
        }
        else if(flag == 3)
        {
            target_position[0] = 15.75;
            target_position[1] = -13.77;
            target_position[2] = 6.291;
        }
        else if(flag == 4)
        {
            target_position[0] = 10;
            target_position[1] = 15;
            target_position[2] = 10;
        }
        // InverseKinematics(target_position, joint_angle);
        // for(int i = 0; i < 3; i++)
        // {
        //   motor[i].expected_angle = joint_angle[i] * RAD_TO_DEG;
        // }
        xSemaphoreTake(xMotorControlSemaphore,0);
        xSemaphoreGive(xMotorControlSemaphore);
        vTaskDelay(pdMS_TO_TICKS(3));
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
    for(int i = 0; i < 3; i++)
    { 
        // 只解析本次DMA实际写入的缓冲区，避免同一串口上其他电机的旧数据被重复解析
        if (motor[i].huart == huart && huart->pRxBuffPtr == motor[i].RealTimePositionRxBuffer)
        {
            RealTimePositionHandle(&motor[i], motor[i].RealTimePositionRxBuffer, Size);
            break;
        }
    }
}

#define ARM_L1        9.0f   // 大臂长度(cm)
#define ARM_L2        15.4f  // 小臂长度(cm)
#define ARM_OFFSET_Y  1.5f   // 小臂所在竖直平面相对云台转轴的侧向偏移(cm)，θ1=0时沿+y，随云台转动
#define ARM_OFFSET_Z  2.0f   // 大臂关节相对基座原点的z偏移(cm)

void ForwardKinematics(Motor* motor, float* current_position)
{
    float theta1 = motor[0].current_angle;
    float theta2 = motor[1].current_angle;
    float theta3 = motor[2].current_angle;

    float l1 = ARM_L1;
    float l2 = ARM_L2;
    float d = ARM_OFFSET_Y;

    float x_y = l1 * cosf(theta2) + l2 * cosf(theta2 + theta3);
    current_position[0] = x_y * cosf(theta1) - d * sinf(theta1); // x坐标
    current_position[1] = x_y * sinf(theta1) + d * cosf(theta1); // y坐标
    current_position[2] = l1 * sinf(theta2) + l2 * sinf(theta2 + theta3) + ARM_OFFSET_Z; // z坐标
}

/**
 * @brief 逆运动学：由小臂末端目标坐标求三个关节角(弧度，与ForwardKinematics约定一致)
 * @param target_position: 目标点在基座坐标系下的坐标{x, y, z}(cm)
 * @param joint_angle: 输出{theta1, theta2, theta3}
 * @retval 0: 可达并已求解  -1: 不可达，joint_angle不被修改
 * @note 多解时固定选择 手臂朝前(r>=0)、肘部朝上(theta3<=0) 的解
 */
int InverseKinematics(const float* target_position, float* joint_angle)
{
    const float l1 = ARM_L1;
    const float l2 = ARM_L2;
    const float d = ARM_OFFSET_Y;

    float px = target_position[0];
    float py = target_position[1];
    float pz = target_position[2] - ARM_OFFSET_Z;

    // 俯视下末端到云台转轴距离rho，小臂平面侧偏d，平面内水平伸出距离 r = sqrt(rho^2 - d^2)
    float rho2 = px * px + py * py;
    if (rho2 < d * d - 1e-3f)
        return -1;
    float r = (rho2 > d * d) ? sqrtf(rho2 - d * d) : 0.0f;
    float theta1 = atan2f(py, px) - atan2f(d, r);
    if (theta1 > 3.14159265f)  theta1 -= 6.28318531f;
    if (theta1 < -3.14159265f) theta1 += 6.28318531f;

    float d2 = r * r + pz * pz;

    float cos_theta3 = (d2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    // 允许少量浮点误差，使工作空间边界上的点仍可解
    if (cos_theta3 > 1.0f + 1e-4f || cos_theta3 < -1.0f - 1e-4f)
        return -1;
    if (cos_theta3 > 1.0f)  cos_theta3 = 1.0f;
    if (cos_theta3 < -1.0f) cos_theta3 = -1.0f;

    float theta3 = -acosf(cos_theta3);
    float theta2 = atan2f(pz, r) - atan2f(l2 * sinf(theta3), l1 + l2 * cosf(theta3));

    joint_angle[0] = theta1;
    joint_angle[1] = theta2;
    joint_angle[2] = theta3;
    return 0;
}

#define IK_TOLERANCE_CM   0.2f    // 末端误差小于该值即认为到达目标
#define IK_STEP_MIN_RAD   0.0005f // 步长缩小到该值仍无法改善即停止
#define IK_MAX_RESTART    12      // 卡在局部最小值时的扰动重启次数
#define ARM_THETA2_MIN_RAD 0.0f   // 大臂下限(水平)，再往下会压到下方齿轮造成堵转
#define IK_INFEASIBLE     1.0e6f  // 越过关节限位的候选角度的误差值

static float PositionError(const float* angle, const float* target)
{
    if (angle[1] < ARM_THETA2_MIN_RAD)
        return IK_INFEASIBLE;
    Motor m[3];
    m[0].current_angle = angle[0];
    m[1].current_angle = angle[1];
    m[2].current_angle = angle[2];
    float p[3];
    ForwardKinematics(m, p);
    float dx = target[0] - p[0];
    float dy = target[1] - p[1];
    float dz = target[2] - p[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

// 线性同余，只在求解内部使用，避免调用库rand带来的重入和栈开销
static uint32_t ik_rng = 0x6C8E9CF5u;
static float IK_RandSigned(void)
{
    ik_rng = ik_rng * 1664525u + 1013904223u;
    return ((int32_t)ik_rng / 2147483648.0f);
}

/**
 * @brief 坐标下降法求逆解：从当前反馈关节角出发，每次只改一个关节，
 *        沿误差减小的方向前进；三个关节都无法改善时把步长减半，
 *        缩小到极限仍未收敛则在当前最优解附近扰动后重新下降，跳出局部最小值
 * @param target_position: 目标点在基座坐标系下的坐标{x, y, z}(cm)
 * @param motor: 三个关节，读取current_angle(弧度)，收敛后写expected_angle(角度)
 * @retval 0: 误差小于IK_TOLERANCE_CM，已写expected_angle
 *        -1: 未收敛，expected_angle保持不变
 */
int IK_CoordinateDescent(const float* target_position, Motor* motor)
{
    float angle[3] = {motor[0].current_angle, motor[1].current_angle, motor[2].current_angle};
    // 回零附近的反馈可能略小于0，先夹到限位上，否则起点本身就被判为不可行
    if (angle[1] < ARM_THETA2_MIN_RAD)
        angle[1] = ARM_THETA2_MIN_RAD;
    float best[3] = {angle[0], angle[1], angle[2]};
    float error = PositionError(angle, target_position);
    float best_error = error;

    for (int restart = 0; restart <= IK_MAX_RESTART && best_error > IK_TOLERANCE_CM; restart++)
    {
        float step = MOTOR_STEP_RAD;
        if (restart > 0)
        {
            float spread = 0.6f * restart;
            angle[0] = best[0] + IK_RandSigned() * spread;
            angle[1] = best[1] + IK_RandSigned() * spread;
            angle[2] = best[2] + IK_RandSigned() * spread;
            if (angle[1] < ARM_THETA2_MIN_RAD)
                angle[1] = ARM_THETA2_MIN_RAD;
            error = PositionError(angle, target_position);
        }
        while (error > IK_TOLERANCE_CM && step >= IK_STEP_MIN_RAD)
        {
            int improved = 0;
            for (int j = 0; j < 3; j++)
            {
                angle[j] += step;
                float e = PositionError(angle, target_position);
                if (e < error) { error = e; improved = 1; continue; }

                angle[j] -= 2.0f * step;
                e = PositionError(angle, target_position);
                if (e < error) { error = e; improved = 1; continue; }

                angle[j] += step; // 两个方向都没有改善，恢复原值
            }
            if (!improved) step *= 0.5f;
        }
        if (error < best_error)
        {
            best_error = error;
            best[0] = angle[0]; best[1] = angle[1]; best[2] = angle[2];
        }
    }

    if (best_error > IK_TOLERANCE_CM)
        return -1;
    for (int i = 0; i < 3; i++)
        motor[i].expected_angle = best[i] * RAD_TO_DEG;
    return 0;
}