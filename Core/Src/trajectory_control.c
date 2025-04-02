/**
  ******************************************************************************
  * @file           : trajectory_control.c
  * @author         : ruslan
  * @brief          : Файл содержит функции для задания и отработки траекторий вращения маховика.
  ******************************************************************************
  */


#include "trajectory_control.h"

extern TIM_HandleTypeDef htim6;

// Массив (хранилище) скоростей для отработки в задаче:
static float speed_arr[2048] = {0,};
// Длина массива задачи:
static uint32_t speed_arr_len = 0;
// Индекс для прохода по массиву скоростей в задаче:
static uint32_t index = 0;
// Переменная для отключения прерываний таймера управления траекторией:
static trajCtrl_enableDisableIT_t enableDisableIT = TIM6_IT_ENABLE;
// Переменная для блокировки задачи на время исполнения:
static trajCtrl_taskAccessStatus_t task_status = TASKSTATUS_IDLE;
// Переменная статуса синхронизации ротора и магнитного поля статора:
//trajectoryControl_rotorSynchronisationFlag_t sync_flag = ROTORSYNC_YES;
// Текущая цель управления:
trajCtrl_taskTarget_t task_target = {.parameter = TARGET_NOTARGET, .speed = 1.f, .acceleration = 0.f};
// Массивы для корректировки скорости вращения по управляющему значению
float arr_speed[ARR_LENGTH] =
{
    0.97260f, 9.86667f, 19.2500f, 28.1333f, 36.6667f, 44.7333f,
    52.3833f, 59.8833f, 66.9667f, 73.7167f, 80.3000f, 86.5833f,
    92.5167f, 98.3167f, 103.750f, 103.751f, 103.752f
};
float arr_control[ARR_LENGTH] =
{
    1.f, 10.f, 20.f, 30.f, 40.f, 50.f,
    60.f, 70.f, 80.f, 90.f, 100.f, 110.f,
    120.f, 130.f, 140.f, 150.f, 160.f
};


static uint16_t
TrajectoryControl_FindIndexForSpeed(float w);
static float
TrajCtrl_Speed2Ctrl(float w);
static uint16_t
TrajectoryControl_FindIndexForControl(float ctrl);


void
TrajCtrl_SetVelocity()
{
    RotationControl_SetControl(speed_arr[index]);
    if (index >= speed_arr_len-1)
    {
        task_status           = TASKSTATUS_IDLE;
        enableDisableIT       = TIM6_IT_DISABLE;
        task_target.parameter = TARGET_NOTARGET;
    }
    else { index += 1; }
}

trajCtrl_taskStatus_t
TrajCtrl_ReachTargetSpeed(float target_speed)
{
#if 0
    // В случае исполняемой задачи постановка новой задачи игнорируется:
    if (task_status == TASKSTATUS_BUSY) { return TASK_BUSY; }

    // Остановка таймера на всякий случай:
    HAL_TIM_Base_Stop_IT(&htim6);
#endif

    // Стартовая скорость задается как измеренная с датчиков Холла
    float start_speed = SpeedTracker_GetFrequency();

    // Установка переменной index на стартовое значение
    index = 0;
    // Шаг изменения скорости
    float dw = 0.25f;
    // Ускорение для изменения скорости
    float acceleration = TRAJCTRL_DEFAULTACCELERATION;
    // Переменная для планировщика траектории
    float inter_speed = start_speed;
    // Время исполнения задачи [сек]
    float time = 0.f;

    // Ограничение заданной целевой скорости возможностями системы:
    if (target_speed > TRAJCTRL_MAXSPEED)
    { target_speed = TRAJCTRL_MAXSPEED; }
    else if (target_speed < -TRAJCTRL_MAXSPEED)
    { target_speed = -TRAJCTRL_MAXSPEED; }
    else if ((target_speed < TRAJCTRL_MINSPEED)   &&
             (target_speed >= 0.f))
    { target_speed = TRAJCTRL_MINSPEED; }
    else if ((target_speed > -TRAJCTRL_MINSPEED)  &&
             (target_speed < 0.f))
    { target_speed = -TRAJCTRL_MINSPEED; }

    task_target.parameter = TARGET_VELOCITY;
    task_target.speed = target_speed;

    // Планировка траектории:
    for (uint8_t i = 0; i < 10; ++i)
    { speed_arr[i] = TrajCtrl_Speed2Ctrl(inter_speed); }
    speed_arr_len = 10;
    // Планировка траектории на случай start_speed < target_speed:
    if (start_speed < target_speed)
    {
        time = (target_speed - start_speed) / acceleration;
        while (inter_speed < target_speed)
        {
            if ((inter_speed >= 0.f)  &&
                (inter_speed < 1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(1.f); }
            else if (inter_speed < 0.f && inter_speed > -1.f)
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(-1.f); }
            else
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(inter_speed); }
            inter_speed += dw;
            speed_arr_len += 1;
            if ((inter_speed >= target_speed)  &&
                (inter_speed < target_speed + dw))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(target_speed); }
        }
    }
    // Планировка траектории на случай start_speed > target_speed:
    else if (start_speed > target_speed)
    {
        time = (start_speed - target_speed) / acceleration;
        while (inter_speed > target_speed)
        {
            if ((inter_speed >= 0.f)  &&
               (inter_speed < 1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(1.f); }
            else if ((inter_speed < 0.f)  &&
                    (inter_speed > -1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(-1.f); }
            else
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(inter_speed); }
            inter_speed -= dw;
            speed_arr_len += 1;
            if ((inter_speed <= target_speed)  &&
                (inter_speed > target_speed - dw))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(target_speed); }
        }
    }

    uint16_t acceleraion_ARR = (uint16_t)(10000 * time / (speed_arr_len - 1) + 1);
    // Настройка таймера, отвечающего за отработку траектории:
    TIM_MasterConfigTypeDef sMasterConfig = {
        .MasterOutputTrigger = TIM_TRGO_RESET,
        .MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE
    };
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 4800-1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = acceleraion_ARR-1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    // Начинаем отработку траектории
    // Подать питание на LTC3130 и датчики Холла
    VoltageControl_MotorHallEnable();
    // Старт контроля напряжения на выходе LTC3130
    //VoltageControl_Start();
    // Установка рабочего напряжения для ускорения
    //VoltageControl_UpdateTargetVoltage(12.f);
    // Включить вращение маховика
    RotationControl_Init();

    // Планировщик скорости переводится в режим "занят"
    task_status = TASKSTATUS_BUSY;
    enableDisableIT = TIM6_IT_ENABLE;
    HAL_TIM_Base_Start_IT(&htim6);

    return TASK_OK;
}

trajCtrl_taskStatus_t
TrajCtrl_KeepAcceleration(float target_acceleration){
#if 0
    // В случае исполняемой задачи постановка новой задачи игнорируется:
    if (task_status == TASKSTATUS_BUSY)
    { return TASK_BUSY; }

    // Остановка таймера на всякий случай:
    HAL_TIM_Base_Stop_IT(&htim6);
#endif
    // установка переменной index на стартовое значение
    index = 0;
    // стартовая скорость задается как измеренная с датчиков Холла
    float start_speed = SpeedTracker_GetFrequency();
    // шаг изменения скорости
    float dw = 0.25f;
    // ускорение для изменения скорости
    float acceleration = target_acceleration;
    // переменная для планировщика траектории
    float inter_speed = start_speed;
    // время исполнения задачи [сек]
    float time = 0.f;
    float target_speed;

    // Ограничение заданной целевой скорости возможностями системы:
    if (target_acceleration >= 0.f)
    { target_speed = TRAJCTRL_MAXSPEED; }
    else
    { target_speed = -TRAJCTRL_MAXSPEED; }

    // Ограничение заданного ускорения возможностями системы:
    if (acceleration > TRAJCTRL_MAXACCELERATION)
    { acceleration = TRAJCTRL_MAXACCELERATION; }

    else if (acceleration < -TRAJCTRL_MAXACCELERATION)
    { acceleration = -TRAJCTRL_MAXACCELERATION; }

    else if ((acceleration < TRAJCTRL_MINACCELERATION)  &&
             (acceleration >= 0.f))
    { acceleration = TRAJCTRL_MINACCELERATION; }
    else if ((acceleration > -TRAJCTRL_MINACCELERATION)  &&
             (acceleration < 0.f))
    { acceleration = -TRAJCTRL_MINACCELERATION; }

    task_target.parameter    = TARGET_ACCELERATION;
    task_target.acceleration = target_acceleration;
    task_target.speed        = target_speed;

    // Планировка траектории:
    for (uint8_t i = 0; i < 10; ++i)
    { speed_arr[i] = TrajCtrl_Speed2Ctrl(inter_speed); }
    speed_arr_len = 10;
    // Планировка траектории на случай start_speed < target_speed:
    if (start_speed < target_speed)
    {
        time = (target_speed - start_speed) / acceleration;
        speed_arr_len = 1;
        while (inter_speed < target_speed)
        {
            if ((inter_speed >= 0.f)  &&
                (inter_speed < 1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(1.f); }
            else if ((inter_speed < 0.f)  &&
                     (inter_speed > -1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(-1.f); }
            else
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(inter_speed); }
            inter_speed += dw;
            speed_arr_len += 1;
            if ((inter_speed >= target_speed)  &&
                (inter_speed < target_speed + dw))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(target_speed); }
        }
    }
    // Планировка траектории на случай start_speed > target_speed:
    else if (start_speed > target_speed)
    {
        time = (target_speed - start_speed) / acceleration;
        speed_arr_len = 1;
        while (inter_speed > target_speed)
        {
            if ((inter_speed >= 0.f)  &&
                (inter_speed < 1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(1.f); }
            else if ((inter_speed < 0.f)  &&
                     (inter_speed > -1.f))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(-1.f); }
            else
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(inter_speed); }
            inter_speed -= dw;
            speed_arr_len += 1;
            if ((inter_speed <= target_speed)  &&
                (inter_speed > target_speed - dw))
            { speed_arr[speed_arr_len-1] = TrajCtrl_Speed2Ctrl(target_speed); }
        }
    }

    uint16_t acceleraion_ARR = (uint16_t)(10000 * time / (speed_arr_len - 1) + 1);
    // Настройка таймера, отвечающего за отработку траектории:
    TIM_MasterConfigTypeDef sMasterConfig = {
        .MasterOutputTrigger = TIM_TRGO_RESET,
        .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE
    };
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 4800-1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = acceleraion_ARR-1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    { Error_Handler(); }
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    { Error_Handler(); }

    // Начинаем отработку траектории:
    // Подать питание на LTC3130 и датчики Холла
    VoltageControl_MotorHallEnable();
    // Старт контроля напряжения на выходе LTC3130
    VoltageControl_Start();
    // Включить вращение маховика
    RotationControl_Init();

    // Планировщик скорости переводится в режим "занят"
    task_status = TASKSTATUS_BUSY;

    enableDisableIT = TIM6_IT_ENABLE;
    HAL_TIM_Base_Start_IT(&htim6);

    return TASK_OK;
}

void
TrajCtrl_AbortTask()
{
    enableDisableIT = TIM6_IT_DISABLE;
    task_status = TASKSTATUS_IDLE;
}

void
TrajCtrl_EnableDisableTim6Interrupts()
{
    switch(enableDisableIT)
    {
        case TIM6_IT_ENABLE:
            HAL_TIM_Base_Start_IT(&htim6);
            break;
        case TIM6_IT_DISABLE:
            HAL_TIM_Base_Stop_IT(&htim6);
            break;
    }
}

float
TrajCtrl_Ctrl2Speed(float ctrl)
{
    float sign = 1.f;
    if (ctrl < 0.f)
    {
        ctrl = -ctrl;
        sign = -1.f;
    }
    uint16_t index = TrajectoryControl_FindIndexForControl(ctrl);
    float speed = (arr_speed[index] * arr_control[index+1] - arr_speed[index+1] * arr_control[index]) /
                  (arr_control[index+1] - arr_control[index]) +
                  (arr_speed[index+1] - arr_speed[index]) /
                  (arr_control[index+1] - arr_control[index]) * ctrl;
    return sign * speed;
}

static uint16_t
TrajectoryControl_FindIndexForSpeed(float w)
{
    uint16_t index = 0;
    if (w < arr_speed[0])
    { return index; }
    if (w > arr_speed[16])
    { return ARR_LENGTH - 2; }
    while (arr_speed[index+1] < w)
    { index += 1; }
    return index;
}

static float
TrajCtrl_Speed2Ctrl(float w)
{
    float sign = 1.f;
    if (w < 0.f)
    {
        w *= -1;
        sign = -1.f;
    }
    uint16_t index = TrajectoryControl_FindIndexForSpeed(w);
    float control = (arr_control[index] * arr_speed[index+1] - arr_control[index+1] * arr_speed[index]) /
                    (arr_speed[index+1] - arr_speed[index]) +
                    (arr_control[index+1] - arr_control[index]) /
                    (arr_speed[index+1] - arr_speed[index]) * w;
    return sign * control;
}

static uint16_t
TrajectoryControl_FindIndexForControl(float ctrl)
{
    uint16_t index = 0;
    if (ctrl < arr_control[0])
    { return index; }
    if (ctrl > arr_control[16])
    { return ARR_LENGTH - 2; }
    while (arr_control[index+1] < ctrl)
    { index += 1; }
    return index;
}
