/**
  ******************************************************************************
  * @file           : trajectory_control.h
  * @author         : ruslan
  * @brief          : Header for trajectory_control.c file.
  ******************************************************************************
  */

#ifndef INC_TRAJECTORY_CONTROL_H_
#define INC_TRAJECTORY_CONTROL_H_

#include "main.h"
#include "speed_tracker.h"
#include "voltage_control.h"
#include "rotation_control.h"

#define TRAJCTRL_DEFAULTACCELERATION  (5.0f)   // [оборот / сек^2]
#define TRAJCTRL_MAXSPEED             (80.f)   // [оборот / сек]
#define TRAJCTRL_MINSPEED             (1.f)    // [оборот / сек]
#define TRAJCTRL_MAXACCELERATION      (8.f)   // [оборот / сек^2]
#define TRAJCTRL_MINACCELERATION      (0.1f)   // [оборот / сек^2]
#define ARR_LENGTH                    17
#define TRAJCTRL_SYNCPRECISION        (4.5f)    // [оборот / сек]


/*
 *  @brief  Флаг для отключения/включения прерываний таймера для контроля траектории.
 */
typedef enum{
    TIM6_IT_DISABLE,
    TIM6_IT_ENABLE
} trajCtrl_enableDisableIT_t;

/*
 *  @brief  Результат задания задачи.
 *  @note   TASK_OK - задача задана успешно (задача уже исполнена)
 *          TASK_ERROR - задача не задана, возникла ошибка
 *          TASK_BUSY - задача не задана, исполняется другая задача
 */
typedef enum{
    TASK_OK,
    TASK_ERROR,
    TASK_BUSY
} trajCtrl_taskStatus_t;

/*
 *  @brief  Статус работы планировщика задач.
 *  @note   TASKSTATUS_IDLE - задачи не исполняются, можно создать задачу
 *          TASKSTATUS_BUSY - задача исполняется, создавать новые задачи нельзя
 */
typedef enum{
    TASKSTATUS_IDLE,
    TASKSTATUS_BUSY
} trajCtrl_taskAccessStatus_t;

typedef enum{
    TARGET_NOTARGET = 0,
    TARGET_VELOCITY,
    TARGET_ACCELERATION
} trajCtrl_targetParameter_t;

typedef struct{
    trajCtrl_targetParameter_t parameter;
    float speed;
    float acceleration;
} trajCtrl_taskTarget_t;


/*
 * @brief  Установить скорость из массива speed_array.
 * @note   Устанавливает заданную скорость,
 *         смещает индекс на следующее значение скорости,
 *         по окончании задачи останавливает выполнение.
 */
void
TrajCtrl_SetVelocity();
/*
 * @brief   Достичь заданной скорости маховика.
 * @note    Планирует траекторию разгона маховика и запускает процесс ее отработки.
 * @param   float                           Целевая скорость [оборот / сек]
 * @retval  trajectoryControl_taskStatus_t  Статус исполнения задачи.
 */
trajCtrl_taskStatus_t
TrajCtrl_ReachTargetSpeed(float target_speed);
/*
 * @brief   Удерживать заданное ускорение маховика.
 * @note    Планирует траекторию разгона маховика с заданным
 *          ускорением и запускает процесс ее отработки.
 * @param   float                           Задаваемое ускорение [оборот / сек^2]
 * @retval  trajectoryControl_taskStatus_t  Статус исполнения задачи.
 */
trajCtrl_taskStatus_t
TrajCtrl_KeepAcceleration(float target_acceleration);
/*
 * @brief  Остановить выполнение текущей задачи.
 */
void
TrajCtrl_AbortTask();
/*
 * @brief  Остановить/продолжить работу таймера 6 в режиме прерываний.
 * @note   Останавливает задачу по флагу enableDisableIT.
 */
void
TrajCtrl_EnableDisableTim6Interrupts();
/*
 * TODO: Добавить описание функции
 * @brief
 * @note
 */
float
TrajCtrl_Ctrl2Speed(float ctrl);

#endif /* INC_TRAJECTORY_CONTROL_H_ */
