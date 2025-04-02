/*
 * motor_supervisor.h
 *
 *  Created on: Nov 26, 2024
 *      Author: ruslan
 */

#ifndef INC_MOTOR_SUPERVISOR_H_
#define INC_MOTOR_SUPERVISOR_H_

#include "trajectory_control.h"
#include "rotation_control.h"

// TODO: Добавить константы
#define MOTORSYPERVISOR_SYNCPRECISION    4.f    // [оборот / сек]

/*
 * @brief Флаг для определения наличия срыва мотора.
 * @note  Флаг принимает значение ROTORSYNCFLAG_YES в случае,
 *        если различие в частоте вращения ротора и частоты
 *        вращения магнитного поля статора невелико (меньше,
 *        чем значение MOTORSYPERVISOR_SYNCPRECISION)
 *        Флаг принимает значение ROTORSYNCFLAG_NO во всех
 *        остальных случаях.
 *        Проверка наличия срыва осуществляется в SysTick_Handler()
 *        с помощью функции MotorSupervisor_UpdateRotorSyncFlag().
 */
typedef enum
{
    ROTORSYNCFLAG_YES = 0,
    ROTORSYNCFLAG_NO
} motorSupervisor_rotorSyncFlag_t;

/*
 * @brief Структура переменных для модуля motor_supervisor.
 * @note
 */
typedef struct
{
    // TODO: Доопределить структуру переменных
    motorSupervisor_rotorSyncFlag_t sync_flag;
} motorSupervisor_variables_t;

typedef enum
{
    // Напряжение питания инвертора упало ниже установленного предела
    ERRCODE_VOLTAGEUNDERFLOW = 1,
    // Напряжение питания инвертора превысило установленный предел
    ERRCODE_VOLTAGEOVERFLOW,
    // Ток потребления мотора мало или равен нулю
    ERRCODE_NOCURRENT,
    // Произошел срыв мотора
    ERRCODE_LOSTMOTORSYNC
} motorSupervisor_errcode_t;

typedef struct
{
    // TODO: Доопределить структуру отчета
    motorSupervisor_errcode_t errcode;
} motorSupervisor_issueReport_t;


/*
 * @brief Проверить наличие срыва ротора и установить
 *        соответствующий флаг в переменную с типом
 *        motorSupervisor_rotorSyncFlag_t.
 */
void
MotorSupervisor_UpdateRotorSyncFlag();

/*
 * @brief Возвращает поле sync_flag структуры переменных
 */
motorSupervisor_rotorSyncFlag_t
MotorSupervisor_GetSyncFlag();

#endif /* INC_MOTOR_SUPERVISOR_H_ */
