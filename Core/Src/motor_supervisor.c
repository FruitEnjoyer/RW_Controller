/*
 * motor_supervisor.c
 *
 *  Created on: Nov 26, 2024
 *      Author: ruslan
 */


#include "motor_supervisor.h"

static motorSupervisor_variables_t motorSupervisor = {
    .sync_flag = ROTORSYNCFLAG_YES
};


/*
 * @brief Проверка синхронности поля статора с ротором.
 * @note  Устанавливает значение в поле sync_flag:
 *          ROTORSYNCFLAG_YES при наличии синхронизации
 *          ROTORSYNCFLAG_NO при отсутствии синхронизации
 */
void
MotorSupervisor_UpdateRotorSyncFlag()
{
    if(fabsf(SpeedTracker_GetFrequency() - TrajCtrl_Ctrl2Speed(RotationControl_GetControl())) >= MOTORSYPERVISOR_SYNCPRECISION)
    {
        motorSupervisor.sync_flag = ROTORSYNCFLAG_NO;
    }
    else { motorSupervisor.sync_flag = ROTORSYNCFLAG_YES; }
}

motorSupervisor_rotorSyncFlag_t
MotorSupervisor_GetSyncFlag()
{
    return motorSupervisor.sync_flag;
}
