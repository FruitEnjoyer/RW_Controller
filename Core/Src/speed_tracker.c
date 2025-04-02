/**
  ******************************************************************************
  * @file           : speed_tracker.c
  * @author         : ruslan
  * @brief          : Файл содержит функции для вычисления скорости ротора и ее углового положения.
  ******************************************************************************
  */

#include "speed_tracker.h"


static void
SpeedTracker_CalcFrequency();
static float
SpeedTracker_GetPCLK1Freq();


static speedTracker_variables_t speedTracker = {
    .frequency = 0.f,
    .currentSector = SECTOR_1,
    .measuredDirection = FORWARD,
    .turnCounter = 0,
    .count1 = 0, .count2 = 0, .count3 = 0,
    .t1 = 0.f, .t2 = 0.f, .t3 = 0.f
};


void
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM2)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            TIM2->CNT = 0;
            /*
             * Запись значений регистров счетчика
             * необходима для более корректных данных
             */
            speedTracker.count1 = TIM2->CCR2;
            speedTracker.count2 = TIM2->CCR3;
            speedTracker.count3 = TIM2->CCR1;

            if(speedTracker.currentSector == SECTOR_3)
            {
                speedTracker.measuredDirection = FORWARD;
            }
            else { speedTracker.measuredDirection = REVERSE; }
            speedTracker.currentSector = SECTOR_1;

            SpeedTracker_CalcFrequency();
        }
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if(speedTracker.currentSector == SECTOR_1)
            {
                speedTracker.measuredDirection = FORWARD;
            }
            else { speedTracker.measuredDirection = REVERSE; }
            speedTracker.currentSector = SECTOR_2;
        }
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            if(speedTracker.currentSector == SECTOR_2)
            {
                speedTracker.measuredDirection = FORWARD;
            }
            else { speedTracker.measuredDirection = REVERSE; }
            speedTracker.currentSector = SECTOR_3;
        }
    }
}


static void
SpeedTracker_CalcFrequency()
{
    float pclk1Freq = SpeedTracker_GetPCLK1Freq();
    if(speedTracker.measuredDirection == FORWARD)
    {
        speedTracker.t1 = -(speedTracker.count1 / pclk1Freq);
        speedTracker.t2 = -(speedTracker.count2 / pclk1Freq);
        speedTracker.t3 = -(speedTracker.count3 / pclk1Freq);
    }
    else if(speedTracker.measuredDirection == REVERSE)
    {
        speedTracker.t1 = -(speedTracker.count2 / pclk1Freq);
        speedTracker.t2 = -(speedTracker.count1 / pclk1Freq);
        speedTracker.t3 = -(speedTracker.count3 / pclk1Freq);
    }

    float numerator = -speedTracker.t1 / 6 - speedTracker.t2 / 3 - speedTracker.t3 / 2;
    float denumerator = powf(speedTracker.t1, 2) + powf(speedTracker.t2, 2) + powf(speedTracker.t3, 2);
    int8_t sign = 1;
    if(speedTracker.measuredDirection == REVERSE) { sign = -sign; }

    /*
     * Необходимая проверка на случай если регистры CCRx
     * содержат нули (например, при старте программы).
     * Если не делать проверку, программа упадет в HardFault_Handler().
     * Причина падения: при CCRx == 0 поле frequency становится равным inf.
     * При операции inf + float в TrajectoryControl происходит ошибка.
     *
     * 2e-5 [сек] это оценка denumerator при
     * самом быстром допустимом вращении маховика
     */
    if(denumerator <= 2e-5)
    {
        speedTracker.frequency = sign * SPEEDTRACKER_MAXSPEED;
    }
    else { speedTracker.frequency = sign * numerator / denumerator; }
}

static float
SpeedTracker_GetPCLK1Freq()
{
  /* Get PCLK1 frequency */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  /* Get PCLK1 prescaler */
  if((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
  {
    /* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
    return (float)(pclk1);
  }
  else
  {
    /* PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1 */
    return (float)(2 * pclk1);
  }
}

float
SpeedTracker_GetFrequency()
{
    return speedTracker.frequency;
}

speedTracker_spinDirection_t
SpeedTracker_GetDirection()
{
    return speedTracker.measuredDirection;
}

speedTracker_angleSector_t
SpeedTracker_GetSector()
{
    return speedTracker.currentSector;
}

void
SpeedTracker_SetFrequencyToZero(){
    speedTracker.frequency = 0.f;
}
