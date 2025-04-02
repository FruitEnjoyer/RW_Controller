/**
  ******************************************************************************
  * @file           : rotation_control.c
  * @author         : ruslan
  * @brief          : Файл содержит функции для задания скорости вращения маховика.
  ******************************************************************************
  */

#include "rotation_control.h"

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

float factor = 1.0f;

// Массив ШИМ значений для инвертора
static uint16_t pwm_arr[2 * ROTATIONCONTROL_PWMPERIOD + 1] =
{
    75, 75, 75, 76, 76, 77, 77, 77, 77, 78, 78, 78, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79,
    79, 79, 79, 79, 79, 78, 78, 78, 77, 77, 77, 77, 76, 76, 75, 75, 75,
    73, 72, 71, 69, 68, 67, 66, 64, 63, 61, 60, 59, 57, 56, 54, 53, 52,
    50, 49, 47, 46, 44, 43, 41, 40, 39, 37, 36, 34, 33, 31, 30, 28, 27,
    26, 24, 23, 21, 20, 19, 17, 16, 14, 13, 12, 11, 9, 8, 7, 5, 5, 5, 4,
    4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 5, 5,
    5, 5, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 5, 5, 5, 7, 8, 9, 11, 12, 13, 14, 16, 17, 19, 20, 21, 23, 24,
    26, 27, 28, 30, 31, 33, 34, 36, 37, 39, 40, 41, 43, 44, 46, 47, 49,
    50, 52, 53, 54, 56, 57, 59, 60, 61, 63, 64, 66, 67, 68, 69, 71, 72,
    73, 75, 75, 75, 76, 76, 77, 77, 77, 77, 78, 78, 78, 79, 79, 79, 79,
    79, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
    79, 79, 79, 79, 79, 79, 78, 78, 78, 77, 77, 77, 77, 76, 76, 75, 75,
    75, 75, 75, 76, 76, 77, 77, 77, 77, 78, 78, 78, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79,
    79, 79, 79, 79, 79, 78, 78, 78, 77, 77, 77, 77, 76, 76, 75, 75, 75,
    73, 72, 71, 69, 68, 67, 66, 64, 63, 61, 60, 59, 57, 56, 54, 53, 52,
    50, 49, 47, 46, 44, 43, 41, 40, 39, 37, 36, 34, 33, 31, 30, 28, 27,
    26, 24, 23, 21, 20, 19, 17, 16, 14, 13, 12, 11, 9, 8, 7, 5, 5, 5, 4,
    4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 5, 5,
    5, 5, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 5, 5, 5, 7, 8, 9, 11, 12, 13, 14, 16, 17, 19, 20, 21, 23, 24,
    26, 27, 28, 30, 31, 33, 34, 36, 37, 39, 40, 41, 43, 44, 46, 47, 49,
    50, 52, 53, 54, 56, 57, 59, 60, 61, 63, 64, 66, 67, 68, 69, 71, 72,
    73, 75, 75, 75, 76, 76, 77, 77, 77, 77, 78, 78, 78, 79, 79, 79, 79,
    79, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
    79, 79, 79, 79, 79, 79, 78, 78, 78, 77, 77, 77, 77, 76, 76, 75, 75, 75
};
/*
 * Индекс массива ШИМ значений для установки вектора
 * магнитного поля статора в заданное направление
 */
static int16_t pwm_counter = 0;
// Направление вращения маховика
speedTracker_spinDirection_t spin_direction = FORWARD;
// Скорость вращения маховика в единицах CNT:
static uint16_t CNTvalue = 0;

static void
RotationControl_SetFieldDirectionByArrayIndex(uint16_t index);


void
RotationControl_Init()
{
    // Включить ШИМ-контроль управляющего инвертора:
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

    // Включить таймер для вращения магнитного поля:
    HAL_TIM_Base_Start_IT(&htim5);
}

void
RotationControl_DeInit()
{
    // Выключить таймер для вращения магнитного поля:
    HAL_TIM_Base_Stop_IT(&htim5);

    // Выключить ШИМ-контроль управляющего инвертора:
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
}

void
RotationControl_SetActualFieldDirection()
{
    RotationControl_SetFieldDirectionByArrayIndex(pwm_counter);

    // Обновить счетчик:
    switch(spin_direction)
    {
        case FORWARD:
            pwm_counter += 1;
            break;
        case REVERSE:
            if(pwm_counter <= 0)
            {
                /*
                 * Необходимо зациклить счетчик,
                 * чтобы не выйти за пределы массива ниже 0
                 */
                pwm_counter += (ROTATIONCONTROL_PWMPERIOD - 1);
            }
            // Декремент в случае инверсного вращения
            else { pwm_counter -= 1; }
            break;
        /*
         * Случай DIRECTION_ERROR не обрабатывается ввиду того,
         * что программа задает конкретное направление вращения.
         */
        case DIRECTION_ERROR:
            break;
        default:
            break;
    }
    // Предотвращение выхода за пределы массива
    pwm_counter = pwm_counter % ROTATIONCONTROL_PWMPERIOD;
}

void
RotationControl_SetSpinDirection(
    speedTracker_spinDirection_t new_direction
)
{
    if((new_direction == FORWARD)   ||
       (new_direction == REVERSE))
    {
        spin_direction = new_direction;
    }
}

void
RotationControl_SetControl(float new_ctrl)
{
    if(new_ctrl >= 0)
    {
        RotationControl_SetSpinDirection(FORWARD);
    }
    else
    {
        RotationControl_SetSpinDirection(REVERSE);
    }
    CNTvalue = (uint16_t)(40000 *
        (1.f - ROTATIONCONTROL_MINSPEED / fabsf(new_ctrl)));
}

void
RotationControl_UpdateCNT()
{
    htim5.Instance->CNT = CNTvalue;
}

float
RotationControl_GetControl()
{
    float abs_speed = ROTATIONCONTROL_MINSPEED *
        (40000.f / (40000 - CNTvalue));
    if(spin_direction == REVERSE) { abs_speed *= -1; }
    return abs_speed;
}

/*
 * @brief  Установить поле статора в указанном направлении.
 * @note   Устанавливает значения ШИМ сигналов для инвертора
 *         в соответствии с таблицей скважностей pwm_arr.
 *         Фазы соседних каналов инвертора смещены друг
 *         относительно друга на 2pi/3.
 * @param  uint16_t  Индекс для массива ШИМ-скважностей.
 */
static void
RotationControl_SetFieldDirectionByArrayIndex(uint16_t index)
{
    TIM8->CCR1 = (uint32_t)(pwm_arr[index] * factor);
    TIM8->CCR2 = (uint32_t)(pwm_arr[index+(uint16_t)(ROTATIONCONTROL_PWMPERIOD*1.f/3.f)] * factor);
    TIM8->CCR3 = (uint32_t)(pwm_arr[index+(uint16_t)(ROTATIONCONTROL_PWMPERIOD*2.f/3.f)] * factor);
}

void
RotationControl_SetPWMcounter(int16_t value)
{
    while (value < 0) { value += ROTATIONCONTROL_PWMPERIOD; }
    pwm_counter = (value % ROTATIONCONTROL_PWMPERIOD);
}
