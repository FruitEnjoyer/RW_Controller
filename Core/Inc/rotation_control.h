/**
  ******************************************************************************
  * @file           : rotation_control.h
  * @author         : ruslan
  * @brief          : Header for rotation_control.c file.
  ******************************************************************************
  */

#ifndef INC_ROTATION_CONTROL_H_
#define INC_ROTATION_CONTROL_H_


#include "main.h"
#include "speed_tracker.h"
#include "math.h"

#define ROTATIONCONTROL_PWMPERIOD  300
#define ROTATIONCONTROL_MINSPEED   (1.f)  // [оборот / сек]


/*
 * @brief  Инициализация управления магнитным полем статора.
 * @note   Включает таймер, формирующий магнитное поле (включает 3-фазный инвертор).
 *         Включает таймер, вращающий магнитное поле.
 */
void
RotationControl_Init();
/*
 * @brief  Деинициализация управления магнитным полем статора.
 * @note   Отключает таймер, вращающий магнитное поле.
 *         Отключает таймер, формирующий магнитное
 *         поле (отключает 3-фазный инвертор).
 */
void
RotationControl_DeInit();
/*
 * @brief  Установить поле статора по заданному индексу массива pwm_arr.
 * @note   Задаёт ШИМ сигналы по заданному индексу и
 *         инкрементирует индекс с учетом направления вращения.
 */
void
RotationControl_SetActualFieldDirection();
/*
 * @brief  Задать направление вращения маховика.
 * @note   Устанавливает новое значение переменной
 *         spin_direction, на основе которой формируется
 *         направление вращения магнитного поля статора.
 *         При получении значения DIRECTION_ERROR
 *         напраление вращения не меняется.
 * @param  speedTracker_spinDirection_t  Направление вращения маховика.
 */
void
RotationControl_SetSpinDirection(
    speedTracker_spinDirection_t new_direction
);
/*
 * @brief  Задать необходимое управляющее значение.
 * @note   Устанавливает значение переменной CNTvalue
 *         и направление вращения. При получении значения
 *         DIRECTION_ERROR напраление вращения не меняется.
 * @param  float  Новая управляющее значение.
 */
void
RotationControl_SetControl(float new_ctrl);
/*
 * @brief  Обновить значение регистра CNT.
 * @note   Обновляет значение регистра CNT в таймере,
 *         отвечающем за скорость вращения маховика.
 */
void
RotationControl_UpdateCNT();
/*
 * @brief   Получить управляющее значение.
 * @note    Вычисляет управляющее значение на
 *          основе содержимого регистра CNT таймера,
 *          отвечающего за скорость вращения маховика.
 * @retval  float  Управляющее значение.
 */
float
RotationControl_GetControl();
void
RotationControl_SetPWMcounter(int16_t value);

#endif /* INC_ROTATION_CONTROL_H_ */
