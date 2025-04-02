/**
  ******************************************************************************
  * @file           : speed_tracker.h
  * @author         : ruslan
  * @brief          : Header for speed_tracker.c file.
  ******************************************************************************
  */

#ifndef INC_SPEED_TRACKER_H_
#define INC_SPEED_TRACKER_H_


#include "main.h"
#include "math.h"


#define HALL_LOW                    (GPIO_PIN_RESET)
#define HALL_HIGH                   (GPIO_PIN_SET)

#define SPEEDTRACKER_MAXSPEED       (100.f)

// Количество датчиков Холла в моторе
#define HALL_SENSOR_NUMBER          3
// Количество пар полюсов одной обмотки в моторе
#define ROTOR_MAGNETIC_PAIR_NUMBER  2


/*
 * @brief  Направление вращения ротора со стороны маховика:
 *         FORWARD - по часовой стрелке
 *         REVERSE - против часовой стрелки
 *         DIRECTION_ERROR - направление не определено
 * @note   DIRECTION_ERROR необходимо для случаев, когда контроллер
 *         пропустил прерывание от датчика Холла.
 */
typedef enum
{
    FORWARD,
    REVERSE,
    DIRECTION_ERROR = -1
} speedTracker_spinDirection_t;

/*
 * @brief    Секторы для определения положения ротора.
 * @note     Секторы расположены в ряд друг за другом,
 *           SECTOR_1 и SECTOR_3 закольцованы и расположены рядом.
 *           Количество секторов - удвоенное количество датчиков Холла.
 *           SECTION_ERROR необходимо для некорректных данных с датчиков Холла.
 * @warning  При прохождении одного оборота ротора программа может проходить
 *           по секторам несколько раз - это зависит от количества пар магнитных
 *           полюсов ротора. В случае мотора Faulhaber 1509T012B пар полюсов 2,
 *           следовательно, секторы пройдутся 2 раза за 1 оборот ротора.
 */
typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_ERROR = -1
} speedTracker_angleSector_t;

/*
 * @brief Структура переменных модуля speed_tracker.
 */
typedef struct
{
    // Частота вращения маховика [оборот / сек]
    float frequency;

    // Информация о состоянии вращения маховика
    speedTracker_angleSector_t currentSector;
    speedTracker_spinDirection_t measuredDirection;

    // Счетчик поворотов маховика
    int32_t turnCounter;

    // Вспомогательные переменные
    uint32_t count1, count2, count3;
    float t1, t2, t3;
} speedTracker_variables_t;

/*
 * @brief Возвращает поле .frequency
 *        из структуры переменных speed_tracker.
 */
float
SpeedTracker_GetFrequency();
/*
 * @brief Возвращает поле .measuredDirection
 *        из структуры переменных speed_tracker.
 */
speedTracker_spinDirection_t
SpeedTracker_GetDirection();
/*
 * @brief Возвращает поле .currentSector
 *        из структуры переменных speed_tracker.
 */
speedTracker_angleSector_t
SpeedTracker_GetSector();
/*
 * @brief Устанавливает ноль в поле .frequency
 * @note  Необходимо для устранения проблемы
 *        залипания маховика в бездвижном положении.
 */
void
SpeedTracker_SetFrequencyToZero();


#endif /* INC_SPEED_TRACKER_H_ */
