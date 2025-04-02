/**
  ******************************************************************************
  * @file           : voltage_control.h
  * @author         : ruslan
  * @brief          : Header for voltage_control.c file.
  ******************************************************************************
  */

#ifndef INC_VOLTAGE_CONTROL_H_
#define INC_VOLTAGE_CONTROL_H_


#include "main.h"


/*      List of required voltage levels for motors      */
// Required voltage level for Faulhaber 1509T012B motor (https://shop.faulhaber.com/1509t012b.html)
#define TARGETVOLTAGE_FAULHABER1509T012B    (12.f)   // Volts
// Required voltage level for another motor in lab
#define TARGETVOLTAGE_ANOTHERMOTOR          (8.f)    // Volts

// Acceptable deviation from target voltage level for fine calibration
#define TARGETVOLTAGE_DEVIATIONFINE         (0.05f)  // Volts
// Max number of iterations for fine calibration
#define TARGETVOLTAGE_MAXFINEITERATION      1
// Ratio for boosted voltage measurement via ADC
#define ADC_VOLTAGE_DIVIDER_RATIO           (1249.f / 249.f)

// Pre-calculated line interpolation coefficients
#define TARGETVOLTAGE_VOLT2DAC_COEF1        (8113.85f)
#define TARGETVOLTAGE_VOLT2DAC_COEF2        (-568.53f)

// Macros for enabling/disabling buck-boost converter LTC3130IMSE
#define LTC3130IMSE_Enable()                (HAL_GPIO_WritePin(PON_GPIO_Port, PON_Pin, GPIO_PIN_SET))
#define LTC3130IMSE_Disable()               (HAL_GPIO_WritePin(PON_GPIO_Port, PON_Pin, GPIO_PIN_RESET))
// Macro for getting LTC3130IMSE status
#define LTC3130IMSE_Status()                (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin))
// Macros for enabling/disabling motor & hall effect sensors
#define VoltageControl_MotorHallEnable()    (HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET))
#define VoltageControl_MotorHallDisable()   (HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_RESET))


typedef struct
{
    uint32_t dacValue;
    float targetVoltage;    // [Volts]
    float invertorVoltage;  // [Volts]
} voltageControl_variables_t;


/**
  * @brief Start-up function to begin voltage regulation on motor
  */
void
VoltageControl_Start();
/**
  * @brief Termination function to end voltage regulation on motor
  */
void
VoltageControl_Stop();
/**
  * @brief  Voltage correction function changing DAC value for better accuracy
  * @note   Executes by SysTick interrupts in SysTick_Handler(). It does nothing
  *         if voltage regulation is disabled.
  */
void
VoltageControl_FineCorrection();
/**
  * @brief  Returns voltage on motor supply pins
  * @retval result  Voltage on motor supply pins (Volts)
  */
void
VoltageControl_GetActualVoltage();
/**
  * @brief  Update target voltage for the invertor
  */
void
VoltageControl_UpdateTargetVoltage(float new_target_voltage);

#endif /* INC_VOLTAGE_CONTROL_H_ */

