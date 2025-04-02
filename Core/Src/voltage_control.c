/**
  ******************************************************************************
  * @file           : voltage_control.c
  * @author         : ruslan
  * @brief          : The file contains functions for buck-boost converter calibration.
  ******************************************************************************
  */

#include "voltage_control.h"


extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;

static voltageControl_variables_t voltageControl = {
    .dacValue = 0,
    .invertorVoltage = 0.f,
    .targetVoltage = TARGETVOLTAGE_FAULHABER1509T012B
};


static uint32_t
GetADCValue(uint32_t channel);
static float
TargetVoltage2DAC(float targetVoltage);

void
VoltageControl_Start()
{
    // Enable buck-boost converter
    LTC3130IMSE_Enable();
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    float coarse_value = TargetVoltage2DAC(voltageControl.targetVoltage);

    // Preventing DAC value under(over)flow
    if(coarse_value < 0) { voltageControl.dacValue = 0; }
    else if(coarse_value > 4095) { voltageControl.dacValue = 4095; }
    else { voltageControl.dacValue = (uint32_t)coarse_value; }

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltageControl.dacValue);
}

void
VoltageControl_Stop()
{
    // Disable buck-boost converter to prevent uncontrolled voltage supply
    LTC3130IMSE_Disable();

    // Disable DAC
    voltageControl.dacValue = 0;
    HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
}

void
VoltageControl_FineCorrection()
{
    if(LTC3130IMSE_Status() == GPIO_PIN_SET)
    {
        for(uint16_t i = 0; i < TARGETVOLTAGE_MAXFINEITERATION; ++i)
        {
            VoltageControl_GetActualVoltage();
            if((voltageControl.invertorVoltage - voltageControl.targetVoltage) > TARGETVOLTAGE_DEVIATIONFINE)
            {
                if(voltageControl.dacValue < 4095) { voltageControl.dacValue += 1; }
                else { voltageControl.dacValue = 4095; }
            }
            else if((voltageControl.invertorVoltage - voltageControl.targetVoltage) < -TARGETVOLTAGE_DEVIATIONFINE)
            {
                if(voltageControl.dacValue > 0) { voltageControl.dacValue -= 1; }
                else { voltageControl.dacValue = 0; }
            }
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltageControl.dacValue);
        }
    }
}

void
VoltageControl_UpdateTargetVoltage(float new_target_voltage)
{
    voltageControl.targetVoltage = new_target_voltage;

    // Linear dependence between DAC value and voltage
    float coarse_value = TargetVoltage2DAC(voltageControl.targetVoltage);

    // Preventing DAC value under(over)flow
    if(coarse_value < 0) { voltageControl.dacValue = 0; }
    else if(coarse_value > 4095) { voltageControl.dacValue = 4095; }
    else { voltageControl.dacValue = (uint32_t)coarse_value; }

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltageControl.dacValue);
}

void
VoltageControl_GetActualVoltage()
{
    uint32_t adc = GetADCValue(ADC_CHANNEL_13);
    // ADC value for VREFINT
    uint32_t vrefADC = GetADCValue(ADC_CHANNEL_VREFINT);
    // Reference voltage on VDDA pin (Volts)
    float vrefint = (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefADC, LL_ADC_RESOLUTION_12B) / 1000.f;

    // Boost-converter OUT voltage (12-bits, 450 board)
    voltageControl.invertorVoltage = (float)(adc / 4096.f * vrefint * ADC_VOLTAGE_DIVIDER_RATIO);
}

/**
  * @brief   Obtains a 12-bit ADC value by
  *          converting on the selected channel
  * @param   channel  ADC channel
  * @retval  result   ADC value obtained by ADC conversion
  */
static uint32_t
GetADCValue(uint32_t channel)
{
    uint32_t result = 0;
    ADC_ChannelConfTypeDef sConfig = {
        .Channel = channel,
        .Rank = 1,
        .SamplingTime = ADC_SAMPLETIME_84CYCLES,
        .Offset = 0  // Optional
    };

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    result = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return result;
}

/*
 * @brief Find DAC value via target voltage
 * @note  DAC value and invertor voltage have
 *        linear dependence and can be calculated
 *        before compiling.
 *        Coefficient are selected from the resistive
 *        connection between the FB pin and the DAC.
 */
static float
TargetVoltage2DAC(float targetVoltage)
{
    return TARGETVOLTAGE_VOLT2DAC_COEF1 + TARGETVOLTAGE_VOLT2DAC_COEF2 * targetVoltage;
}
