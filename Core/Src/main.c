/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>

#include "voltage_control.h"

#include "rotation_control.h"

#include "speed_tracker.h"

#include "trajectory_control.h"

#include "motor_supervisor.h"

#include "communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_MOTOR
#define USE_CAN
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern trajCtrl_taskTarget_t task_target;
extern communication_vars_t can_vars;

uint8_t flag_speed = 0;
float target_speed = 0.f;
uint8_t flag_ctrl = 0;
float target_ctrl = 0.f;
uint8_t flag_volt = 0;
uint32_t target_dac = 0;
uint8_t flag_acceleration = 0;
float target_acceleration = 0.f;

extern can_float_t can_spd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#ifdef USE_MOTOR
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

  VoltageControl_MotorHallEnable();  // Подать питание на LTC3130 и датчики Холла
  VoltageControl_Start();        // Старт контроля напряжения на выходе LTC3130
  RotationControl_Init();        // Включить вращение маховика
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
#endif
#ifdef USE_CAN
  // Включение питания трансиверов
  // FIXME: для включения на 484 RESET, на 450 SET
  HAL_GPIO_WritePin(CAN_EN_GPIO_Port, CAN_EN_Pin, GPIO_PIN_RESET);
  can_vars.TxHeader.StdId = 0x0378;
  can_vars.TxHeader.ExtId = 0;
  can_vars.TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  can_vars.TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
  can_vars.TxHeader.DLC = 8;
  can_vars.TxHeader.TransmitGlobalTime = 0;
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef USE_MOTOR
    if(flag_speed != 0)
    {
       TrajCtrl_ReachTargetSpeed(target_speed);
       flag_speed = 0;
    }
    if(flag_acceleration != 0)
    {
       TrajCtrl_KeepAcceleration(target_acceleration);
       flag_acceleration = 0;
    }
    if(flag_volt != 0)
    {
       HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, target_dac);
       flag_volt = 0;
    }
#if 1
    // Восстановление после обнаружения срыва
    if (MotorSupervisor_GetSyncFlag() == ROTORSYNCFLAG_NO)
    {
        //TrajCtrl_AbortTask();
        //TrajCtrl_KeepAcceleration((float)acc);
        TrajCtrl_ReachTargetSpeed(can_vars.speed.f);
    }
#endif
    /*
     * Восстановление вращения при залипании маховика в неподвижном состоянии.
     *
     * Описание проблемы: наблюдается ситуация, при которой маховик неподвижен,
     *     датчики Холла не срабатывают (т.к. маховик не пересекает их), и
     *     скорость, измеряемая модулем speed_tracker, не обновляется до
     *     правильного околонулевого значения.
     *     Система обнаруживает срыв, но пытается восстановить скорость с
     *     неправильного значения скорости, что приводит к тому, что маховик не
     *     зацепляется быстровращающимся магнитным полем, и остается неподвижным.
     *     Такая ситуация может возникнуть при кратковременном заклинивании маховика.
     *
     * Решение: при чересчур долгом пересечении датчиков Холла принудительно сбросить
     *     значение измеряемой скорости в нулевое значение. Тем самым, обнаружив срыв,
     *     система попытается восстановить скорость с нулевой скорости, что раскрутит
     *     маховик.
     */
    if (TIM2->CNT > 48000000)
    {
        SpeedTracker_SetFrequencyToZero();
        switch (SpeedTracker_GetSector())
        {
        case SECTOR_1:
            if (SpeedTracker_GetDirection() == FORWARD)
            { RotationControl_SetPWMcounter(285); }
            else
            { RotationControl_SetPWMcounter(35); }
            break;
        case SECTOR_2:
            if (SpeedTracker_GetDirection() == FORWARD)
            { RotationControl_SetPWMcounter(85); }
            else
            { RotationControl_SetPWMcounter(135); }
            break;
        case SECTOR_3:
            if (SpeedTracker_GetDirection() == FORWARD)
            { RotationControl_SetPWMcounter(185); }
            else
            { RotationControl_SetPWMcounter(235); }
            break;
        case SECTOR_ERROR:
            break;
        default:
        	break;
        }
        TIM2->CNT = 0;
    }
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
