/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRESET_I_Pin GPIO_PIN_2
#define NRESET_I_GPIO_Port GPIOF
#define P_GAUGE_ADC1_CH0_I_Pin GPIO_PIN_0
#define P_GAUGE_ADC1_CH0_I_GPIO_Port GPIOA
#define P_GAUGE_SW_O_Pin GPIO_PIN_1
#define P_GAUGE_SW_O_GPIO_Port GPIOA
#define DN_UART2_TX_Pin GPIO_PIN_2
#define DN_UART2_TX_GPIO_Port GPIOA
#define DN_UART2_RX_Pin GPIO_PIN_3
#define DN_UART2_RX_GPIO_Port GPIOA
#define POS_RIGHT_Pin GPIO_PIN_4
#define POS_RIGHT_GPIO_Port GPIOA
#define CURRENT_ADC1_CH5_I_Pin GPIO_PIN_5
#define CURRENT_ADC1_CH5_I_GPIO_Port GPIOA
#define VOLTAGE_ADC1_CH6_I_Pin GPIO_PIN_6
#define VOLTAGE_ADC1_CH6_I_GPIO_Port GPIOA
#define MT_TEMP_ADC1_CH7_I_Pin GPIO_PIN_7
#define MT_TEMP_ADC1_CH7_I_GPIO_Port GPIOA
#define BD_TEMP_ADC1_CH8_I_Pin GPIO_PIN_0
#define BD_TEMP_ADC1_CH8_I_GPIO_Port GPIOB
#define MT_SPEED_14CH1_PWM_Pin GPIO_PIN_1
#define MT_SPEED_14CH1_PWM_GPIO_Port GPIOB
#define MT_CW_CCW_O_Pin GPIO_PIN_8
#define MT_CW_CCW_O_GPIO_Port GPIOA
#define MT_SENSOR_T3CH1_CNT_Pin GPIO_PIN_6
#define MT_SENSOR_T3CH1_CNT_GPIO_Port GPIOC
#define MT_STOP_START_O_Pin GPIO_PIN_11
#define MT_STOP_START_O_GPIO_Port GPIOA
#define TEMP_PW_SW_O_Pin GPIO_PIN_12
#define TEMP_PW_SW_O_GPIO_Port GPIOA
#define LED_G_MT_RUN_O_Pin GPIO_PIN_15
#define LED_G_MT_RUN_O_GPIO_Port GPIOA
#define LED_R_MCU_ERR_O_Pin GPIO_PIN_3
#define LED_R_MCU_ERR_O_GPIO_Port GPIOB
#define LED_G_MCU_RUN_O_Pin GPIO_PIN_4
#define LED_G_MCU_RUN_O_GPIO_Port GPIOB
#define POS_LEFT_I_Pin GPIO_PIN_5
#define POS_LEFT_I_GPIO_Port GPIOB
#define UP_UART1_TX_Pin GPIO_PIN_6
#define UP_UART1_TX_GPIO_Port GPIOB
#define UP_UART1_RX_Pin GPIO_PIN_7
#define UP_UART1_RX_GPIO_Port GPIOB
#define POS_CENTOR_I_Pin GPIO_PIN_8
#define POS_CENTOR_I_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
