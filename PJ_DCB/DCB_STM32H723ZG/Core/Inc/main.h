/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define LCD_ER_SPI4_SCK_Pin GPIO_PIN_2
#define LCD_ER_SPI4_SCK_GPIO_Port GPIOE
#define LCD_ER_DC_Pin GPIO_PIN_3
#define LCD_ER_DC_GPIO_Port GPIOE
#define LCD_ER_SPI4_NSS_Pin GPIO_PIN_4
#define LCD_ER_SPI4_NSS_GPIO_Port GPIOE
#define LCD_ER_SPI4_MISO_Pin GPIO_PIN_5
#define LCD_ER_SPI4_MISO_GPIO_Port GPIOE
#define LCD_ER_SPI4_MOSI_Pin GPIO_PIN_6
#define LCD_ER_SPI4_MOSI_GPIO_Port GPIOE
#define HYPER_FLASH_RESET_Pin GPIO_PIN_5
#define HYPER_FLASH_RESET_GPIO_Port GPIOF
#define LIGHT_CTRL_DAC1_Pin GPIO_PIN_4
#define LIGHT_CTRL_DAC1_GPIO_Port GPIOA
#define LIGHT_ENABLE_Pin GPIO_PIN_5
#define LIGHT_ENABLE_GPIO_Port GPIOC
#define LCD_MT_SPI2_NSS_Pin GPIO_PIN_12
#define LCD_MT_SPI2_NSS_GPIO_Port GPIOB
#define LCD_MT_SPI2_SCK_Pin GPIO_PIN_13
#define LCD_MT_SPI2_SCK_GPIO_Port GPIOB
#define LCD_MT_SPI2_MISO_Pin GPIO_PIN_14
#define LCD_MT_SPI2_MISO_GPIO_Port GPIOB
#define LCD_MT_SPI2_MISOB15_Pin GPIO_PIN_15
#define LCD_MT_SPI2_MISOB15_GPIO_Port GPIOB
#define LCD_MT_DC_Pin GPIO_PIN_8
#define LCD_MT_DC_GPIO_Port GPIOD
#define LCD_MT_RESET_Pin GPIO_PIN_9
#define LCD_MT_RESET_GPIO_Port GPIOD
#define LED_G_UART_TX_Pin GPIO_PIN_6
#define LED_G_UART_TX_GPIO_Port GPIOG
#define LED_G_UART_RX_Pin GPIO_PIN_7
#define LED_G_UART_RX_GPIO_Port GPIOG
#define LED_G_SPI_RX_Pin GPIO_PIN_8
#define LED_G_SPI_RX_GPIO_Port GPIOG
#define LCD_MT_TIM3_CH1_Pin GPIO_PIN_6
#define LCD_MT_TIM3_CH1_GPIO_Port GPIOC
#define LCD_EL_TIM3_CH2_Pin GPIO_PIN_7
#define LCD_EL_TIM3_CH2_GPIO_Port GPIOC
#define LCD_ER_TIM3_CH3_Pin GPIO_PIN_8
#define LCD_ER_TIM3_CH3_GPIO_Port GPIOC
#define LED_R_ERROR_Pin GPIO_PIN_8
#define LED_R_ERROR_GPIO_Port GPIOA
#define LED_G_RUN_Pin GPIO_PIN_9
#define LED_G_RUN_GPIO_Port GPIOA
#define LCD_EL_SPI3_NSS_Pin GPIO_PIN_15
#define LCD_EL_SPI3_NSS_GPIO_Port GPIOA
#define LCD_EL_SPI3_SCK_Pin GPIO_PIN_10
#define LCD_EL_SPI3_SCK_GPIO_Port GPIOC
#define LCD_EL_SPI3_MISO_Pin GPIO_PIN_11
#define LCD_EL_SPI3_MISO_GPIO_Port GPIOC
#define LCD_EL_SPI3_MOSI_Pin GPIO_PIN_12
#define LCD_EL_SPI3_MOSI_GPIO_Port GPIOC
#define LCD_EL_DC_Pin GPIO_PIN_0
#define LCD_EL_DC_GPIO_Port GPIOD
#define LCD_EL_RESET_Pin GPIO_PIN_1
#define LCD_EL_RESET_GPIO_Port GPIOD
#define LCD_ER_RESET_Pin GPIO_PIN_1
#define LCD_ER_RESET_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
