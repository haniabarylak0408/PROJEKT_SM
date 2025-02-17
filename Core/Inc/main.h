/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMP280_SCK_Pin GPIO_PIN_2
#define BMP280_SCK_GPIO_Port GPIOE
#define BMP280_CS2_Pin GPIO_PIN_3
#define BMP280_CS2_GPIO_Port GPIOE
#define BMP280_CS3_Pin GPIO_PIN_4
#define BMP280_CS3_GPIO_Port GPIOE
#define BMP280_MISO_Pin GPIO_PIN_5
#define BMP280_MISO_GPIO_Port GPIOE
#define BMP280_MOSI_Pin GPIO_PIN_6
#define BMP280_MOSI_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LCD_RW_Pin GPIO_PIN_0
#define LCD_RW_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_3
#define LCD_RS_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_3
#define LCD_EN_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define Heater_Pin GPIO_PIN_13
#define Heater_GPIO_Port GPIOE
#define Fan_Pin GPIO_PIN_14
#define Fan_GPIO_Port GPIOE
#define LD1EX_Pin GPIO_PIN_15
#define LD1EX_GPIO_Port GPIOE
#define LD2EX_Pin GPIO_PIN_10
#define LD2EX_GPIO_Port GPIOB
#define LD3EX_Pin GPIO_PIN_11
#define LD3EX_GPIO_Port GPIOB
#define BMP280_CS1_Pin GPIO_PIN_12
#define BMP280_CS1_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define ENC_CLK_Pin GPIO_PIN_14
#define ENC_CLK_GPIO_Port GPIOD
#define ENC_DT_Pin GPIO_PIN_15
#define ENC_DT_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOD
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOD
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOD
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
