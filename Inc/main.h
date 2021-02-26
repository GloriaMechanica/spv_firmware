/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define NOTE_LATCH_Pin GPIO_PIN_4
#define NOTE_LATCH_GPIO_Port GPIOA
#define NOTE_SCK_Pin GPIO_PIN_5
#define NOTE_SCK_GPIO_Port GPIOA
#define NOTE_RETURN_Pin GPIO_PIN_6
#define NOTE_RETURN_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define CPU_LOAD_Pin GPIO_PIN_1
#define CPU_LOAD_GPIO_Port GPIOB
#define ISR_LOAD_Pin GPIO_PIN_13
#define ISR_LOAD_GPIO_Port GPIOF
#define ENA_DAE_Pin GPIO_PIN_7
#define ENA_DAE_GPIO_Port GPIOE
#define X_DAE_DIR_Pin GPIO_PIN_8
#define X_DAE_DIR_GPIO_Port GPIOE
#define X_DAE_STEP_Pin GPIO_PIN_9
#define X_DAE_STEP_GPIO_Port GPIOE
#define Y_DAE_DIR_Pin GPIO_PIN_10
#define Y_DAE_DIR_GPIO_Port GPIOE
#define Y_DAE_STEP_Pin GPIO_PIN_11
#define Y_DAE_STEP_GPIO_Port GPIOE
#define Z_DAE_DIR_Pin GPIO_PIN_12
#define Z_DAE_DIR_GPIO_Port GPIOE
#define Z_DAE_STEP_Pin GPIO_PIN_13
#define Z_DAE_STEP_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define LIMIT_X_DAE_Pin GPIO_PIN_10
#define LIMIT_X_DAE_GPIO_Port GPIOD
#define LIMIT_Y_DAE_Pin GPIO_PIN_11
#define LIMIT_Y_DAE_GPIO_Port GPIOD
#define LIMIT_Z_DAE_Pin GPIO_PIN_12
#define LIMIT_Z_DAE_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define X_GDA_STEP_Pin GPIO_PIN_6
#define X_GDA_STEP_GPIO_Port GPIOC
#define Y_GDA_STEP_Pin GPIO_PIN_7
#define Y_GDA_STEP_GPIO_Port GPIOC
#define Z_GDA_STEP_Pin GPIO_PIN_8
#define Z_GDA_STEP_GPIO_Port GPIOC
#define X_GDA_DIR_Pin GPIO_PIN_9
#define X_GDA_DIR_GPIO_Port GPIOC
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
#define Y_GDA_DIR_Pin GPIO_PIN_10
#define Y_GDA_DIR_GPIO_Port GPIOC
#define Z_GDA_DIR_Pin GPIO_PIN_11
#define Z_GDA_DIR_GPIO_Port GPIOC
#define NOTE_DATA_Pin GPIO_PIN_7
#define NOTE_DATA_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SUCCESS		0
#define ERROR		-1
#define ACK			0
#define NACK		1


// To quickly change from float to double
typedef double real;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
