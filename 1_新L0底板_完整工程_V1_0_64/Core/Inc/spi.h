/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define LORA_NSS_H       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)  //CS
#define LORA_NSS_L       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)//CS

#define LORA_SW1_H      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET)//CTL1
#define LORA_SW1_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)//CTL1

#define LORA_SW2_H      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)//CTL2
#define LORA_SW2_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)//CTL2

#define LORA_RST_H       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET)  //RET-Rf
#define LORA_RST_L       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)

#define LORA_DIO_VAL  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4)  //DIO1

#define LORA_BUSY_VAL   HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) //BUSY 
uint8_t Spi_WRByte(uint8_t Data);
void MX_SPI1_Init_Stop(void);


extern SPI_HandleTypeDef hspi1;


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

