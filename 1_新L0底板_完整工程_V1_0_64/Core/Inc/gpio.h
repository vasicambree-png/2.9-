/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "stdio.h"
#include "string.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
/* ★ 下面两个宏是历史遗留、全工程已经没有任何人调用，而且极性与 usart.c
   里真正在用的操作点相反（LCD_ON_Power 写 RESET）。CH584M / 屏幕供电的
   权威操作点是 usart.c 的 lcd_power_on() / lcd_power_off()，极性由
   Core/Inc/usart.h 的 LCD_PWR_ON_LEVEL / LCD_PWR_OFF_LEVEL 决定
   （冻结契约 §7.1）。新增代码不要参考下面这两行。 */
#define LCD_OFF_Power HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
#define LCD_ON_Power  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);//MB_BRTS

#define Light_on    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
#define Light_off   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);//MB_BRTS


void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

