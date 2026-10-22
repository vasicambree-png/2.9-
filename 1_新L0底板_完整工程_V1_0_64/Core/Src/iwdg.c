/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
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
/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"
#include "usart.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
//喂狗超时时间 Time_OUT = Prescaler/LSI*Reload (ms)
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;  //关闭喂狗窗口限制
  hiwdg.Init.Reload = 1200;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}
void  re_iwdg(void)
{
	/* V1_0_60：ALL_init 现在很靠前就调 re_iwdg()，那时 hiwdg.Instance 可能还没初始化；
	   HAL_IWDG_Refresh() 会解引用 Instance，NULL 就是 HardFault。加一道保护。 */
	if (hiwdg.Instance == NULL) { return; }
	if(HAL_IWDG_Refresh(&hiwdg)!= HAL_OK)
	{
		printf("iiwdg_error\r\n");

	}
//	else{
////		printf("re_iwdg\r\n");
//	}
}
