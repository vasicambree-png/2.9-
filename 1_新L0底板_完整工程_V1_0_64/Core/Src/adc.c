/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"
#include "usart.h"
#include "L_B_ADC.h"

uint16_t ADC_buff[2]={0};
uint8_t ADC_complete_flag=0;
ADC_HandleTypeDef hadc;
//DMA_HandleTypeDef hdma_adc;

/* DMA传输完成回调函数 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
// DMA_ADC_Start_flag=0;  
   ADC_DATA_pro(); 
}
 
/* DMA传输完成回调函数（HAL库自动调用） */
void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef *hdma)
{
  if (hdma->Instance == DMA1_Channel1)
  {
     /* 1. 停止ADC（可选） */
     HAL_ADC_Stop_DMA(&hadc);
     /* 2. 触发用户处理函数 */
     HAL_ADC_ConvCpltCallback(&hadc);
	}
}
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
	
  hadc.Instance = ADC1;
  if (HAL_ADC_DeInit(&hadc) != HAL_OK)
  {
    /* ADC initialization error */
//    Error_Handler();
  }
	
	hadc.Instance = ADC1;
	//过采样器
//	hadc.Init.OversamplingMode         = ENABLE;
//  hadc.Init.Oversample.Ratio         = ADC_OVERSAMPLING_RATIO_128;
//  hadc.Init.Oversample.RightBitShift = ADC_RIGHTBITSHIFT_3;
//  hadc.Init.Oversample.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode          = ADC_SCAN_ENABLE;    /* Sequencer will convert the number of channels configured below, successively from the lowest to the highest channel number */
  hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait      = DISABLE;
  hadc.Init.LowPowerAutoPowerOff  = DISABLE;
  hadc.Init.ContinuousConvMode    = ENABLE;                       /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  hadc.Init.DiscontinuousConvMode = DISABLE;                        /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;  /* Parameter discarded because trig of conversion without external event */
  hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Trig of conversion without external event: start done manually by software */
  hadc.Init.DMAContinuousRequests = DISABLE;                        /* ADC-DMA continuous requests to match with DMA configured in circular mode */
  hadc.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints. Refer to device datasheet for     */
  /*       min/typ/max values.                                                */
  hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
  
  /* ★★ 看门狗审查补丁（V1_0_2 第三轮）：这里原来调 Error_Handler()，
     而 Error_Handler() 是 `__disable_irq(); while(1){}` —— 只等看门狗复位。
     问题是 MX_ADC_Init() 会被 get_vdd_adc_value_sleep() **每 10 秒**调用一次
     （Time_1s -> L_B_ADC.c），所以一旦 ADC 初始化失败，整机就变成
     "每 2 秒复位一次"的复位循环，而且连一串口日志都看不到。
     ADC 只是用来测电池电压的，失败应当**优雅降级**（本次不测、保留上次值），
     绝不该让整机反复重启。 */
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    printf("adc init fail\r\n");
    return;
  }

	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {     
   
  }
	sConfig.Channel = ADC_CHANNEL_VREFINT;
  /* ★ 同上：改成非致命 */
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    printf("adc ch fail\r\n");
    return;
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc1)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc1->Instance==ADC1)
  {
		 __HAL_RCC_ADC1_CLK_ENABLE();
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();
  }
}


