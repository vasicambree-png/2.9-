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
#ifndef __LPTIM_H__
#define __LPTIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
void Lptim_Init(void);

uint32_t TickDiff_100ms(uint32_t Tick);

void Lptim_time_setting(uint16_t time_num);

/* V1_0_57：LPTIM1 重新武装（Sleep.c 的 100ms 活性检测调用；RTC 已删除，LPTIM1 是 STOP 下唯一时基） */
void Lptim_ReArm_Tick100ms(void);

/* V1_0_57：直接读 LPTIM1 的**硬件计数器/重装值**（不经 HAL 缓存、不依赖 Rt_tick）。
   用途：① 判断 100ms 是否真的在走（CNT 两次采样是否变化）；
         ② 早醒判定用硬件证据（STOP 前后 CNT 是否跨过）；
         ③ 现场用 J-Link 直接看 CNT 就能确认定时器死没死。
   手册依据：STM32L0 参考手册 LPTIM 章节（连续模式下 CNT 由 LSI/LSE 驱动自由递增，
   STOP 期间计数继续，CNT/ARR 为 16 位可读寄存器；读 CNT 不影响计数）。 */
uint16_t Lptim_CntRead(void);
uint16_t Lptim_ArrRead(void);


#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

