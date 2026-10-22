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
#include "stm32l0xx_hal.h"


/* ==========================================================================
 * V1_0_57：**RTC 已从本工程整体删除**
 * --------------------------------------------------------------------------
 * 历史上 RTC 只承担一个职责：给 STOP 模式当"兜底闹钟"（sleep_stop_ms()）。
 * 现在 STOP 的定时唤醒与全工程的时基 Rt_tick 完全由 LPTIM1 的 100ms ARRM
 * 中断承担（Core/Src/lptim.c），并且加了 LPTIM1 活性检测 + 自动重新武装
 * （见 Sleep.c 的 g_lptim_rearm_cnt / Lptim_ReArm_Tick100ms()）。
 * 因此本文件的 RTC_* 宏（RTC_CLOCK_SOURCE_LSI / RTC_ASYNCH_PREDIV /
 * RTC_SYNCH_PREDIV）连同 Core/Src/rtc.c、Core/Inc/rtc.h 一起废弃：
 * 文件已删除（备份 rtc.c.before_live / rtc.h.before_live），
 * 工程文件 MDK-ARM\\MX_02.uvprojx 里的 <Compile> 项也已移除。
 * stm32l0xx_it.c 里保留空的 RTC_IRQHandler 只为满足启动文件向量表。
 * ========================================================================== */
/* 仅作历史记录保留（无任何代码引用；见上面的说明） */
/* #define RTC_CLOCK_SOURCE_LSI */
/* #define RTC_CLOCK_SOURCE_LSE */

/* Comment to disable displaying the time on the debugger */
#define DISPLAY_ON_DUBUGGER
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
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

/* USER CODE BEGIN Private defines */
void ALL_init(void);
void Time_1s(void);
/* USER CODE END Private defines */
void CheckResetReason(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
