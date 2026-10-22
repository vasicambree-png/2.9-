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
#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define     MAXPART         64
#define     MAXADDR         120

#define     NONEPART         0  //无分区为0
#define     NONEADDR        121  //无设备地址121
#define     STATION         122  //分站122号

/* 出厂默认地址(EEPROM 未初始化时用)。两份底板代码改 DEFAULT_HOST_NUM 区分。 */
#define     DEFAULT_PARTITION   2
#define     DEFAULT_HOST_NUM    3
/* 自动测试模式: 1=上电自动组网+缩短LoRa/息屏周期(仅测试用); 验收改回 0 */
#define     TEST_MODE          0
/* TEST: force Setup.On_Off_flag = 0 to verify power-off gating (0=off) */
#define     TEST_ONOFF_FORCE   0
/* TEST: force device onto the power-off wait page to verify main-loop isolation (0=off) */
#define     TEST_PWRWAIT       0
/* TEST: run USART1 parser self-test with synthetic corrupt streams (0=off) */
#define     TEST_PARSER        0
/* TEST: print GPIO pull/idle registers every second (0=off) */
#define     TEST_GPIO          0
#define     TEST_KEY_SRC       0   /* 每秒打印 6 个按键脚电平 + 触摸防护计数器（查"没按键页面却乱跳"）*/
#define     TOUCH_INPUT_DISABLE 0  /* 1 = 整机不使用触摸输入（只保留机械 K1/K2/K3）。给触摸控制器损坏的整机用 */
#define     TEST_KEEP_AWAKE    0   /* 1 = 测试用：永不息屏，便于长时间观察按键假信号 */

/* TEST: enter the soft power-off flow right after boot (0=off) */
#define     TEST_PWRSHUT       0
/* MONITOR: shorten only the screen-off read period for long-term observation.
   Keys stay ENABLED and no auto-net is started (0=off, uses the 600s production period). */
#define     MONITOR_MODE       0
#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

