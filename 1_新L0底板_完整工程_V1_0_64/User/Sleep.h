#ifndef  __SLEEP_H__
#define  __SLEEP_H__

#include "stm32l0xx_hal.h"
#include "stdio.h"
#include "string.h"

void EXTI_wakeup(void);
void sleep_check(void);
void SystemPower_Config(void);
void sleep_timer(void);
void sleep_timer1(void);
void ALL_IO_CLOSE(void);
void Lora_io_close(void);
void Lora_io_open(void);
void ALL_close_lora(void);
void Light_check(void);
void work_sleep(void);
void SystemClock_Config_MSI(void);
void SystemPower_Config_MSI(void);
void System_check_Init(void);

/* ★ 第 11 轮：软关机模式（同时长按 K1+K3）——实现在 Sleep.c 末尾 */
extern uint8_t  g_power_off_state;
extern uint16_t g_power_off_prompt_sec;
extern volatile uint8_t g_key_wake_flag;
void    power_off_enter(void);
uint8_t power_off_service(void);
uint8_t key_k1k3_hold_2s(void);
void    power_off_key_reset(void);
void    power_off_mask_wakeup(uint8_t mask_on);

#endif
extern volatile uint32_t g_light_chk_cnt;
