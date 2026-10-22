#ifndef _NEW_KEY_H_
#define _NEW_KEY_H_


#include "main.h"


#define KEY1  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)     //K1
#define KEY2  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)     //K2
#define KEY3  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)    //K3


#define  Touch_KEY1  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)   //K1
#define  Touch_KEY2  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)   //K2
#define  Touch_KEY3  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)   //K3

void key_diag_dump(void);   /* ★ V1_0_53：测试用 —— 打印按键脚电平与触摸防护计数 */
void key_touch_mute(uint16_t ticks);  /* ★ V1_0_53：上电/唤醒后的触摸静默窗口（10ms 拍）*/
#define KEY_ON  1    //按下是低电平
#define KEY_OFF  0   //


/* ==========================================================================
 * 键号（key_scan / 长短按状态机共用；★ V1_0_6 起中断里不再有边沿事件队列）
 * 注意：KEY_ON 的值与注释里的"按下是低电平"是矛盾的 —— 这是原有代码就
 * 带着的历史包袱，本次**不做改动**（改它等于改硬件极性，必须先在板上确认）：
 *   · PA8/PA9/PA10（KEY1/2/3，物理按键）在 MX_GPIO_Init() 里配的是
 *     GPIO_MODE_IT_RISING，即"按下 = 上升沿" -> KEY_ON = 1 与它一致；
 *   · PC6/PB14/PB15（Touch_KEY1/2/3，轻触）配的是 GPIO_MODE_IT_FALLING，
 *     即"按下 = 下降沿"，key_scan() 里也是按 Touch_KEYx == 0 判断的。
 * 两组按键极性不同但各自自洽；注释里的"低电平"只是针对触摸那一组的。
 * ========================================================================== */
#define KEY1_PRES 1
#define KEY2_PRES 2
#define KEY3_PRES 3

/* ★ V1_0_6：按键"边沿事件"队列已整体移除 —— 中断不再产生按键事件。
   ★ V1_0_7：按键的采样 / 去抖 / 按住时长改由 10ms 定时器**中断**里的
   key_tick_isr() 负责（主循环被 LoRa/ADC/printf 卡住时也不漏按键），
   主循环的 Key_task() 只"取粘滞电平 + 派发动作"。
   唤醒路径（main.c / Sleep.c）调用 key_state_reset() 复位整套按键状态。 */
void key_tick_isr(void);             /* ★ V1_0_7：10ms 定时器中断里调用（只采样，不派发） */
void key_state_reset(void);          /* 主循环/唤醒路径：复位长短按计时与中断侧按键状态 */
void key_wait_release(void);        /* V1_0_59：开机后等按键全部松开（吞掉"开机那一按"）*/


void Key_init(void);
void Key_task(void);
void Esc_setting(void);

typedef struct{
	uint8_t   partition; //分站号
	uint8_t   On_Off_flag; //开机或者关机
	uint8_t   Host_number;//主机号码
	uint8_t   transmit_part[3];   //发送的分站号
	uint8_t   transmit_number[3];//传递主机号
  uint16_t  brightness;//亮度
	uint8_t   ble_bangding_flag; //蓝牙绑定标志，0未绑定，1绑定
	int8_t    transmitted_power; //发射功率。-3~22；
}Flash_Set;
#endif
