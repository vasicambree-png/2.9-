/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "stm32l0xx_it.h"
#include "soft_timer.h"
#include "loraPro.h"
#include "New_Key.h"     /* KEY_ON / KEYx_PRES 宏（本文件只做"按键唤醒"，不产生按键事件） */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern ADC_HandleTypeDef hadc;

extern L_TxStatus    Lora_Tx_state;

extern uint8_t sleep_mode;
extern volatile uint8_t g_key_wake_flag;   /* ★ 第 11 轮：软关机按键唤醒标志（Sleep.c） */

extern volatile uint8_t  Lora_Rx_state;


volatile uint32_t Rt_tick=0;


void NMI_Handler(void)
{
   while (1)
  {
  }
}

/* HardFault：把故障现场留在 r_sp 里方便调试器查看（原来是局部变量，
   优化后可能被丢掉；改成全局 volatile，Keil 里 watch 变量名即可）。
   注意：这里仍然不喂狗 —— 让 IWDG 复位是设计意图。 */
volatile uint32_t g_hardfault_sp = 0U;
volatile uint32_t g_hardfault_count = 0U;

void HardFault_Handler(void)
{
  g_hardfault_count++;
  g_hardfault_sp = __get_MSP();

  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* ======================================================================
	 * ★★★ 按键中断的唯一职责：**把机器唤醒**（V1_0_6 按键改造后冻结）
	 * ----------------------------------------------------------------------
	 * 1) 这 6 个键（实体键 PA8/PA9/PA10 + 触摸键 PB14/PB15/PC6）把机器从
	 *    软关机态 / 息屏 STOP 里叫醒：g_key_wake_flag 给 Sleep.c 的
	 *    power_off_service() 消费，sleep_mode=3 让状态机退出睡眠。
	 * 2) **中断里不再产生任何按键事件**：原来这里还有 flag_key=N; 与
	 *    key_evt_push(KEYx_PRES);（边沿事件队列），已按需求整体删除。
	 *    原因：实体/轻触键的按下与松开都带毫秒级抖动，边沿触发会把**一次
	 *    物理按下**拆成多个中断，再被当成"多次按下"消费 -> 一次按键触发
	 *    多次动作。现在按键的识别与派发只在 Key_task（10ms 软定时器）里做，
	 *    并带"防重复触发窗口"（见 New_Key.c）。
	 * 3) 下面仍然清 EXTI 挂起标志：不清的话边沿会一直挂着，反复进中断。
	 * ====================================================================== */
	if (GPIO_Pin == GPIO_PIN_8  || GPIO_Pin == GPIO_PIN_9  || GPIO_Pin == GPIO_PIN_10 ||
	    GPIO_Pin == GPIO_PIN_14 || GPIO_Pin == GPIO_PIN_15 || GPIO_Pin == GPIO_PIN_6)
	{
		g_key_wake_flag = 1u;
	}
	 switch(GPIO_Pin)
	 {
		 case GPIO_PIN_4: //DIO1
				 switch(Lora_Rx_state)
				 {
					 case 1:
						 Lora_Rx_state=2;
						 break;
					 case 3:
						 Lora_Rx_state=4;
						 break;
				 }
			   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
			 break;
		 case GPIO_PIN_5://DIO2
			 {
					/* ★★★ V1_0_11：DIO2 只在 CAD 期间才有意义，不能无条件改写状态
					 * ------------------------------------------------------------------
					 * DIO2 只在 SX126X_CADinit() 里被映射成 IRQ_CAD_ACTIVITY_DETECTED；
					 * 在 TX（SX126X_TxPacket / SX126X_Awake）和 RX（SX126X_WOR_Exit）
					 * 的配置里 DIO2 = IRQ_RADIO_NONE。也就是说：
					 *   在"发射中(1)"或"接收中(3)"收到 DIO2 边沿，只能是**上一轮 CAD
					 *   残留的挂起边沿 / 干扰毛刺**，不是本次的有效事件。
					 * 原实现无条件 Lora_Rx_state=6，后果是**直接打断正在进行的收发**：
					 *   · 打断 TX：while(Lora_Rx_state==1) 立刻退出 -> 包还在发就当成发完；
					 *   · 打断 RX：while(Lora_Rx_state==3 && 超时) 立刻退出 ->
					 *     走到"被误唤醒"分支 -> Lora_Rx_state=0 + 射频进睡眠
					 *     -> **整包丢失**，现场就是"切换模式时卡住收不到数据"。
					 * 现在只有真正处于 CAD 流程（5=CAD进行中，7=CAD已结束但结果
					 * 尚未被消费）时才认这个边沿。 */
					if(Lora_Rx_state==5 || Lora_Rx_state==7)
					{
						Lora_Rx_state=6;      /* CAD 检测到信道有活动 -> 转长接收 */
					}
					 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
			 }
			 break;
		 case GPIO_PIN_0: //DIO3
			 	switch(Lora_Rx_state)
				{
					case 3:
						 Lora_Rx_state=8;
						break;
					case 5:
						 Lora_Rx_state=7;
						break;
				}
		    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
				break;
	   case GPIO_PIN_8:
							  if(sleep_mode!=0)
			 {
				 sleep_mode=3;
			 }
			 __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_8);
				break;
		  case GPIO_PIN_6:
			 if(sleep_mode!=0)
			 {
				 sleep_mode=3;
			 }
		 	 __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_6);
			 break;
			case GPIO_PIN_14:
			 if(sleep_mode!=0)
			 {
				 sleep_mode=3;
			 }
		 	 __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_14);
			 break;
		  case GPIO_PIN_15:
			 if(sleep_mode!=0)
			 {
				 sleep_mode=3;
			 }
		 	 __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_15);
			 break;
		 case GPIO_PIN_9:
			  if(sleep_mode!=0)
			  {
				  sleep_mode=3;
			  }
	 		  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_9);
			 break;
		 case GPIO_PIN_10:
			  if(sleep_mode!=0)
			  {
				  sleep_mode=3;
			  }
		 	  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_10);
			 break;
	 }
}
void EXTI0_1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void EXTI4_15_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void TIM2_IRQHandler(void)
{
  soft_timer_tick();
  HAL_TIM_IRQHandler(&htim2);
}
void DMA1_Channel2_3_IRQHandler(void)
{
}
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        /* cmd 0x03 response byte: enqueue into the ring buffer + re-arm the
           1 byte reception only. Framing / checksum / Ch_senser[] update run
           in the main loop (sensor_rx_task()). */
        sensor_rx_isr_handler();
    }
		if(huart->Instance == USART2)
		{
		}
}
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

extern LPTIM_HandleTypeDef             LptimHandle;

void ADCx_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc);
}

void LPTIM1_IRQHandler(void)
{
	Rt_tick++;
  HAL_LPTIM_IRQHandler(&LptimHandle);
}


/* RTC 中断：本工程已删除全部 RTC 逻辑（RTC 不再初始化、也不再作为唤醒源），
   这里保留空实现，只为满足启动文件向量表里的符号引用。
   唤醒/计时只由 LPTIM1（100ms）与 TIM2（10ms）承担。 */
void RTC_IRQHandler(void)
{
  /* 空实现：RTC 中断源已在本工程中全部关闭（见审核报告 §3） */
}
