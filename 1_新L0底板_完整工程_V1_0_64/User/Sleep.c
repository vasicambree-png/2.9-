#include "sleep.h"
#include "ui.h"
#include "usart.h"
#include "gpio.h"
#include "Sx126x_Driver.h"
#include "LoraPro.h"
#include "ui.h"       /* ALL_UI UI / CDisOpt（关机等待页 UI 归零用） */
#include "New_Key.h"   /* Flash_Set Setup（开机关机标志位） */
extern Flash_Set Setup;
extern CDisOpt DisOpt;
#include "spi.h"
#include "lptim.h"
#include "iwdg.h"
#include "New_key.h"      /* KEY1/KEY3/KEY_ON 宏、key_state_reset()：关机模式要用 */

volatile uint8_t sleep_mode=0;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi1;
extern  void SystemClock_Config(void);
extern  uint16_t sleep_cnt;      /* V1_0_60：power_off_boot() 里清息屏计时（定义在 main.c） */
/* g_rt_tick_alive：由 Core/Src/lptim.c 的 Lptim_Init() 置位，
   1 = LPTIM1 的 100ms 中断确实在跑、Rt_tick 可信。为 0 时 sleep_stop_ms()
   拒绝进 STOP（否则这一段 STOP 没有任何定时唤醒源，只能等看门狗复位）。 */
extern uint8_t g_rt_tick_alive;
/* 按键 EXTI 置位（本文件末尾定义），STOP 分块循环里用它提前收尾 */
extern volatile uint8_t g_key_wake_flag;
uint8_t Oled_open_flag=0;
uint16_t sleep_cnt_num=0;
/* 进出 STOP 的 100ms 节拍对比（诊断 + 射频模式切换闸门，见 LoraPro.c 的 CAD_work）
   g_stop_ticks_last ：上一段 STOP 跨过的 100ms 拍数（0 = 一拍都没跨）
   g_stop_woke_early ：1 = 早醒（被 EXTI/噪声立刻叫醒），调用方据此补等射频切换
   g_stop_early_cnt  ：累计早醒次数（纯诊断，J-Link 可读） */
volatile uint32_t g_stop_ticks_last = 0;
volatile uint8_t  g_stop_woke_early = 0;
volatile uint32_t g_stop_early_cnt  = 0;

/* ==========================================================================
 * V1_0_57：LPTIM1 活性检测（没有 RTC 之后，LPTIM1 是 STOP 下**唯一**的定时
 * 唤醒源和全工程唯一时基 Rt_tick 的时钟源；它"静默死掉"必须可检测、可自愈）
 * --------------------------------------------------------------------------
 * 两个检测口，互相独立：
 *   ① 早醒侧（sleep_stop_ms）：连续 LORA_LPTIM_DEAD_SEGS 段 STOP 都"一拍都没
 *      跨过"**且**整段时间 Rt_tick 一次没变 -> 判定 100ms 拍没来 -> 重新武装；
 *   ② 清醒侧（sleep_timer）：清醒累计超过 LORA_LPTIM_AWAKE_MS(3s) 而 Rt_tick
 *      一次没变 -> 同样重新武装（这条在亮屏/读窗口这些"根本不进 STOP"的路径上
 *      也能工作）。
 * 为什么要求"Rt_tick 一次没变"：被 DIO/按键/光正常叫醒时 g_stop_ticks_last 也
 * 是 0，那种情况 Rt_tick 一定在涨，绝不会误判 -> 不会无谓地重排定时器相位。
 * 幂等：重新武装后立刻清计数；失败也清计数，下次再试，绝不 while 等待。
 * ========================================================================== */
#define LORA_LPTIM_DEAD_SEGS   20U     /* 连续 20 段早醒（约 2s）且 Rt_tick 未变 */
#define LORA_LPTIM_AWAKE_MS  3000U     /* 清醒侧：3s 内 Rt_tick 不走就算死       */
static uint16_t s_lptim_dead_segs  = 0U;
static uint32_t s_lptim_dead_tick0 = 0U;
static uint32_t s_awake_h0         = 0U;
static uint16_t s_cnt_seen         = 0U;   /* 上一次采到的 LPTIM1->CNT（硬件判活用） */
volatile uint32_t g_lptim_rearm_cnt = 0U;   /* 诊断：LPTIM1 重新武装次数 */
volatile uint16_t g_lptim_cnt_before = 0U;   /* 进 STOP 前的 LPTIM1->CNT（硬件证据） */
volatile uint16_t g_lptim_cnt_after  = 0U;   /* 醒来后的 LPTIM1->CNT                 */
volatile uint8_t  g_lptim_hw_moved   = 0U;   /* 1 = 硬件计数器确实动过（STOP 期间）   */
uint32_t light_tick=0;
volatile uint32_t g_light_chk_cnt = 0U;   /* 光检测次数(诊断) */
extern volatile uint32_t Rt_tick;

/* stop mode power config: ultra low power + fast wakeup, HSI after wake */
void SystemPower_Config(void)
{
  HAL_PWREx_EnableUltraLowPower();
  HAL_PWREx_EnableFastWakeUp();
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
}

void SystemClock_Config_MSI(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI| RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void SystemPower_Config_MSI(void)
{
  HAL_PWREx_EnableUltraLowPower();
  HAL_PWREx_EnableFastWakeUp();
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
}

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

void EXTI_wakeup1(void)
{
   __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void ALL_IO_CLOSE(void)
{
  GPIO_InitTypeDef GPIO_InitStructure= {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  UART1_DeInitSafe();  /* Clear UART state before the MSP disables its clock. */

  HAL_ADC_DeInit(&hadc);
  HAL_ADC_MspDeInit(&hadc);

  HAL_TIM_Base_DeInit(&htim2);
  HAL_TIM_Base_MspInit(&htim2);

  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin =  GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull =  GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);



	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //红外接收
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); //光唤醒供电
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //屏幕供电控制
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //触摸供电控制
	
	//关闭原来的蓝牙
		GPIO_InitStructure.Pin = GPIO_PIN_2;        
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
			
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);//蓝牙供电控制
	

	GPIO_InitStructure.Pin =GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
  /* 息屏即给 CH584M（屏幕 + 蓝牙 + 采集）整机断电；PB4 只由 lcd_power_off() 操作。 */
  lcd_power_off();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);


  /* 机械键 K1/K2/K3 必须与唤醒态一致：上升沿中断 + 下拉。
     配成 GPIO_MODE_INPUT 会顺带关掉 EXTI，息屏下按键唤醒就失效。 */
  GPIO_InitStructure.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 非按键脚(PB6/7/8/13)设模拟省电；触摸脚 PB14/PB15 必须按唤醒态配成
     下降沿中断 + 上拉（模拟模式下 IDR 恒 0，会被当成"一直在触摸"）。 */
  GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_13;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

void Lora_io_close(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure= {0};
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  HAL_SPI_DeInit(&hspi1);
  HAL_SPI_MspDeInit(&hspi1);
}

void ALL_close_lora(void)
{
  HAL_SPI_MspDeInit(&hspi1);
  HAL_SPI_DeInit(&hspi1);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure= {0};

  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
}

void Lora_io_open(void)
{
  	EXTI_wakeup();
	MX_SPI1_Init();
}

void EXTI_wakeup(void)
{
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;   // DIO3
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    GPIO_InitStruct.Pin = GPIO_PIN_4;   // DIO1
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;   // DIO2
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* sleep_stop_ms ：进一段有界 STOP（毫秒 -> 100ms 拍，向上取整）。
   唤醒源是 LPTIM1 的 100ms 中断（LSI，STOP 下仍在走）与各路 EXTI，不再使用 RTC。
   返回 1 = 真的睡过；0 = 没睡（读取窗口内 / Rt_tick 不可信）。
   硬约束：g_rt_tick_alive==0 时绝不进 STOP（没有定时唤醒源只能等 IWDG 复位）；
   g_slp_keep_awake!=0 时不进 STOP（USART1 用 HSI，STOP 下停振会丢应答）。 */
static uint8_t sleep_stop_ms(uint16_t ms)
{
  uint32_t t0;
  uint32_t want;          /* 需要跨过的 100ms 拍数（向上取整，至少 1 拍） */
  uint8_t  slept = 0U;
  uint8_t  guard = 0U;

  if (g_slp_keep_awake != 0U)
  {
    return 0U;                    /* 息屏读数据窗口：绝不能进 STOP */
  }
  if (g_rt_tick_alive == 0U)
  {
    return 0U;                    /* 没有定时唤醒源：绝不"睡到看门狗复位" */
  }

  want = ((uint32_t)ms + 99U) / 100U;
  if (want == 0U)
  {
    want = 1U;
  }

  re_iwdg();                      /* 进 STOP 前先喂一口 */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  t0 = Rt_tick;
  /* V1_0_57：记下进入 STOP 前的**硬件计数值**，醒来再读一次，用作"100ms 拍到底有没有来"
     的硬件证据（不再只靠软件 Rt_tick 推算 —— 那是循环论证）。 */
  g_lptim_cnt_before = Lptim_CntRead();

  while (((uint32_t)(Rt_tick - t0) < want) && (guard < 12U))
  {
    guard++;
    /* LPTIM1 的 ARRM 中断（100ms）把 CPU 从 STOP 叫醒；EXTI 也可以 */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    slept = 1U;

    if (Rt_tick == t0)
    {
      break;                      /* 一拍都没跨：被 EXTI 早醒，立刻交还主循环 */
    }
    if (g_key_wake_flag != 0U)
    {
      break;                      /* 按键唤醒：不再等剩下的拍 */
    }
    re_iwdg();                    /* IWDG 在 STOP 下照常计数 */
  }

  /* 节拍对比：供 LoraPro.c 的 CAD_work() 判断射频是否已走完模式切换 */
  g_stop_ticks_last = (uint32_t)(Rt_tick - t0);
  g_stop_woke_early = (g_stop_ticks_last == 0U) ? 1U : 0U;   /* 软件判定（供 LoraPro 的射频切换闸门用，语义保持不变） */
  /* V1_0_57：硬件判定 —— CNT 有没有跨过（只读 ARR/CNT，不动标志位）。
     g_lptim_hw_moved=0 表示"这段 STOP 里芯片的 100ms 计数器一步都没走"，
     那才是真正的"定时器没在跑"。它比软件 Rt_tick 更硬：Rt_tick 是中断累加的，
     而 CNT 是 LSI 直接驱动的，两者独立可交叉验证。 */
  g_lptim_cnt_after = Lptim_CntRead();
  g_lptim_hw_moved  = (g_lptim_cnt_after != g_lptim_cnt_before) ? 1U : 0U;
  if (g_stop_woke_early != 0U)
  {
    g_stop_early_cnt++;
  }

  /* V1_0_57：LPTIM1（100ms 拍）活性检测 -> 死了就重新武装（见文件头的长注释）。
     判据必须同时满足"连续 N 段早醒"与"Rt_tick 一次没变"，正常被 DIO/按键叫醒
     时 Rt_tick 一定在涨，不会误触发。 */
  if ((g_stop_ticks_last == 0U) && (g_lptim_hw_moved == 0U))
  {
    if (s_lptim_dead_segs == 0U) { s_lptim_dead_tick0 = Rt_tick; }
    if (s_lptim_dead_segs < 250U) { s_lptim_dead_segs++; }
    if ((s_lptim_dead_segs >= LORA_LPTIM_DEAD_SEGS) && (Rt_tick == s_lptim_dead_tick0))
    {
      s_lptim_dead_segs = 0U;
      g_lptim_rearm_cnt++;
      Lptim_ReArm_Tick100ms();     /* 停/重配/启动，见 Core/Src/lptim.c */
    }
  }
  else
  {
    s_lptim_dead_segs = 0U;
  }

  if (slept != 0U)
  {
    SystemClock_Config();         /* STOP 唤醒后必须重配系统时钟 */
    SystemPower_Config();
  }
  re_iwdg();                      /* STOP 期间 IWDG 在计数，醒来马上喂一口 */
  return slept;
}

/* brief stop-mode sleep used by LoRa CAD / TX wait */
void work_sleep(void)
{
  sleep_timer();
}


#define LORA_WAIT_CHUNK_MS   100U   /* LoRa 等待 STOP 的兜底唤醒周期（必须远小于 2.08s） */

void sleep_timer(void)
{
  uint8_t i;


  /* V1_0_57：清醒侧 LPTIM1 活性检测 —— **直接读硬件计数器 CNT**。
     判据：用 SysTick(HAL_GetTick，与 LPTIM1 完全独立的硬件时基) 观察 3s，
     若 LPTIM1->CNT 在这 3s 里一次都没变 => 100ms 拍没来 => 定时器死了 => 重新武装。
     为什么不用 Rt_tick：Rt_tick 就是本中断累加的，用它证明自己活着是循环论证。
     CNT 在活着的状态下每 0.84ms 就变一次，3s 内几乎必然变化 => 不会误判。 */
  {
    uint32_t h_now = HAL_GetTick();
    uint16_t c_now = Lptim_CntRead();
    if (s_awake_h0 == 0U)
    {
      s_awake_h0 = h_now;
      s_cnt_seen = c_now;
    }
    else if (c_now != s_cnt_seen)
    {
      s_cnt_seen = c_now;
      s_awake_h0 = h_now;               /* 计数器在动 -> 判活，继续观察 */
    }
    else if ((uint32_t)(h_now - s_awake_h0) > LORA_LPTIM_AWAKE_MS)
    {
      g_lptim_rearm_cnt++;
      Lptim_ReArm_Tick100ms();
      s_awake_h0 = h_now;
      s_cnt_seen = Lptim_CntRead();
    }
  }

  g_stop_woke_early = 0U;

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

  if (g_slp_keep_awake != 0U)
  {
    /* 息屏读数据窗口：有界等待 + 持续搬运串口 FIFO（绝不进 STOP） */
    for (i = 0U; i < 10U; i++)
    {
      sensor_rx_task();             /* 分帧 + 校验 + 填 Ch_senser[]，FIFO 不溢出 */
      re_iwdg();
      HAL_Delay(1U);
    }
    return;
  }


  if (UI.lcd_open != 0U)
  {
    for (i = 0U; i < 10U; i++)
    {
      sensor_rx_task();             /* 0x03/0x06 应答不能丢 */
      re_iwdg();
      HAL_Delay(1U);
    }
    return;                         /* ★ 不进 STOP：串口时钟必须保持活着 */
  }

  /* 有界 STOP（只在息屏走这里）：由 LPTIM1 的 100ms 中断或 DIO/按键/光 EXTI 唤醒 */
  if (sleep_stop_ms((uint16_t)LORA_WAIT_CHUNK_MS) == 0U)
  {

    HAL_Delay(1U);
    g_stop_woke_early = 1U;
  }
  re_iwdg();
}

void sleep_check(void)
{
   switch(sleep_mode)
   {
     case 0: // normal work
        if (Setup.On_Off_flag == 1u) { RfNetPro(); }   /* 关机标志位: 不组网 */
        Light_check();
        break;
     case 1: // enter quiet sleep
        printf("Queit_Into\r\n");
        UI.lcd_open=0;
        re_iwdg();
        SX126X_WOR_Execute(Sx1268_Sleep);
        ALL_IO_CLOSE();          /* 里面对 PB4 断电：CH584M / 屏幕整机断电 */
        re_iwdg();
        sleep_read_begin();      /* 复位息屏读数据状态机 + 记"下次读取时刻" */
        sleep_mode=2;
#if !SLP_READ_ENABLE
        work_sleep();            /* SLP_READ_ENABLE=0 时保持旧行为：直接进 STOP */
#endif
        printf("5555555 ==%d\r\n",1);
        break;
     case 2:
        /* 息屏常驻：每次主循环只推进一步，绝不阻塞（旧实现是 while 死循环，
           会把 LoRa / 光检测 / 按键 / 喂狗一起停掉）。 */
#if SLP_READ_ENABLE
        if (Setup.On_Off_flag == 1u)
        {
            sleep_read_service();    /* 上电 -> 读 0x03 -> 断电，分步推进 */
        }
        /* 关机标志位(On_Off_flag==0): 不做息屏读蓝牙数据 */
        if (sleep_mode != 2U)
        {
            break;               /* 读数据过程中被触摸/光唤醒，交给 case 3 */
        }
#endif
        Light_check();           /* 光检测：息屏下的光唤醒必须靠它 */
        if (sleep_mode == 2U)
        {
            /* 空闲睡一段省电。真睡成功了就记一段，作为 Rt_tick 万一在 STOP
               下不计数时的**备用计时基准**（见 usart.c 的说明）。 */
            if (sleep_stop_ms((uint16_t)SLP_SLEEP_CHUNK_MS) != 0U)
            {
                sleep_read_chunk_tick();
            }
        }
        break;
     case 3:
        g_slp_keep_awake = 0U;   /* 唤醒：放开 STOP 限制，别让 LoRa 再也睡不下去 */
        ALL_init();
        sleep_mode=0;
        break;
     case 4:
        break;
     default:
        break;
   }
}

static uint8_t last_sample = 0xFF;
static uint8_t output_state = 0;
static uint8_t debouncing = 0;
#define LIGHT_CONFIRM_TICKS  3U   /* ★ V1_0_47：光电平变化需连续稳定这么多次才认可并唤醒 */
static uint8_t stable_cnt = 0;

void System_check_Init(void) {
    Light_on;
    HAL_Delay(10);
    uint8_t init_level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    Light_off;
    printf("init_level ==%d\r\n",init_level);
    last_sample = init_level;
    output_state = init_level;
    debouncing = 0;
    stable_cnt = 0;
}

void Light_check(void)
{
    uint8_t level;
    static uint8_t first_run = 1;

    if (TickDiff_100ms(light_tick) > 5) /* 6 x 100ms; retain 3 confirmations. */
    {
        g_light_chk_cnt++;
        Light_on;
        /* 点亮光敏 LED 后等稳定：拆成 10 × (泵串口 + 喂狗 + 1ms)。
           不能 HAL_Delay(10) 死等 —— USART1 环形缓冲只有 128B（≈11ms@115200），
           死等会把应答帧顶掉，期间也没人喂狗。总时长与原来一致（≈10ms）。 */
        {
            uint8_t w;
            for (w = 0U; w < 10U; w++)
            {
                sensor_rx_task();        /* 0x03/0x06 应答不能丢 */
                re_iwdg();
                HAL_Delay(1U);
            }
        }
        level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
        Light_off;
        if(first_run)
        {
            output_state = level;
            last_sample = level;
            first_run = 0;
            return;
        }
        light_tick = Rt_tick;

        if (level == last_sample)
        {
            if (debouncing)
            {
                stable_cnt++;
                /* 新电平必须连续 LIGHT_CONFIRM_TICKS 次（3 次 ≈ 1.8s）不变才认可，
                   否则光噪声（1~5 秒振荡）会反复把设备点亮再入睡、白耗电。 */
                if (stable_cnt >= LIGHT_CONFIRM_TICKS)
                {
                    if (output_state != level)
                    {
                        output_state = level;
                        printf("Light state changed to %u\r\n", (unsigned)level);
                        if (sleep_mode == 2)
                        {
                            sleep_mode = 3;     /* 只有确认过的变化才允许光唤醒 */
                        }
                    }
                    debouncing = 0;
                    stable_cnt = 0;
                }
            }
        }
        else
        {
            /* 只是"与上次不同"：进入去抖确认流程，**绝不在这里直接唤醒** */
            if (!debouncing)
            {
                debouncing = 1;
                stable_cnt = 0;
            }
            else
            {
                stable_cnt = 0;     /* 又变了：确认计数重来 */
            }
            last_sample = level;
        }
    }
}
/* 软关机模式（同时长按 K1+K3，见下方 power_off_* 一组函数）
   关机态只允许 6 个按键 EXTI 唤醒（屏蔽 LoRa DIO 与光敏），
   分块 STOP + 每段喂狗，因为 IWDG 走 LSI、在 STOP 下照常计数（约 2.0s）。 */

#define PWR_OFF_NONE        0u      /* 正常（含普通息屏） */
#define PWR_OFF_STANDBY     1u      /* 已关机 */
#define PWR_OFF_PROMPT      2u      /* 提示页 */
#define PWR_OFF_SHUTTING    3u      /* 正在关机中：屏幕亮着显示"正在关机中"，5 秒后再断电 */

#define PWR_OFF_HOLD_TICK   20u     /* K1+K3 长按门限：20 x 100ms = 2 秒 */
#define PWR_OFF_PROMPT_SEC  10u     /* ★ V1_0_50：提示页无操作超时 20->10 秒（现场要求）；超时后走"正在关机中"5 秒再断电 */
#define PWR_OFF_SHUTDOWN_SEC 5u     /* "正在关机中"提示页停留时长：5 秒 */
#define PWR_OFF_CHUNK_MS    500u    /* 关机态每次 STOP 时长（必须 < IWDG 2.076s） */
#define PWR_OFF_PROMPT_RESEND_MS 1000u  /* ★ V1_0_50：提示帧重发间隔（毫秒，走 SysTick） */
#define PWR_OFF_PROMPT_BURST_MS    300u    /* V1_0_59：进提示页后前 1 秒的密集补发间隔（CH584M boot 约 0.5s） */

uint8_t  g_power_off_state       = PWR_OFF_NONE;
uint16_t g_power_off_prompt_sec  = 0u;
volatile uint8_t g_key_wake_flag = 0u;   /* 由 stm32l0xx_it.c 的按键 EXTI 置位 */

static uint8_t  s_k13_active = 0u;
static uint32_t s_k13_start  = 0u;
static uint8_t  s_k13_fired  = 0u;
static uint8_t  s_k13_need_release = 0u;  /* 手势已生效，必须等两个键都松开才能再次触发 */
static uint32_t s_prompt_tick = 0u;
static uint32_t s_prompt_send  = 0u;   /* V1_0_59：上次发提示帧的时刻（提示帧重发用） */
static uint8_t  s_prompt_burst = 0u;   /* V1_0_59：剩余密集补发次数（CH584M 上电要 ~0.5s） */
static uint32_t s_standby_loops  = 0u;   /* 关机态循环拍数（诊断） */
static uint32_t s_spurious_wakes = 0u;   /* 被噪声伪唤醒、经确认后丢弃的次数（诊断） */
static uint32_t s_shutdown_tick = 0u;   /* 进入"正在关机中"的时刻 */
static uint32_t s_shutdown_send = 0u;   /* 上次重发"正在关机中"提示帧的时刻 */

static uint8_t k13_pressed(void)
{
    return (uint8_t)((KEY1 == KEY_ON) && (KEY3 == KEY_ON));
}

/* 复位 K1+K3 长按计时。进/出关机、开机后都要调，
   否则"上一次按住的那点时间"会被算进这一次。 */
void power_off_key_reset(void)
{
    s_k13_active = 0u;
    s_k13_start  = 0u;
    s_k13_fired  = 0u;
}

/* 主循环调用：同时长按 K1+K3 满 2 秒时返回 1（只报一次；松手后才能再次触发） */
uint8_t key_k1k3_hold_2s(void)
{
    if (k13_pressed() == 0u)
    {
        s_k13_need_release = 0u;      /* ★ 两个键都松开了：解除"必须先松手"的约束 */
        power_off_key_reset();
        return 0u;
    }

    /* 手势已经生效过一次（返回过 1），必须等 K1、K3 都松开才允许再次触发，
       否则主循环下一拍就重新计时，长按开机后立刻又跳回关机。 */
    if (s_k13_need_release != 0u)
    {
        return 0u;
    }

    if (s_k13_active == 0u)
    {
        s_k13_active = 1u;
        s_k13_start  = Rt_tick;
        s_k13_fired  = 0u;
        printf("[KEY] k13 down rt=%lu\r\n", (unsigned long)Rt_tick);
        return 0u;
    }

    if (s_k13_fired != 0u) return 0u;      /* 已经报过一次，等松手 */

    /* V1_0_63：按键判定**只用 Rt_tick 单一 100ms 时基**（不用双时基）。
       Rt_tick 由 LPTIM1 的 ARRM 中断累加，LPTIM1 现在不会再被关闭（见 lptim.c），
       所以这条判据是可靠的。 */
    if ((uint32_t)(Rt_tick - s_k13_start) >= PWR_OFF_HOLD_TICK)
    {
        s_k13_fired        = 1u;
        s_k13_need_release = 1u;      /* ★ 必须等松手，见上面的说明 */
        printf("[KEY] k13 fire rt=%lu d_tick=%lu\r\n",
               (unsigned long)Rt_tick,
               (unsigned long)(uint32_t)(Rt_tick - s_k13_start));
        return 1u;
    }
    return 0u;
}

/* 关机态唤醒源屏蔽：1 = 只留 6 个按键；0 = 恢复 LoRa 三条 DIO 线 */
void power_off_mask_wakeup(uint8_t mask_on)
{
    if (mask_on != 0u)
    {
        EXTI->IMR &= ~((uint32_t)((1UL << 0) | (1UL << 4) | (1UL << 5)));
    }
    else
    {
        EXTI_wakeup();      /* 重新配置并恢复 LoRa 的 EXTI */
    }
}

/* 真正的关机动作：CH584M/屏幕断电 + 只留 6 个按键能唤醒。
   由"正在关机中"提示页计时 5 秒后调用（见 power_off_service 的 SHUTTING 分支）。 */
static void power_off_enter_standby(void)
{
    printf("[PWR] shutdown timer expired -> STANDBY\r\n");

    g_power_off_state      = PWR_OFF_STANDBY;
    g_power_off_prompt_sec = 0u;
    g_key_wake_flag        = 0u;

    sleep_mode  = 0;                 /* 与普通息屏解耦 */
    UI.lcd_open = 0;                 /* 停掉 UI 与软定时器 */
    lcd_power_off();                 /* 屏幕 + CH584M 断电 */

    /* V1_0_59：这里**不再 DeInit USART1**（原来 MspDeInit+DeInit）。
       原因：DeInit 之后回提示页必须重新 HAL_UART_Init()，而它的尾部用
       HAL_UART_TIMEOUT_VALUE(0x1FFFFFF，约 9.3 小时)等 TEACK，中途不喂狗；
       一旦 HSI16/USART1 时钟没就绪就是必然的 IWDG 复位（现场日志坐实：
       "key pressed -> show boot prompt page" 之后 2.13s 出现 IWDG 复位，
       表现为"关机后按键直接跳主页"而不是停在开机提示页）。
       正常息屏路径 ALL_IO_CLOSE() 里的 DeInit 保持不变（那条路径本来就正常）。
       只要把 PB6/PB7 置模拟即可防反灌：USART1 外设与寄存器一直有效，
       提示页只把引脚恢复成 AF0 就能立刻发帧，全程不碰 HAL_UART_Init()。 */
    /* 调试口 USART2 不动（power_off_to_prompt 要打印）。 */
    {
        GPIO_InitTypeDef gi_usart = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gi_usart.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
        gi_usart.Mode = GPIO_MODE_ANALOG;
        gi_usart.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &gi_usart);
    }
    power_off_mask_wakeup(1u);       /* 只留按键能唤醒 */
    /* 清掉"必须先松手"护栏：否则它会泄漏到开机提示页，把那里的长按开机屏蔽掉 */
    s_k13_need_release = 0u;
    power_off_key_reset();
    key_state_reset();
    /* 屏蔽之后再打印：IMR 的 bit0/4/5（LoRa DIO）应为 0，bit6/8/9/10/14/15 应为 1 */
    printf("[PWR] EXTI_IMR=%08lX  keyPUPDR_A=%08lX\r\n",
           (unsigned long)EXTI->IMR, (unsigned long)GPIOA->PUPDR);
    re_iwdg();
}

/* 被按键唤醒 -> 切到提示页 */
static void power_off_to_prompt(void)
{
    GPIO_InitTypeDef gi = {0};

    printf("[PWR] key pressed -> show boot prompt page\r\n");
    re_iwdg();   /* 唤醒到提示页这段可能被屏上电拖长：进门先喂一口 */

    g_power_off_state      = PWR_OFF_PROMPT;
    g_power_off_prompt_sec = 0u;
    /* 提示页/关机页的计时用 SysTick(HAL_GetTick)：Rt_tick 走 LSI，实测偏快 16%，
       会让"10 秒倒数"变成 8.4 秒；SysTick 走 HSI16，实测 0.24% 误差。 */
    s_prompt_tick          = HAL_GetTick();
    s_prompt_send          = HAL_GetTick();   /* V1_0_59：密集补发起点 */
    s_prompt_burst         = 3u;              /* V1_0_59：前 1 秒补发 3 次（0/300/600ms） */

    /* 恢复屏幕/CH584M 供电并重配协议口。这里不调 ALL_init()：
       那会连 LoRa 一起重新初始化，而提示页期间不希望 LoRa 参与。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gi.Pin  = GPIO_PIN_4;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gi);
    lcd_power_on();

    /* V1_0_59：只恢复引脚，不调 HAL_UART_Init()（理由见 power_off_enter_standby）。 */
    {
        GPIO_InitTypeDef gi_u1 = {0};
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gi_u1.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        gi_u1.Mode      = GPIO_MODE_AF_PP;
        gi_u1.Pull      = GPIO_NOPULL;
        gi_u1.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gi_u1.Alternate = GPIO_AF0_USART1;
        HAL_GPIO_Init(GPIOB, &gi_u1);
        __HAL_UART_ENABLE(&huart1);
        if (huart1.gState != HAL_UART_STATE_READY)
        {
            huart1.gState = HAL_UART_STATE_READY;
        }
    }
    re_iwdg();
    UI.lcd_open = 1;
    /* 提示等待开机页: UI 导航状态归零, 只保留开关机功能 */
    UI.Setting_flag = 0u;
    UI.Sub2         = Sub2_None;
    UI.Addr         = Set_Addr1;
    DisOpt          = Opt0;

    /* 提示页是全新上下文（这里的 K1+K3 长按含义是开机）：必须清掉关机那刻留下的
       "必须先松手"护栏，否则开机长按会被 key_k1k3_hold_2s() 直接屏蔽掉。 */
    s_k13_need_release = 0u;
    power_off_key_reset();
    key_state_reset();

    /* 提示帧必须配一帧普通页面帧，屏幕才会亮：CH584M 的点屏策略只认 menu_rank 1~4
       的页面帧，提示帧是 menu_rank=6，不参与点亮。刚上电时两帧都可能丢，
       power_off_service() 每秒会重发。 */
    UI_force_redraw();
    UI_send_frame();
    UI_show_save_result(UI_MSG_POWER_OFF);
    printf("[PWR] prompt ready +%lums\r\n", (unsigned long)(HAL_GetTick() - s_prompt_tick));
    re_iwdg();
}

/* TEST ONLY: 把设备直接推进"关机提示页"(PWR_OFF_PROMPT)。
   供 TEST_PWRWAIT 编译开关在台架上验证: 等待页期间主循环是否真的
   跳过了全部正常业务(键处理/UI刷新/LoRa/息屏读)。生产构建不会调用。 */
void power_off_test_to_prompt(void)
{
    power_off_to_prompt();
}

/* 开机：恢复全部正常功能 */
static void power_off_boot(void)
{
    printf("[PWR] K1+K3 held 2s -> BOOT, restoring all functions\r\n");
    printf("[PWR] standby loops=%lu spurious wakes=%lu\r\n",
           (unsigned long)s_standby_loops, (unsigned long)s_spurious_wakes);
    s_standby_loops = 0u; s_spurious_wakes = 0u;

    power_off_mask_wakeup(0u);       /* 恢复 LoRa 的 EXTI */
    lcd_power_on();
    UI.lcd_open = 1;

    ALL_init();                      /* 与息屏唤醒同一条路径：重新初始化整机 */

    /* V1_0_59：用户是长按 K1+K3 满 2 秒才开机的，此刻手通常还按着。
       不拦的话 Key_task 会把"开机那一按"当成一次新的按下，按住超过 1 秒
       就触发长按动作（页面乱跳 / 误进菜单）—— 即"开关机过程中按键串到别的逻辑"。 */
    key_wait_release();

    /* V1_0_60：开机后必须**强制醒着**。ALL_init() 里有
       `if(reset_flag==2){ sleep_mode=1; }` —— reset_flag 是"本次上电周期的复位原因"，
       由 CheckResetReason() 只写一次，之后**不再变化**。所以只要这一上电周期里发生过
       一次 IWDG 复位（reset_flag=2），此后每次长按 K1+K3 开机都会在 ALL_init 里被
       设成 sleep_mode=1 -> sleep_check() case 1 立刻息屏（日志：boot done 之后 0.3s
       就 Queit_Into、屏幕随即断电）—— 现场就是"开机后几秒进入静置休眠 / 页面丢失"。 */
    sleep_mode = 0;
    sleep_cnt  = 0;
    printf("[PWR] stay awake after boot (sleep_mode=0)\r\n");

    /* 开机后必须补发一帧 msg=0(UI_MSG_NONE)：CH584M 的关机提示是常驻提示，
       不会自己消失、普通页面帧也盖不掉，不清除就会一直显示"长按K1+K3开机"。
       连发 3 帧做冗余，防止 CH584M 还在启动时丢帧。 */
    {
        uint8_t k;
        for (k = 0U; k < 3U; k++)
        {
            UI_show_save_result(UI_MSG_NONE);
        }
    }
    printf("[PWR] power-off overlay cleared on CH584M (msg=0 x3)\r\n");

    g_power_off_state      = PWR_OFF_NONE;
    g_power_off_prompt_sec = 0u;
    g_key_wake_flag        = 0u;
    power_off_key_reset();
    printf("[PWR] boot done, normal mode\r\n");
}

/* 主循环调用，返回 1 = 本拍处于关机/提示页态，应跳过全部正常业务。
   power_off_service 是软关机状态机：SHUTTING(5s 提示) -> STANDBY(分块 STOP) ->
   PROMPT(按键唤醒后的开机提示页，超时回 STANDBY)。 */
/* 当前是否真有按键按着（轻触键 PA8/9/10 高有效；触摸键 PB14/PB15/PC6 低有效）。
   用于关机唤醒的去抖确认：触摸控制器在非工作态的噪声脉冲过不了这一关。 */
static uint8_t key_any_pressed_now(void)
{
    if (KEY1 == KEY_ON || KEY2 == KEY_ON || KEY3 == KEY_ON) return 1U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)  == GPIO_PIN_RESET) return 1U;
    return 0U;
}

uint8_t power_off_service(void)
{
    if (g_power_off_state == PWR_OFF_NONE)
    {
        return 0u;
    }

    re_iwdg();      /* 关机/提示页状态机每拍先喂狗，与具体分支无关 */

    /* ---- 正在关机中：屏幕亮着显示提示页，5 秒后再真断电 ---- */
    if (g_power_off_state == PWR_OFF_SHUTTING)
    {
        if ((uint32_t)(HAL_GetTick() - s_shutdown_tick) >= (uint32_t)(PWR_OFF_SHUTDOWN_SEC * 1000U))
        {
            power_off_enter_standby();       /* 时间到：断电 + 只留按键唤醒 */
            return 1u;
        }

        /* 每秒重发一次页面帧 + 提示帧（CH584M 可能还在启动，页面帧才点得亮屏） */
        if ((uint32_t)(HAL_GetTick() - s_shutdown_send) >= 1000U)
        {
            s_shutdown_send = HAL_GetTick();
            UI_force_redraw();
            UI_send_frame();
            UI_show_save_result(UI_MSG_SHUTTING_DOWN);
        }

        /* 这段只有 5 秒、用户正盯着屏幕看，不值得用 STOP 换那点电；
           而 STOP 是看门狗复位唯一的来源。 */
        HAL_Delay(20);
        re_iwdg();
        return 1u;
    }

    if (g_power_off_state == PWR_OFF_STANDBY)
    {
        /* 分块 STOP：每 500ms 醒一次喂狗，其余时间都在 STOP。
           只屏蔽了 LoRa/光敏，6 个按键的 EXTI 仍开着 -> 按任意键即可唤醒。 */
        re_iwdg();
        if (sleep_stop_ms((uint16_t)PWR_OFF_CHUNK_MS) == 0U)
        {
            /* 兜底：sleep_stop_ms() 在「Rt_tick 不可信(g_rt_tick_alive==0) /
               息屏读数据窗口内 / 唤醒源没配上」时会拒绝进 STOP 并立刻返回 0。
               若这里什么都不做，关机态就退化成「喂着狗的忙等死循环」——不会炸，
               但功耗完全不是关机该有的样子。落空时补一段有界短延时，保证循环有节拍。
               HAL_Delay 走 SysTick；没进 STOP 时 SysTick 是活的，安全。 */
            HAL_Delay(50);      /* 50ms，远小于 IWDG 的 2.0s */
        }
        re_iwdg();

        s_standby_loops++;

        if (g_key_wake_flag != 0u)
        {
            uint8_t i, hit = 0u;

            g_key_wake_flag = 0u;

            /* ★ 唤醒去抖确认：连续 3 次、每次间隔 8ms 都看到"确实有键按着"才认。
               真实的轻触/触摸按键会持续几十~几百毫秒，一定抓得住；
               而触摸控制器在非工作态吐出的窄脉冲噪声过不了这一关。
               不确认就直接上电 -> CH584M 被点亮一下又超时回关机，
               现场表现就是"屏幕定时闪一下残影"。 */
            for (i = 0u; i < 3u; i++)
            {
                if (key_any_pressed_now() != 0u) { hit = 1u; break; }
                HAL_Delay(8);
            }

            if (hit != 0u)
            {
                power_off_to_prompt();
            }
            else
            {
                s_spurious_wakes++;      /* 伪唤醒：丢弃，留在关机态 */
            }
        }
        return 1u;
    }

    /* ---- PWR_OFF_PROMPT ---- */

    /* 再长按 K1+K3 满 2 秒 -> 开机 */
    if (key_k1k3_hold_2s() != 0u)
    {
        power_off_boot();
        return 0u;                   /* 已开机，这一拍起恢复正常业务 */
    }

    /* 无操作超时：只有 K1+K3 能续命。这里不能用 key_any_pressed_now() ——
       触摸脚被外部拉死的设备读出来永远是"按下"，会让提示页永不超时。 */
    if (k13_pressed() != 0u) { g_power_off_prompt_sec = 0u; }
    /* V1_0_59：提示帧重发分两段 —— 进页面后的前 1 秒密集补发（300ms x 3），
       因为 CH584M 从断电到跑完 boot 约 0.5s，第一枪必然打在屏还没起来的时候；
       之后回到 1 秒一次。提示帧每次连发两遍，单帧丢了还有一次机会。 */
    {
        uint32_t gap = (s_prompt_burst != 0u) ? (uint32_t)PWR_OFF_PROMPT_BURST_MS
                                               : (uint32_t)PWR_OFF_PROMPT_RESEND_MS;
        if ((uint32_t)(HAL_GetTick() - s_prompt_send) >= gap)
        {
            s_prompt_send = HAL_GetTick();
            if (s_prompt_burst != 0u) { s_prompt_burst--; }
            /* 页面帧负责点亮 CH584M，提示帧负责内容；两帧都重发 */
            UI_force_redraw();
            UI_send_frame();
            UI_show_save_result(UI_MSG_POWER_OFF);
            UI_show_save_result(UI_MSG_POWER_OFF);
            printf("[PWR] prompt tx burst=%u +%lums\r\n",
                   (unsigned)s_prompt_burst,
                   (unsigned long)(HAL_GetTick() - s_prompt_tick));
        }
    }

    /* 1 秒计数 + 10 秒无操作超时 -> 走"正在关机中"（节奏与改动前一致） */
    if ((uint32_t)(HAL_GetTick() - s_prompt_tick) >= 1000U)
    {
        s_prompt_tick = HAL_GetTick();
#if TEST_PWRWAIT
        printf("[PWR] cal ms=%lu rt=%lu sec=%u\r\n",
               (unsigned long)HAL_GetTick(), (unsigned long)Rt_tick,
               (unsigned)g_power_off_prompt_sec);
#endif
        g_power_off_prompt_sec++;
        if (g_power_off_prompt_sec >= PWR_OFF_PROMPT_SEC)
        {
            /* 超时不直接断电：先进"正在关机中"亮 5 秒，与长按关机同一路径 */
            printf("[PWR] prompt timeout %us -> show shutting-down page\r\n",
                   (unsigned)PWR_OFF_PROMPT_SEC);

            g_power_off_state      = PWR_OFF_SHUTTING;
            g_power_off_prompt_sec = 0u;
            g_key_wake_flag        = 0u;
            s_shutdown_tick        = HAL_GetTick();
            s_shutdown_send        = HAL_GetTick();
            UI.lcd_open            = 1u;
            UI_show_save_result(UI_MSG_SHUTTING_DOWN);
            power_off_key_reset();
            key_state_reset();
            return 1u;
        }
    }

    /* 提示页改用 SysTick 短节拍，不进 STOP：它是最长 10 秒的用户可见状态，
       省这点电毫无意义，而 STOP 是看门狗复位唯一的来源。
       （关机待机态 PWR_OFF_STANDBY 仍然用 STOP，那里的省电才是必要的。） */
    HAL_Delay(20);
    re_iwdg();
    return 1u;
}

/* power_off_enter ：长按 K1+K3 满 2 秒时调用（公开接口）。
   不立刻断电，先进 PWR_OFF_SHUTTING 亮 5 秒"正在关机中"，
   再由 power_off_enter_standby() 真正断电，用户能看到关机过程。 */
void power_off_enter(void)
{
    printf("[PWR] K1+K3 held 2s -> shutting down, show prompt %us\r\n",
           (unsigned)PWR_OFF_SHUTDOWN_SEC);

    g_power_off_state      = PWR_OFF_SHUTTING;
    g_power_off_prompt_sec = 0u;
    g_key_wake_flag        = 0u;
    /* 关机提示页的 5 秒同样走 SysTick（Rt_tick 偏快 16%） */
    s_shutdown_tick        = HAL_GetTick();
    s_shutdown_send        = HAL_GetTick();

    UI.lcd_open = 1;                       /* 屏幕继续亮着，显示提示页 */
    /* 杜绝关机等待页下按 K2 等键进入设置页: 把 UI 导航状态强制归零(回主页) */
    UI.Setting_flag = 0u;
    UI.Sub2         = Sub2_None;
    UI.Addr         = Set_Addr1;
    DisOpt          = Opt0;
    UI_show_save_result(UI_MSG_SHUTTING_DOWN);

    power_off_key_reset();
    key_state_reset();
    /* 这里是**关机提示页刚进入**的时刻：LoRa/光敏的 EXTI 还没被屏蔽，
       所以 bit0/4/5 仍为 1；真正屏蔽在 power_off_enter_standby() 里做，
       那里会再打印一次屏蔽后的值作为对照。 */
    printf("[PWR] EXTI_IMR=%08lX  keyPUPDR_A=%08lX\r\n",
           (unsigned long)EXTI->IMR, (unsigned long)GPIOA->PUPDR);
    re_iwdg();
}
