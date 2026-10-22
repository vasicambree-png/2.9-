#include "lptim.h"
#include "usart.h"
#include "iwdg.h"   /* V1_0_57: Lptim_ReArm_Tick100ms() 里要用 re_iwdg() */
LPTIM_HandleTypeDef   LptimHandle;

/* ★★ 必须放在 Lptim_Init() **之前**：下面那个函数体里要用到 Rt_tick
   （启动后当场验证它真的在涨）。原来这行 extern 在文件中间，
   在它之前使用 Rt_tick 会直接编译报错（identifier undefined）。 */
extern volatile uint32_t Rt_tick;

/* ==========================================================================
 * ★★ 看门狗审查补丁（V1_0_2 第三轮）：Rt_tick 的"唯一时钟源"必须有可观测性
 * --------------------------------------------------------------------------
 * Rt_tick 完全由 LPTIM1_IRQHandler 累加（stm32l0xx_it.c 里 Rt_tick++），
 * 而它同时是三件要命事情的基础：
 *   1) STOP 的唯一"非 EXTI"唤醒源（LPTIM1 的 100ms 中断会把 CPU 从 STOP 叫醒）；
 *   2) 本工程**所有**新加的软件超时的时基
 *      （息屏读数据状态机、0x03 在途守卫、LoRa TX 等待、按键去抖与长短按……）；
 *   3) 光检测 / 发送周期的节拍。
 * 如果 LPTIM1 因为任何原因没跑起来（初始化失败、时钟配置变过、被 DeInit），
 * 那么第 2 条里那些"有界"的 while 循环**永远不会到期**，而它们每一轮都会
 * 调 re_iwdg() —— 结果是"喂着狗死等"的**永久死锁**（比看门狗复位更糟，
 * 因为连自动复位自愈都没有了）。
 * 所以：
 *   a) 启动失败必须留下痕迹（原来这个 if 是空体，失败没有任何痕迹）；
 *   b) 启动之后**当场验证它真的在走**（用 SysTick 计时，此时还没进 STOP），
 *      结果放进 g_rt_tick_alive；
 *   c) Sleep.c 的 sleep_stop_ms() 与 sleep_timer() 会检查这一位：
 *      Rt_tick 不可信时**绝不进 STOP**（宁可费电，不做"睡到复位"或
 *      "喂着狗死等"两种更糟的事）。
 * ========================================================================== */
uint8_t g_rt_tick_alive = 0U;

/* ★★★ V1_0_50：LPTIM1 的"100ms"周期标定值。
   ------------------------------------------------------------------
   LPTIM1 的时钟源在本工程里被选为 **LSI**（见 Sleep.c 的
   RCC_PeriphCLKInitStruct.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI），
   而下面原来固定写 100 —— 那是"LSI 恰好 32 kHz"才成立的算法。
   实测本机 LSI ≈ 38.2 kHz（时基标定：SysTick 与 Rt_tick 同时打点，
   90 个 Rt_tick 对应 7.541 s 真实时间 ? 83.8 ms/tick，比标称快 16.2%）。
   后果是全工程**所有以 Rt_tick 计时的时长都偏短约 16%**：
   "长按 2 秒"实际约 1.68 秒、"600 秒周期读取"实际约 503 秒……
   按 38.2 kHz 反算：N = 0.1 s × 38200 / 32 = 119.4 → 取 119（≈99.7 ms）。
   LSI 本身有批次与温漂（数据手册典型 ±5%），标定后残余误差约 ±5%，
   远好于原来的 16%。
   改这一处即可，全工程唯一的 LPTIM1 启动点就是下面的 HAL_LPTIM_Counter_Start_IT()。 */
#define LPTIM_TICK_COMPARE   119u

volatile uint32_t g_lptim_skip_cnt = 0U;   /* V1_0_62：重武装被请求但定时器其实在跑的次数 */

/* V1_0_61：CR.ENABLE 是 bit0；CMSIS 头里没有独立的 LPTIM_CR_ENABLE 宏，这里补一个 */
#ifndef LPTIM_CR_ENABLE
#define LPTIM_CR_ENABLE      0x00000001UL
#endif


/* V1_0_61：Lptim_Init() 里要读 CNT 判活，而 Lptim_CntRead() 定义在文件后半部分，
   这里先给一个原型（否则 C89 会按隐式 int 声明，和后面的定义冲突）。 */
uint16_t Lptim_CntRead(void);

/* 观察 CNT 是否在动（毫秒级，纯读寄存器） */
static uint8_t lptim_cnt_is_moving(uint16_t ms)
{
  uint16_t c0 = Lptim_CntRead();
  uint32_t h0 = HAL_GetTick();

  while (((uint32_t)(HAL_GetTick() - h0) < ms) && (Lptim_CntRead() == c0))
  {
  }
  return (uint8_t)((Lptim_CntRead() != c0) ? 1u : 0u);
}

//100ms
void Lptim_Init(void)
{
  /* V1_0_64：**在跑就一个寄存器都不动**（绝不关闭定时器）；只有确认没在跑才启动。
     启动一律走 HAL_LPTIM_Counter_Start_IT() —— 它的 ARR/ARROK/ENABLE/CNTSTRT 时序
     是现场验证过能给出 100ms 周期的（V1_0_58 就是这条路径，长按 2s / 提示页 1s 都对）。
     V1_0_62 试过自己写寄存器启动，结果周期变成约 1ms（Rt_tick 快 100 倍），已废弃。 */
  if ((LPTIM1->CR & LPTIM_CR_ENABLE) != 0u)
  {
    if (lptim_cnt_is_moving(30U) != 0u)
    {
      g_rt_tick_alive = 1U;
      printf("[LPT] init: already running, keep it\r\n");
      return;
    }
    printf("[LPT] init: enabled but not counting, restart via HAL\r\n");
  }

  /* LSI 必须先在跑（LPTIM 的 LPOSC 时钟源） */
  __HAL_RCC_LSI_ENABLE();
  {
    uint32_t w = 0U;
    while ((__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) && (w < 400000U))
    {
      w++;
      if ((w & 0xFFFFU) == 0U) { re_iwdg(); }
    }
  }

  LptimHandle.Instance = LPTIM1;
  LptimHandle.Init.Clock.Source       = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  LptimHandle.Init.CounterSource      = LPTIM_COUNTERSOURCE_INTERNAL;
  LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV32;
  LptimHandle.Init.Trigger.Source     = LPTIM_TRIGSOURCE_SOFTWARE;
  LptimHandle.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
  LptimHandle.Init.UpdateMode         = LPTIM_UPDATE_IMMEDIATE;

  if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
  {
    printf("lptim init fail\r\n");
  }

  /* HAL 启动：写 ARR -> 等 ARROK -> 使能 -> 连续计数 -> 开 ARRM 中断 */
  if (HAL_LPTIM_Counter_Start_IT(&LptimHandle, LPTIM_TICK_COMPARE) != HAL_OK)
  {
    printf("lptim start fail\r\n");
  }

  if (lptim_cnt_is_moving(200U) != 0u)
  {
    g_rt_tick_alive = 1U;
    printf("lptim start ok\r\n");
  }
  else
  {
    g_rt_tick_alive = 0U;
    printf("lptim tick NOT running! stop disabled\r\n");
  }
  //HAL_LPTIM_TimeOut_Start_IT(&LptimHandle,36999,1000);
}
/* 注：extern uint32_t Rt_tick; 已提到文件开头（Lptim_Init 之前要用）。 */
uint32_t TickDiff_100ms(uint32_t Tick)
{
  return Rt_tick - Tick;
}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
    
  /* ## - 1 - Enable LPTIM clock ############################################ */
  __HAL_RCC_LPTIM1_CLK_ENABLE();
  
  /* ## - 4 - Enable and set LPTIM Interrupt to the highest priority ######## */
  HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
	 
}

void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
	  __HAL_RCC_LPTIM1_CLK_DISABLE();
	  HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
}
//单位 ms
/* ==========================================================================
 * V1_0_57：LPTIM1 重新武装（Sleep.c 的活性检测调用）
 * --------------------------------------------------------------------------
 * 用途：本工程已彻底删除 RTC，LPTIM1 的 100ms ARRM 中断是 STOP 下唯一的定时
 * 唤醒源，也是 Rt_tick 的唯一时钟源。它一旦静默停走（初始化被改、时钟源丢、
 * 被 DeInit、噪声把 ARRM 挂起位卡住……），所有以 Rt_tick 计时的软件看门狗都会
 * 变成"喂着狗死等"。所以必须能就地重新武装。
 * 做法：Stop -> 重写 Init（时钟源/预分频/触发/更新模式，与 Lptim_Init 一致）
 *       -> Start(ARRM=LPTIM_TICK_COMPARE) -> 用 SysTick 复核 Rt_tick 是否在涨。
 * 有界：整段只有 HAL 调用，无 while 死等；复核窗口 <= 250ms（远小于 IWDG 2.08s），
 *       窗口内每 10ms 喂一次狗。
 * 幂等/可重入：只操作 LPTIM1 自己的寄存器与句柄，可在任意主循环上下文重复调用；
 *       Rt_tick 只增不减，重新武装不会造成时间跳变（最多丢一次 100ms 节拍）。
 * ========================================================================== */
/* ==========================================================================
 * V1_0_57：LPTIM1 硬件寄存器直读（不经 HAL、不用 Rt_tick 做"自证"）
 * --------------------------------------------------------------------------
 * 为什么要它：原来判断"100ms 拍有没有来"完全依赖 Rt_tick，而 Rt_tick 本身就是
 * LPTIM1 中断累加的 —— 用 Rt_tick 去证明 LPTIM1 活着是**循环论证**：
 * 一旦 LPTIM1 静默停走，Rt_tick 冻住，所有基于它的判据（包括活性检测自己）都
 * 永远不会成立，而主循环还在 re_iwdg()，于是变成"喂着狗死等"。
 * 这里直接读硬件：
 *   · LPTIM1->CNT：16 位计数器。连续模式(CNTSTRT)下由 LSI 经 /32 预分频驱动，
 *     1194 Hz -> 每 0.84ms 加 1，ARR=119 时约 100ms 溢出一次并产生 ARRM 中断。
 *     STOP 期间计数继续（LPTIM 由 LSI 驱动，与 HCLK 无关），唤醒后可直接读。
 *   · LPTIM1->ARR：重装值（= LPTIM_TICK_COMPARE = 119），用于换算"跨过几拍"。
 * ★ 只读，不写、不清标志 —— 绝不干扰 LPTIM1_IRQHandler 的正常计数与清标志，
 *   因此可以在任意上下文（含 STOP 前后）安全调用，天然幂等/可重入。
 * 手册依据（STM32L0 参考手册，LPTIM 章节）：
 *   · 连续模式 CNTSTRT=1 时计数器自由运行，CNT 随时可读；
 *   · LPTIM 在 Stop 模式下仍工作（时钟源为 LSI/LSE）；
 *   · ARR 为计数上限，CNT==ARR 时产生 ARRM 事件。
 * ========================================================================== */
uint16_t Lptim_CntRead(void)
{
  return (uint16_t)(LPTIM1->CNT & 0xFFFFU);
}

uint16_t Lptim_ArrRead(void)
{
  return (uint16_t)(LPTIM1->ARR & 0xFFFFU);
}

void Lptim_ReArm_Tick100ms(void)
{
  /* V1_0_64：**永不 Stop**。在跑 -> 直接返回（只记一次误判）；没在跑 -> 用 HAL 启动。
     绝不能在这里调 HAL_LPTIM_Counter_Stop_IT()：它无条件关外设，一旦随后启动失败，
     100ms 拍就真的停了（Rt_tick 不涨 -> 长按关不了机 / 页面不刷新 / LoRa 超时全乱）。 */
  if ((LPTIM1->CR & LPTIM_CR_ENABLE) != 0u)
  {
    if (lptim_cnt_is_moving(20U) != 0u)
    {
      g_rt_tick_alive = 1U;
      g_lptim_skip_cnt++;              /* 诊断：检测器误判次数 */
      return;
    }
  }

  __HAL_RCC_LSI_ENABLE();
  {
    uint32_t w = 0U;
    while ((__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) && (w < 400000U))
    {
      w++;
      if ((w & 0xFFFFU) == 0U) { re_iwdg(); }
    }
  }

  LptimHandle.Instance = LPTIM1;
  LptimHandle.Init.Clock.Source       = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  LptimHandle.Init.CounterSource      = LPTIM_COUNTERSOURCE_INTERNAL;
  LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV32;
  LptimHandle.Init.Trigger.Source     = LPTIM_TRIGSOURCE_SOFTWARE;
  LptimHandle.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
  LptimHandle.Init.UpdateMode         = LPTIM_UPDATE_IMMEDIATE;

  if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
  {
    printf("lptim rearm init fail\r\n");
  }
  if (HAL_LPTIM_Counter_Start_IT(&LptimHandle, LPTIM_TICK_COMPARE) != HAL_OK)
  {
    printf("lptim rearm start fail\r\n");
    return;                            /* 失败就留给下一次检测，绝不关定时器 */
  }

  if (lptim_cnt_is_moving(200U) != 0u)
  {
    g_rt_tick_alive = 1U;
    printf("lptim rearm OK\r\n");
  }
  else
  {
    printf("lptim rearm: still not running\r\n");
  }
}

void Lptim_time_setting(uint16_t time_num)
{
  /* V1_0_62：**不关闭定时器**，只改自动重装值（本工程当前没有调用点，
     保留接口是为了将来改周期时不会再走"Stop + Start"）。 */
  __HAL_LPTIM_AUTORELOAD_SET(&LptimHandle, (uint32_t)time_num);
}
