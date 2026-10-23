#include "New_Key.h"
#include "usart.h"
#include "string.h"
#include "flash.h"
#include <stdio.h>
#include <ui.h>
#include "LoraPro.h"

extern ALL_UI UI;
Flash_Set  Setup={0};
Flash_Set  backups_Setup={0};
uint16_t num_on=0;
extern  CDisOpt   DisOpt;
/* 键号宏 KEY1_PRES/KEY2_PRES/KEY3_PRES 原来定义在这里，现在移到
   New_key.h —— 那里集中放"按键相关的宏与对外接口"。
   ★ V1_0_6：EXTI 回调已不再使用键号（中断里不再产生按键事件），
   但仍留着统一放在头文件里，便于调试与后续维护。 */

/* ==========================================================================
 * 按键策略（V1_0_6 按需求冻结，**取代**原来的"边沿事件队列"方案）
 * --------------------------------------------------------------------------
 * ■ 现场现象（用户报的）：
 *     "轻触按键触发总是不灵敏，偶尔灵敏"；
 *     "复位代码以后要按两次或者数次 K2 才触发"；
 *     以及"一次按键触发多次动作"（中断太快、抖动被当多次按下）。
 *
 * ■ 原来为什么会用"边沿事件队列"（V1_0_2 的修正，现已废弃）：
 *     Key_task 是软定时器回调、只在主循环的 soft_timer_loop_async() 里执行，
 *     而主循环还要在 Lora_task() 里进 STOP / 跑 CAD（最长 600ms）、ADC、printf ——
 *     实际一拍往往上百毫秒。轻触键的输出脉冲只有几毫秒，轮询看电平会整片漏掉，
 *     所以在 EXTI 里把边沿压进队列补偿。
 *
 * ■ 现在改为**纯轮询方案**（需求：不能在中断里触发按键，只允许按键中断唤醒）：
 *     · 按键的识别 / 去抖 / 长短按 / 连按派发，**只**由 Key_task() 用
 *       key_scan() 轮询 GPIO 电平完成；Key_task 由 TIM2 的 10ms 节拍驱动
 *       （soft_timer_create(Key_task, 10, SOFT_TIMER_CB_MAIN)）。
 *     · **中断里不再产生任何按键事件**：EXTI 只负责把机器从 STOP / 软关机态
 *       唤醒（见 stm32l0xx_it.c 的 HAL_GPIO_EXTI_Callback），
 *       原来中断里的 flag_key=N; 与 key_evt_push(KEYx_PRES); 已整体删除。
 *     · 抖动防护不再依赖"采样间隔"，改为显式的"防重复触发窗口"：
 *       一次动作派发后，同一个键在窗口内不再接受新的按下 —— 这样实体键的
 *       抖动不会被当成"多次按下"（见下面 KEY_ACTION_GUARD_*）。
 *
 * ■ 代价（如实记录）：若某个按键脉冲短于 10ms 且恰好落在两次采样之间，
 *   会被漏掉 —— 这与旧的纯轮询版本行为一致。实体键人手按下通常 ≥50ms，
 *   不受影响；轻触键要看它自己的输出脉宽（原来那套队列就是为了补这个）。
 *   彻底解决仍建议把主循环的长阻塞拆掉（LoRa CAD 状态机化），
 *   见《V1_0_3_最新优化问题.md》P1-1 / P1-2。
 * ========================================================================== */
/* ★ V1_0_6：这里原本是一个 KEY_EVT_LEN = 32 的"按键边沿事件环形队列"
   （key_evt_push / key_evt_pop / key_evt_flush / key_evt_reset + s_key_evt[]
   + s_key_evt_head/tail + g_key_evt_drop_cnt），由 EXTI 中断生产、Key_task 消费。
   按需求已**整体移除**：中断不再产生按键事件，按键逻辑全部回到定时器轮询。
   原因与取舍见上面的"按键策略"说明。 */

extern volatile uint32_t Rt_tick;    /* 100ms 节拍，定义在 stm32l0xx_it.c（长短按 / 连按 / 防抖计时用） */

/* ==========================================================================
 * ★★★ V1_0_7 按键改造：采样与去抖搬进 **10ms 定时器中断**，主循环只负责派发
 * --------------------------------------------------------------------------
 * ■ 为什么还要改（V1_0_6 之后现场仍然"有时候按下没反应、串口也看不到按键指令"）：
 *     V1_0_6 把 Key_task 放在 10ms 软定时器上、但它是 SOFT_TIMER_CB_MAIN，
 *     **回调只在主循环里执行**；而主循环里有 LoRa CAD（最长 600ms）、LoRa 发送
 *     等待（最长 5s）、ADC 的 HAL_Delay、printf 等长阻塞。手指按下的那一下
 *     如果整段落在阻塞里，这一按就被整片漏掉 —— 串口既没有 0x01 帧、
 *     调试口也没有 KEYx_Short，表现就是"按键失效"。
 *     ★ 另外 V1_0_6 里的"防重复触发窗口"是按**采样拍数**计的（8 拍），
 *       主循环一慢，8 拍可能是几百毫秒，会把用户的后一次正常按键一起吞掉，
 *       本身就是"偶尔按下没反应"的一个来源。本次一并去掉。
 *
 * ■ 现在的做法（三步，互不越界）：
 *     1) **中断里只做采样**：key_tick_isr() 由 10ms 定时器（TIM2 节拍）在中断
 *        上下文里调用，读 6 个按键引脚 -> 连续 2 拍（20ms）一致才认 -> 更新
 *        "当前按住的键"；按下沿把键号写进**单槽粘滞寄存器** s_isr_key_sticky；
 *        同时累加按住拍数 s_isr_hold_ticks（长按计时用，10ms 分辨率）。
 *        中断里**不产生任何队列、不做任何 UI/串口动作**，只更新这几个变量。
 *     2) **主循环只取一次"粘滞电平"**：key_level_sticky()
 *          · 键还按着          -> 返回该键号（连续多拍，供状态机累时长/判长按）
 *          · 已抬起但没取过    -> 补返回一次该键号（当作"这一拍还按着"），
 *                                 下一拍再返回 0 -> 走原有的短按派发
 *        这样即使主循环被卡住几百毫秒，**那一按也不会丢**：粘滞值会一直等到
 *        主循环来取。一次按下仍然只派发一次动作（粘滞是单槽，不累积）。
 *     3) **去抖在中断里按真实时间做**（20ms），不再依赖"采样间隔"或"采样拍数"，
 *        所以主循环快慢都不影响抗抖动效果，也不会吞掉正常连按。
 *
 * ■ 代价（如实记录）：粘滞是**单槽**，不是队列。极端情况下"主循环被卡住期间
 *   用户按了两个不同的键、并且都松开了"，只有后一个键会被执行（前一个丢）。
 *   这是刻意取舍：V1_0_2 的 32 深队列换来的正是"一次物理按下被拆成多次动作"。
 *   真要两个都不丢，得先把主循环的长阻塞拆掉（见《V1_0_3_最新优化问题.md》P1-1/P1-2）。
 * ========================================================================== */

/* 读 6 个按键引脚，返回键号（0 = 都没有按下）。
   优先级与原 key_scan() 一致；只看电平，不做任何去抖/计时（那是下面 ISR 的事）。 */
/* ★ V1_0_41：触摸脚"被拉死"防护。
 * 现象：某台设备的触摸输入脚被外部持续拉低（例如触摸控制器未供电时经 ESD 二极管把线拉低），
 *       而触摸判据是"低 = 触摸"，于是被当成"一直被触摸"，按键状态机无限连发
 *       （实测约 6 次/秒的 KEYx_Loog 风暴），设备永远无法入睡，表现为"卡死"。
 * 判据：真实的触摸总会松手 —— 因此某个触摸脚自上次为高以来持续为低超过
 *       TOUCH_STUCK_TICKS 拍（10ms/拍），就认定它被拉死，暂不参与按键判定；
 *       一旦它重新变高（触摸控制器恢复正常），计数清零、触摸功能自动恢复。
 * 机械按键（PA8/9/10）不受影响，仍按原有逻辑判定。 */
/* ★ V1_0_54：阈值由 500（5.0s）降到 80（0.8s）。
   现场 [KEY] 诊断实测：B 机在"整机醒着、屏幕有电"的整段时间里（复位后 4~17 秒）
   PB14 / PC6 会**反复**被触摸控制器拉低（每次约 1 秒、间隔约 0.2~1.2 秒），
   机械键 PA8/9/10 全程为 0 —— 是触摸脚在制造假按键。
   原来"连续为低 5 秒"的判据要十几秒才积累够，期间页面已经跳了很多次；
   配合上面的"低 +1 / 高 -2"积分，阈值取 80 拍（0.8s）即可：
     · 假信号（低 1 秒的成串脉冲）——第一串就超过 80，立刻抑制；
     · 真实点按（0.1~0.3 秒）——积分最多 30，0.15 秒后即衰减归零，绝不误伤；
     · 阈值 80 < 长按门限 100（1s），所以连"一次假长按"都不会漏出来。
   已被抑制的脚，只要持续为高，积分每拍 -2，0.4 秒内就回到阈值以下、自动恢复。 */
#define TOUCH_STUCK_TICKS    80U    /* 0.8s：低电平积分超过此值即判为触摸脚异常 */
#define TOUCH_RELEASE_TICKS  50U    /* 0.5s：必须持续为高这么久才解除抑制（滤掉唤醒瞬间的毛刺） */
static uint16_t s_touch_hi_cnt[3] = {0U, 0U, 0U};   /* 三个触摸脚连续为高的拍数 */

/* ★ V1_0_53：触摸"积分"计数（见 key_raw_scan 里的说明）。
   低电平 +1、高电平 -2（减到 0 为止），不再是"连续低"。
   单位仍是 10ms 拍，阈值 TOUCH_STUCK_TICKS(500) 不变。 */
static uint16_t s_touch_low_cnt[3] = {0U, 0U, 0U};   /* PB14 / PB15 / PC6 的"低电平积分" */

/* ★ V1_0_53：上电 / 开机 / 息屏唤醒后的**触摸静默窗口**（10ms 拍）。
   现场实测（见 [KEY] 诊断行）：每次屏幕上电、触摸控制器初始化时，
   它会把 PB14 / PC6 拉低约 5 秒（机械键 PA8/9/10 全程为 0，没人按键），
   固件判据"低 = 触摸"，于是白白产生一串假按键 —— 页面自己乱跳、甚至误触发长按关机。
   因此在刚上电/唤醒的这段时间里**只认机械键、完全忽略触摸**。 */
static volatile uint16_t s_touch_mute_ticks = 0U;
#define TOUCH_MUTE_TICKS   300U   /* 3.0s */

/* ★★★ V1_0_55：触摸脚的**长期占空比**判定。
   ------------------------------------------------------------------
   现场反馈："用轻触键进入设置页后不操作，静置时页面自己在走。"
   根因还是触摸脚假信号：B 机的触摸控制器在无人触摸时把 PB14 / PC6
   反复拉低（低约 1 秒、恢复高 0.2~1.2 秒）。V1_0_54 的"低 +1 / 高 ?2"积分
   虽然快，但**只要出现一次够长的高电平，积分就会跌回阈值以下**，
   触摸又被重新放行 —— 于是每隔十几秒就漏出一次假按键，页面就"走一格"。
   这里再加一层**慢速占空比**判据：
     · 每秒统计该脚"低电平拍数是否过半"，压入一个 10 位的历史移位寄存器；
     · 最近 10 秒里若 ≥4 秒以低为主 —— 真实触摸绝不可能这样，判为异常并
       **持续抑制该脚**（不是"抑制一会儿就又放行"）；
     · 降到 ≤2 秒才解除（带迟滞，避免在阈值附近抖动）。
   真实使用（点按 0.1~0.3 秒、即使 10 秒内连点十几次）占空比约 20%，
   远低于 40% 的门限，**不会误伤**。 */
#define TOUCH_DUTY_WIN     10U    /* 窗口：10 秒 */
#define TOUCH_DUTY_FAULT    4U    /* 10 秒里 ≥4 秒为低 -> 判异常 */
#define TOUCH_DUTY_RECOVER  2U    /* 降到 ≤2 秒 -> 解除 */
static uint16_t s_touch_sec_low[3] = {0U, 0U, 0U};   /* 本秒内的低电平拍数 */
static uint16_t s_touch_hist[3]    = {0U, 0U, 0U};   /* 最近 10 秒的低电平历史 */
static uint16_t s_touch_sec_tick   = 0U;             /* 秒计数（10ms/拍） */
static uint8_t  s_touch_fault[3]   = {0U, 0U, 0U};   /* 1 = 该脚被长期抑制 */

/* ★★★ V1_0_55：触摸控制器"还没上电"的判据。
   现场 [KEY] 诊断实测（B 机，复位后 4~8 秒）：
       [KEY] PA8=0 PA9=0 PA10=0 | PB14=0 PB15=0 PC6=0 | low=13,13,13
   三个触摸脚**同时**为低，随后又同时恢复高。人不可能用一根手指同时按三个键 ——
   这是"触摸控制器尚未供电"的典型特征：未上电的 IC 其 I/O 会经内部 ESD 二极管
   把线箝到接近 0V，读出来就是低。这段时间里固件把它当成了连续触摸，
   于是页面自己一格一格地走。
   判据：三脚同时为低持续 ≥ 0.2 秒，就认为控制器没就绪，**整体静默触摸**
   （只认机械键），并持续续期；等三脚不再同时为低，再过 1 秒自动恢复。
   正常使用（手指按其中一个键）另外两个脚是高电平，不会触发这条。 */
static uint16_t s_touch_all_low_cnt = 0U;
/* 触发拍数取 **2 拍（20ms）**：按键去抖本身就是"连续 2 拍一致才认"，
   所以必须在这个"锁存点"之前就把触摸关掉 —— 取 20 拍（0.2s）太晚，
   那时这次触摸已经被记成一次按下了（实测仍然漏出 KEY1_Short）。 */
#define TOUCH_ALLLLOW_TRIG    2U   /* 任意两脚同时为低 2 拍即认定控制器未就绪 */
#define TOUCH_ALLLLOW_HOLD  100U   /* 未就绪消失后再静默 1.0s */

/* 统计 10 位历史里"低"的秒数 */
static uint8_t touch_duty_count(uint16_t h)
{
    uint8_t n = 0U, i;
    for (i = 0U; i < TOUCH_DUTY_WIN; i++) { if ((h & (uint16_t)(1U << i)) != 0U) { n++; } }
    return n;
}

/* 触摸脚是否允许参与按键判定（1 = 允许） */
static uint8_t touch_pin_ok(uint8_t idx)
{
    if (s_touch_fault[idx] != 0U) { return 0U; }
    if (s_touch_mute_ticks != 0U) { return 0U; }
    if (s_touch_low_cnt[idx] >= TOUCH_STUCK_TICKS) { return 0U; }
    return 1U;
}

void key_touch_mute(uint16_t ticks)
{
    s_touch_mute_ticks = ticks;
}

/* ★ V1_0_53：诊断用 —— 每秒打印一次 6 个按键脚的**实时电平**和触摸"卡住"防护的两个计数器。
   只读、不阻塞、不改变任何状态；生产构建里由 main.c 的 Time_1s() 在 TEST_KEY_SRC==1 时调用。
   行首含义：
     PA8/PA9/PA10 = K1/K2/K3 机械键（高有效）
     PB14/PB15/PC6 = 触摸 K1/K2/K3（低有效）
     low/hi        = s_touch_low_cnt / s_touch_hi_cnt（连续低 / 连续高 的 10ms 拍数）
   判读：若"没碰按键"时某个 PB/PC 脚一直为 0、且 low 停在 499 以下来回，就是它在制造假按键。 */
void key_diag_dump(void)
{
    printf("[KEY] PA8=%u PA9=%u PA10=%u | PB14=%u PB15=%u PC6=%u | low=%u,%u,%u hi=%u,%u,%u\r\n",
           (unsigned)(KEY1 == KEY_ON),
           (unsigned)(KEY2 == KEY_ON),
           (unsigned)(KEY3 == KEY_ON),
           (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14),
           (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15),
           (unsigned)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6),
           (unsigned)s_touch_low_cnt[0], (unsigned)s_touch_low_cnt[1], (unsigned)s_touch_low_cnt[2],
           (unsigned)s_touch_hi_cnt[0],  (unsigned)s_touch_hi_cnt[1],  (unsigned)s_touch_hi_cnt[2]);
    printf("[KEY] duty=%u,%u,%u fault=%u,%u,%u alllow=%u mute=%u\r\n",
           (unsigned)touch_duty_count(s_touch_hist[0]),
           (unsigned)touch_duty_count(s_touch_hist[1]),
           (unsigned)touch_duty_count(s_touch_hist[2]),
           (unsigned)s_touch_fault[0], (unsigned)s_touch_fault[1], (unsigned)s_touch_fault[2],
           (unsigned)s_touch_all_low_cnt, (unsigned)s_touch_mute_ticks);
}

static uint8_t key_raw_scan(void)
{
    /* 每 10ms 一拍统计三个触摸脚的连续低电平拍数 */
    /* ★ V1_0_42：解除抑制要求"持续为高"，而不是"高一个采样" ——
       否则唤醒瞬间触摸控制器上电/复位造成的一次短暂高电平就会把计数清零，
       于是每个周期都要重新经历一次 5 秒宽限（实测每 60 秒一轮就复发一串假按键）。 */
    /* ★★★ V1_0_53：由"连续为低 500 拍"改成"低 +1 / 高 -2 的积分"。
       ------------------------------------------------------------------
       现场 [KEY] 诊断实测（B 机，机械键全程 PA8/PA9/PA10 = 0）：
           15:15:45.871  PB14=0 PC6=0        low=102,0,102
           15:15:49.837  PB14=0 PC6=0        low=501,0,501   ← 刚过阈值
           15:15:50.836  PB14=1(恢复高 0.5s)  low=0,0,601     ← 前一个被清零，防护失效
       即"低约 5 秒 → 恢复高 0.5 秒 → 再低"这种模式，会把"连续低"判据
       反复清零，防护永远停在临界点上，假按键源源不断。
       改成积分后：
         · 假信号（长期低、偶尔高一下）——低 5 秒就能累积过 500，稳定抑制；
         · 真实的点按/长按（低一会儿就松开）——每拍 +1、松开每拍 -2，
           积分很快归零，永远不会误判"卡住"；
         · 已被抑制的脚，积分降到 500 以下即自动恢复。 */
    if (Touch_KEY1 == 0U) { if (s_touch_low_cnt[0] < 0xFFFEU) { s_touch_low_cnt[0]++; } s_touch_hi_cnt[0] = 0U;
                            if (s_touch_sec_low[0] < 0xFFFEU) { s_touch_sec_low[0]++; } }
    else { if (s_touch_hi_cnt[0] < 0xFFFEU) { s_touch_hi_cnt[0]++; }
           s_touch_low_cnt[0] = (s_touch_low_cnt[0] >= 2U) ? (uint16_t)(s_touch_low_cnt[0] - 2U) : 0U; }
    if (Touch_KEY2 == 0U) { if (s_touch_low_cnt[1] < 0xFFFEU) { s_touch_low_cnt[1]++; } s_touch_hi_cnt[1] = 0U;
                            if (s_touch_sec_low[1] < 0xFFFEU) { s_touch_sec_low[1]++; } }
    else { if (s_touch_hi_cnt[1] < 0xFFFEU) { s_touch_hi_cnt[1]++; }
           s_touch_low_cnt[1] = (s_touch_low_cnt[1] >= 2U) ? (uint16_t)(s_touch_low_cnt[1] - 2U) : 0U; }
    if (Touch_KEY3 == 0U) { if (s_touch_low_cnt[2] < 0xFFFEU) { s_touch_low_cnt[2]++; } s_touch_hi_cnt[2] = 0U;
                            if (s_touch_sec_low[2] < 0xFFFEU) { s_touch_sec_low[2]++; } }
    else { if (s_touch_hi_cnt[2] < 0xFFFEU) { s_touch_hi_cnt[2]++; }
           s_touch_low_cnt[2] = (s_touch_low_cnt[2] >= 2U) ? (uint16_t)(s_touch_low_cnt[2] - 2U) : 0U; }

    /* ★★★ V1_0_55：**任意两个及以上**触摸脚同时为低 => 触摸控制器未就绪 => 整体静默触摸。
       为什么不是"三个都低"：现场还出现过"PB14 与 PC6 同时低、PB15 正常"的模式；
       为什么阈值取 2 拍：按键去抖是"连续 2 拍一致才认"，必须在锁存之前关掉，
       取 0.2 秒太晚（实测那时这次触摸已经被记成一次按下了）。
       正常使用：手指按其中一个键时，另外两个脚是高电平，不会触发本条。 */
    {
        uint8_t low_n = 0U;
        if (Touch_KEY1 == 0U) { low_n++; }
        if (Touch_KEY2 == 0U) { low_n++; }
        if (Touch_KEY3 == 0U) { low_n++; }
        if (low_n >= 2U)
        {
            if (s_touch_all_low_cnt < 0xFFFFU) { s_touch_all_low_cnt++; }
            if (s_touch_all_low_cnt >= TOUCH_ALLLLOW_TRIG) { s_touch_mute_ticks = TOUCH_ALLLLOW_HOLD; }
        }
        else
        {
            s_touch_all_low_cnt = 0U;
        }
    }

    /* ★ V1_0_55：每满 1 秒做一次占空比判定（带迟滞） */
    s_touch_sec_tick++;
    if (s_touch_sec_tick >= 100U)
    {
        uint8_t k;
        s_touch_sec_tick = 0U;
        for (k = 0U; k < 3U; k++)
        {
            uint8_t n;
            s_touch_hist[k] = (uint16_t)(((s_touch_hist[k] << 1) |
                               ((s_touch_sec_low[k] >= 50U) ? 1U : 0U)) & 0x03FFU);
            s_touch_sec_low[k] = 0U;
            n = touch_duty_count(s_touch_hist[k]);
            if (s_touch_fault[k] == 0U) { if (n >= TOUCH_DUTY_FAULT)   { s_touch_fault[k] = 1U; } }
            else                        { if (n <= TOUCH_DUTY_RECOVER) { s_touch_fault[k] = 0U; } }
        }
    }

    /* 上电/唤醒后的静默窗口内：只认机械键，触摸一律不算 */
    {
        /* ★ V1_0_55：触摸是否放行统一交给 touch_pin_ok() —— 它同时考虑
           ①长期占空比抑制 ②上电/唤醒静默窗 ③短期低电平积分；
           配合 TOUCH_INPUT_DISABLE（整机禁用触摸，给控制器损坏的机器用）。 */
#if TOUCH_INPUT_DISABLE
        if     (KEY1==KEY_ON) { return KEY1_PRES; }
        else if(KEY2==KEY_ON) { return KEY2_PRES; }
        else if(KEY3==KEY_ON) { return KEY3_PRES; }
#else
        if     ((KEY1==KEY_ON) || (touch_pin_ok(0U) != 0U && Touch_KEY1==0)) { return KEY1_PRES; }
        else if((KEY2==KEY_ON) || (touch_pin_ok(1U) != 0U && Touch_KEY2==0)) { return KEY2_PRES; }
        else if((KEY3==KEY_ON) || (touch_pin_ok(2U) != 0U && Touch_KEY3==0)) { return KEY3_PRES; }
#endif
    }
    return 0U;
}

/* ---- 10ms 定时器中断上下文维护的按键状态（volatile：中断写、主循环读） ---- */
#define KEY_DEBOUNCE_TICKS   2U      /* 连续 2 拍（10ms/拍 = 20ms）一致才认，滤毫秒级抖动 */
static volatile uint8_t  s_isr_key_now    = 0U;   /* 去抖后"当前按住的键"（0=无） */
static volatile uint8_t  s_isr_key_sticky = 0U;   /* 粘滞：按下后保持到主循环取走（单槽） */
static volatile uint16_t s_isr_hold_ticks = 0U;   /* 当前这次按住的拍数（10ms/拍，长按计时） */
static volatile uint8_t  s_isr_raw_prev = 0xFFU;          /* 上一拍的原始扫描值（0xFF = 未初始化） */
static volatile uint8_t  s_isr_same_cnt = 0U;             /* 原始值连续相同的拍数 */

/* ==========================================================================
 * key_tick_isr ：**10ms 定时器中断里调用**（soft_timer_create(...,10,CB_IRQ)）
 * --------------------------------------------------------------------------
 * 约束：只允许 GPIO 读 + 整数运算；不许 printf / 不许阻塞 / 不许发帧。
 * 语义：
 *   · 原始扫描值连续 KEY_DEBOUNCE_TICKS 拍一致才认（去抖 20ms）；
 *   · 认到"有键按下"-> 更新 s_isr_key_now，按下的那一次把键号写进粘滞槽，
 *     并把按住拍数清零重新计；
 *   · 认到"抬起"-> s_isr_key_now = 0（粘滞槽**不清**，等主循环取）；
 *   · 按住期间累加 s_isr_hold_ticks（上限 60000 拍 = 10 分钟，防溢出）。
 * ========================================================================== */
void key_tick_isr(void)
{
    if (s_touch_mute_ticks != 0U) { s_touch_mute_ticks--; }   /* ★ V1_0_53：静默窗口倒数 */
    uint8_t raw = key_raw_scan();

    if(raw == s_isr_raw_prev)
    {
        if(s_isr_same_cnt < 250U) { s_isr_same_cnt++; }
    }
    else
    {
        s_isr_raw_prev = raw;
        s_isr_same_cnt = 1U;
    }

    if(s_isr_same_cnt >= KEY_DEBOUNCE_TICKS)      /* 连续 2 拍一致：认这一次电平变化 */
    {
        if(s_isr_key_now != raw)
        {
            s_isr_key_now = raw;
            if(raw != 0U)
            {
                s_isr_key_sticky = raw;          /* ★ 按下沿：写粘滞槽（不累积） */
                s_isr_hold_ticks = 0U;           /* ★ 新的一次按下：时长从 0 起 */
            }
        }
        if(s_isr_key_now != 0U)
        {
            if(s_isr_hold_ticks < 60000U) { s_isr_hold_ticks++; }
        }
    }
}

/* ==========================================================================
 * key_level_sticky ：主循环每拍取一次（取代原来的 key_scan(1)）
 * --------------------------------------------------------------------------
 * 返回非 0 = "这一拍该键是按下状态"，返回 0 = "抬起"。规则：
 *   · 键还按着        -> 每次返回该键号，并把粘滞槽清掉（按下事件已被消费）；
 *   · 已经抬起        -> 若粘滞槽还有没消费的按下，**补返回一次**该键号；
 *                       下一次调用就返回 0 -> 状态机照原样走"松手那一拍派发"。
 * 这样"按下-抬起都发生在主循环被卡住期间"的那一按，也能被完整地补上一次短按。
 * ========================================================================== */
static uint8_t key_level_sticky(void)
{
    uint8_t k = s_isr_key_now;

    if(k != 0U)
    {
        s_isr_key_sticky = 0U;      /* 电平已在：这一拍的按下事件不用再补 */
        return k;
    }

    k = s_isr_key_sticky;           /* 已经抬起：把没消费掉的"按下"补一次 */
    s_isr_key_sticky = 0U;
    return k;
}

/* 主循环用的"按住拍数"（10ms/拍）：给长按判定做**不依赖主循环速率**的判据 */
static uint32_t key_hold_ticks(void)
{
    return (uint32_t)s_isr_hold_ticks;
}
/* ★ V1_0_6：按键"边沿事件"队列（key_evt_push / key_evt_pop / key_evt_flush +
   s_key_evt[] / s_key_evt_head / s_key_evt_tail / g_key_evt_drop_cnt /
   KEY_EVT_LEN / KEY_EVT_DEBOUNCE_TICKS）已在这里**整体删除** —— 见文件顶部"按键策略"。 */

static uint8_t old_key_flag=0;
uint8_t  key_value=0;

static  uint8_t old_key_value =0;
#define Key_scan_time   100   /* ★ V1_0_6：10ms 时基下 100 拍 = 1s（原 20ms 时基下是 50 拍）*/

/* ==========================================================================
 * ★★ V1_0_2 修正：长短按判定改成"按真实时间"，不再数 Key_task 被调用的次数
 * --------------------------------------------------------------------------
 * 旧实现用 `num_on`（Key_task 被调用的次数）当时间：Key_scan_time = 50，
 * 只有在 Key_task **每 20ms 恰好跑一次**时 "num_on > 50" 才等于"按住 > 1s"。
 * 但 Key_task 是软定时器回调，只在主循环的 soft_timer_loop_async() 里执行，
 * 而主循环一拍的实际长度被 LoRa 的 STOP/CAD、ADC、printf 等拖成
 * 几十~几百毫秒都正常（见 LoraPro.c 的 CAD_work() / 驱动里的等待循环）。
 * 于是：主循环越慢，num_on 涨得越慢 —— 手指按住 2 秒，num_on 可能还不到 50，
 * 松手时被判成"短按" => **长按（快速调数值）时灵时不灵**；长按连发的
 * 间隔（原来是 num_on2*10>100，即 11 次回调）也跟着主循环速率漂移。
 * 现在：用一个**按真实时间**的按住时长（Rt_tick，100ms 节拍，STOP 下仍在走）
 * 来判定，回调次数只在主循环正常时才与时间等价。
 *   · 长按门限 = KEY_LONG_PRESS_TICKS(10) × 100ms = 1s（与原来 50×20ms 等效）
 *   · 连发间隔 = KEY_REPEAT_TICKS(2) × 100ms = 200ms（接近原来 11×20ms≈220ms）
 * 兼容性：Rt_tick 万一不走，held 恒为 0，新增的两个条件都不成立 =>
 * 行为与改动前完全一致，绝不会因为这次修改而按不出按键。
 * ========================================================================== */
#define KEY_LONG_PRESS_TICKS   10U   /* 按住多久算长按：10 × 100ms = 1s */
#define KEY_REPEAT_TICKS        2U   /* 长按连发间隔：2 × 100ms = 200ms */

static volatile uint32_t s_key_down_tick = 0U;   /* 本次按下的起始时刻（Rt_tick） */
static uint8_t  s_long_fired    = 0U;   /* 本次按下是否已经按"长按"处理过 */
/* ★ 注意类型：必须 uint32_t。Rt_tick 是无符号 32 位且一直在涨，
   存成 uint8_t 会被截断，导致 (Rt_tick - key_repeat_tick) 恒为巨大值，
   长按连发会退化成"每次 Key_task 都触发"。 */
static volatile uint32_t key_repeat_tick = 0U;   /* 长按连发的计时（Rt_tick） */

/* 当前这一次按键已经按住多久（100ms 为单位的节拍数）；没按住时返回 0 */
static uint32_t key_held_ticks(void)
{
    if (num_on == 0)
    {
        return 0U;
    }
    return (uint32_t)(Rt_tick - s_key_down_tick);
}

/* 这一次按下算不算"长按"：优先按**真实按住时间**；Rt_tick 万一不走
   （held 恒为 0）就退回原来的"按回调次数"判据，行为始终可预测。 */
static uint8_t key_is_long_press(void)
{
    uint32_t held = key_held_ticks();

    /* ★ V1_0_7：优先用 10ms 定时器中断里量到的按住拍数 —— 它与主循环快慢无关，
       100 拍 = 1s，手指按住多久就量到多久，不会因为主循环卡顿而把长按判成短按。 */
    if(key_hold_ticks() >= ((uint32_t)KEY_LONG_PRESS_TICKS * 10U))
    {
        return 1U;
    }

    if (held != 0U)
    {
        return (held >= KEY_LONG_PRESS_TICKS) ? 1U : 0U;
    }
    return (num_on > Key_scan_time) ? 1U : 0U;
}

uint8_t num_on2=0;

/* 主循环 / 唤醒路径调用：**复位按键状态机**。
   历史名 key_evt_reset 已改名 —— 因为"边沿事件队列"整体移除后，这个函数不再
   与事件队列有关，只负责把长短按的计时与防重复触发窗口清零。
   ★ 必须在**唤醒后**调用：否则第一次按键会拿着息屏前的时间戳，被判成
   "已经按住很久"从而直接触发长按动作；也会把息屏前那次按键的防抖窗口带过来。 */
/* V1_0_59：开机瞬间"手还没松"的闸门。
   用户是长按 K1+K3 满 2 秒才开机的，开机后手通常还按着；不拦的话 Key_task
   会把"开机那一按"当成一次新的按键，按住超过 1 秒就触发长按动作。
   闸门在有键按着期间**吞掉全部按键**（不派发任何业务），松手即解除；
   另设 3 秒上限，防止某个脚被外部拉死导致按键永久失效。 */
static uint8_t  s_wait_release       = 0u;
static uint16_t s_wait_release_ticks = 0u;

void key_wait_release(void)
{
    s_wait_release       = 1u;
    s_wait_release_ticks = 300u;     /* 10ms 拍 -> 3s 上限 */
    key_state_reset();               /* 把"开机那一按"的计时全部清掉 */
}

void key_state_reset(void)
{
    num_on          = 0;
    num_on2         = 0;
    s_key_down_tick = 0U;
    s_long_fired    = 0U;
    key_repeat_tick = 0U;
    /* ★ V1_0_7：中断侧的按键状态也一并复位，否则唤醒后会把"用来唤醒的那一下"
       的粘滞值 / 按住拍数带进来，凭空多触发一次动作。 */
    s_isr_key_now    = 0U;
    s_isr_key_sticky = 0U;
    s_isr_hold_ticks = 0U;
    s_isr_raw_prev   = 0xFFU;
    s_isr_same_cnt   = 0U;
    /* ★ V1_0_42：**不清零** s_touch_low_cnt。
       触摸脚"被拉死"是持续性的硬件状态，不是一次按键事件；若在这里清零，
       每次从息屏唤醒都会重新获得 5 秒的"宽限期"，于是每个读取周期都要白醒
       约 5 秒并刷一串假按键（实测每 60 秒一轮就会出现 20~25 次）。
       保留计数后：只在冷启动时有一次 5 秒宽限，之后一直保持抑制；
       该脚一旦恢复高电平，计数会自动清零、触摸功能自动恢复。 */
}

extern ALL_UI UI;
extern CRfNet RfNet;
extern uint16_t sleep_cnt;
uint8_t save_data_flash_flag=0;

uint8_t key_scan(uint8_t mode)
{
  static uint8_t key_up =1;
  if(mode)
  {
      key_up=1;
  }
  if(key_up&&(KEY1==KEY_ON||KEY2==KEY_ON||KEY3==KEY_ON||Touch_KEY1==0||Touch_KEY2==0||Touch_KEY3==0))
  {
      key_up=0;
      if(KEY1==KEY_ON||Touch_KEY1==0)
      {
          return KEY1_PRES;
      }
      else if (KEY2 == KEY_ON||Touch_KEY2==0)
      {
          return KEY2_PRES;
      }
      else if (KEY3 == KEY_ON||Touch_KEY3==0)
      {
          return KEY3_PRES;
      }
  }
  else if(KEY1==KEY_OFF&&KEY2==KEY_OFF&&KEY3==KEY_OFF&&Touch_KEY1==1&&Touch_KEY2==1&&Touch_KEY3==1)
  {
      key_up=1;
  }
  return 0;
}

/* ==========================================================================
 * 第 4 层 UI：二级子页（冻结契约第 1 / 2.3 节）
 * --------------------------------------------------------------------------
 * UI.Sub2 作为 0x01 帧的 re_flag 发出，用来选择由 CH584M 绘制的页面
 * （所有绘制都在显示屏一侧本地完成）：
 *   Sub2_Unbind  1 ：第 0 项 = 解绑全部（cmd 0x04），第 1 项 = 返回
 *   Sub2_BindDev 2 ：第 0 项 = 保存当前设备（cmd 0x05），第 1 项 = 返回
 *   Sub2_Signal  3 ：纯显示页，单页全屏 20 条，K1/K3 无操作，K2 = 返回
 *   Sub2_Volt    4 ：纯显示页，单页全屏 20 条，K1/K3 无操作，K2 = 返回
 *   Sub2_BindNam 5 ：纯显示页，单页全屏 20 条，K1/K3 无操作，K2 = 返回
 *   Sub2_Factory 6 ：第 0 项 = 确认（cmd 0x07），第 1 项 = 返回
 * UI.Addr 在项目页里被复用为项目索引（它作为 rank3_addr 发出，
 * 所以 CH584M 的高亮框能跟着走），UI.SubPage 已作废（恒 0）：这三张纯显示页改成单页全屏后，chu_num2 不再当页码（冻结契约 §4.2）。
 * 当 UI.Sub2 != 0 时，下面的 rank3 处理函数绝不能运行：
 * 这正是 0x04/0x05/0x07 只发一次、以及二级子页在屏时设置值不会被改动的
 * 保证。
 * ========================================================================== */

/* 退出二级子页；setting_flag = 1 菜单列表 / 2 rank3 列表 */
static void sub2_close(uint8_t setting_flag, Set_addr addr)
{
    UI.Sub2          = Sub2_None;
    UI.SubPage       = 0;
    UI.Addr          = addr;
    UI.Setting_flag  = setting_flag;
    UI.dis_data_flag = 1;
}

/* 二级子页里的 K1 / K3 */
static void sub2_nav(void)
{
    if ((UI.Sub2 == Sub2_Signal) || (UI.Sub2 == Sub2_Volt) ||
        (UI.Sub2 == Sub2_BindNam))
    {
        /* 冻结契约（协议_息屏省电数据链与页面体系_冻结.md §4.2）：
           这三页在 CH584M 侧已经改成「单页全屏 20 条」，没有第二页了，
           所以 K1/K3 不再翻页 —— 保持无操作，也不动任何字段。
           退出统一走 K2 的「返回」（见 sub2_select()）。 */
        UI.dis_data_flag = 1;
        return;
    }
    UI.Addr = (UI.Addr == Set_Addr1) ? Set_Addr2 : Set_Addr1;
    UI.dis_data_flag = 1;
}

/* 二级子页里的 K2：激活当前高亮项 */
static void sub2_select(void)
{
    switch (UI.Sub2)
    {
        case Sub2_Unbind:                          /* 解绑全部 / 返回    */
            if (UI.Addr == Set_Addr1)
            {
                send_unbind_all();                 /* 复用已有的 cmd 0x04 发送实现 */
                DisOpt = Opt2;                     /* 菜单列表：设备绑定 */
                sub2_close(1, Set_Addr1);          /* 然后回到菜单       */
            }
            else
            {
                sub2_close(2, Set_Addr2);          /* 设备绑定 / 一键解绑  */
            }
            break;

        case Sub2_BindDev:                         /* 保存设备 / 返回      */
            if (UI.Addr == Set_Addr1)
            {
                /* ★ V1_0_7（P2）：保存当前设备之后**返回上一级**。
                   上一级就是"设备绑定"的 rank3 列表：Setting_flag=2 / DisOpt=Opt2 /
                   Addr=Set_Addr3（= 索引 2「绑定设备」，也就是刚才进来的那一项），
                   这样屏上的高亮框正好停回原处。
                   原来这里只 send_save_binding() 就停在本页，用户按了保存看不到返回，
                   还得再按一次「返回」。
                   ★ CH584M 侧的"绑定保存成功/失败"提示是**覆盖层**（g_ui_msg，
                     不是页面状态，见 Usart3_task.c 的 menu_rank==6 说明），所以本函数
                     发出的 0x01 帧不会顶掉提示：提示显示完 UI_MSG_HOLD_SEC 秒后，
                     屏上留下的正是这一帧的页面（= 上一级列表）。 */
                send_save_binding();               /* cmd 0x05：让 CH584M 存绑定        */
                DisOpt = Opt2;                     /* 确保停在「设备绑定」这一页        */
                sub2_close(2, Set_Addr3);          /* 回上一级（高亮 = 2 绑定设备）     */
            }
            else
            {
                sub2_close(2, Set_Addr3);          /* 设备绑定 / 绑定设备  */
            }
            break;

        /* 冻结契约 §4.2 / §4.4：设备信号 / 设备电压 / 绑定名称 都是纯显示页
           （单页全屏 20 条），CH584M 在页面上画了「返回」项，K2（确认）
           落在它上面就是"回上一级"：sub2_close() 会把 UI.Sub2 清成
           Sub2_None（下一帧的 re_flag = 0）并回到进入这三页的那一级
           rank3 列表 —— 沿用原有的 sub2_close() 路径，没有新增语义。 */
        case Sub2_Signal:                          /* K2 = 返回 -> 安装调试 */
            sub2_close(2, Set_Addr1);
            break;

        case Sub2_Volt:                            /* K2 = 返回 -> 安装调试 */
            sub2_close(2, Set_Addr2);
            break;

        case Sub2_BindNam:                         /* K2 = 返回 -> 信息汇总 */
            sub2_close(2, Set_Addr1);
            break;

        case Sub2_Factory:                         /* 确认 / 返回          */
            if (UI.Addr == Set_Addr1)
            {
                factory_reset();                   /* 本机恢复默认值       */
                send_factory_reset();              /* cmd 0x07 -> CH584M   */
                UI.SubPage       = 0;
                UI.Sub2          = Sub2_None;
                UI.Addr          = Set_Addr1;
                DisOpt           = Opt0;
                UI.Setting_flag  = 0;               /* 然后回到主页         */
                UI.dis_data_flag = 1;
            }
            else
            {
                sub2_close(1, Set_Addr1);           /* 回到菜单列表         */
            }
            break;

        default:
            sub2_close(1, Set_Addr1);
            break;
    }
}

void Key_task(void)
{
   /* V1_0_59：开机后等手松开 —— 闸门期间吞掉全部按键（见 key_wait_release 的说明）。 */
   if (s_wait_release != 0u)
   {
       if (key_level_sticky() == 0u)
       {
           s_wait_release = 0u;
       }
       else if (s_wait_release_ticks != 0u)
       {
           s_wait_release_ticks--;
           if (s_wait_release_ticks == 0u) { s_wait_release = 0u; }   /* 兜底：3s 后强制解除 */
       }
       key_value    = 0U;
       old_key_flag = 0U;
       num_on       = 0;
       return;
   }

   /* ★ V1_0_7：按键的采样 / 去抖 / 按住时长全部由 10ms 定时器中断 key_tick_isr()
      维护，这里只"取一次粘滞电平"（见 key_level_sticky() 的说明）：
        · 键还按着        -> 返回键号（连续多拍，供状态机累时长 / 判长按）
        · 已抬起但没取过  -> 补返回一次键号（这一拍当作"还按着"），下一拍返回 0，
                             于是走下面原有的短按派发 —— **卡住期间的那一按也不会丢**。
      原来 V1_0_6 的"防重复触发窗口"（8 拍）已删除：它按采样拍数计，主循环一慢
      就会吞掉用户后一次正常按键；现在抗抖动由中断里的 20ms 真实时间去抖负责。 */
#if TEST_MODE
     key_value = 0U;   /* TEST: 忽略按键(规避浮空连发干扰睡眠读测试) */
#else
     key_value = key_level_sticky();
#endif

   if(key_value==0)
   {
     old_key_flag=0;
   }
      if(key_value!=0&&num_on==0)
   {
       old_key_value=key_value;
       num_on=1;
       s_key_down_tick = Rt_tick;   /* ★ 记录按下时刻，供 key_held_ticks() 用 */
       s_long_fired    = 0U;        /* ★ 新的这一次按下：还没按长按处理过 */
   }
   /* ★ V1_0_6：num_on 上限随 10ms 时基从 300 改成 600（都等于 6s 封顶） */
   if(key_value!=0&&num_on>=1&&num_on<=600)
   {
       num_on++;
		   sleep_cnt=0;
   }
   /* ★★★ V1_0_11：只要还有键按着，就一直清零息屏计时。
      原来 sleep_cnt=0 写在 if(key_value!=0 && num_on>=1 && num_on<=600) 里，
      num_on 到 600（6 秒）就不再自增 -> 条件失效 -> **长按住不放超过 6 秒，
      息屏计时又开始走**，主页上（Setup.brightness=5）会在手还按着的时候就
      息屏。长按处理用的是 Rt_tick（s_key_down_tick），与 num_on 无关，
      所以这里单独补一条无条件的清零，绝不改长按判据。 */
   if(key_value!=0)
   {
       sleep_cnt=0;
   }
   /* ★ 短按 = "松手这一拍，而且这一次按下**没有**被当作长按处理过"。
      用 s_long_fired 而不是再判一次时间，是为了覆盖一种退化场景：
      主循环慢到"整次按下（连按带放）都落在同一拍里"时，num_on 还很小时
      时间已经够长了 —— 那种情况必须退化成**短按**（让用户至少得到一次响应），
      绝不能因为"时间够长"就走了长按分支却在 key_value==0 时什么都不做。 */
   if(key_value==0&&num_on>0&&s_long_fired==0U)
   {
       switch(old_key_value)
       {
          case KEY1_PRES:
             switch(UI.Setting_flag)
						 {
							 case 0:   /* 主页：K1/K3 翻 20 通道列表
							              （通道 1..10 / 11..20，0x01 帧的 chu_num1） */
								 UI.MainPage = (UI.MainPage == 2) ? 1 : 2;
								 UI.dis_data_flag = 1;
								 break;
							 case 1:
								  /* ★ V1_0_7：菜单级也要显式置刷新标志（菜单光标 DisOpt 虽然会被
								     UI_display() 的影子变量发现，但显式置一次更稳，避免漏帧） */
								  UI.dis_data_flag = 1;
								  UI.Sub2 = Sub2_None;   /* 退出子页 */
								  switch(DisOpt)
									{
										case Opt7:
											DisOpt  = Opt0;
											break;
										default:
											DisOpt ++;
                                            /* 关机状态跳过组网测试菜单。 */
                                            if (Setup.On_Off_flag == 0u && DisOpt == Opt1)
                                            {
                                                DisOpt = Opt2;
                                            }
											break;
									}
								 break;
							 case 2:
									UI.dis_data_flag=1;
									if(UI.Sub2 != Sub2_None)
									{
										sub2_nav();          /* 二级子页：切换项目（三张纯显示页里 sub2_nav() 无操作） */
										break;
									}
									switch(DisOpt)
									{
										case Opt0:
											switch(UI.Addr)
											{
												case 0:
													UI.Addr=2;
													break;
											  case 1:
														backups_Setup.partition ++ ;
														if(backups_Setup.partition>64)
														{
															backups_Setup.partition=0;
														}
													break;
												case 2:
													UI.Addr=4;
													break;
												case 3:
													if(UI.dis_data_flag==1)
													{
															backups_Setup.Host_number++;
															if(backups_Setup.Host_number>121)
															{
																backups_Setup.Host_number=0;
															}
													}
													break;
												case 4:
													UI.Addr=5;
													break;
												case 5:
													UI.Addr=0;
													break;
												default:
													break;
											}
											break;
										case Opt1:
											/* ★ V1_0_7（P1）：删掉「一键解绑(2) / 保存绑定(3)」两个**残留项**。
											   CH584M 的组网页现在只画「开始组网(0) / 返回(1)」两项
											   （yuying_TFT.c 的 networking_Control1()，注释明确写了
											     obsolete STM32 focus IDs 2/3 不得再映射到任何动作），
											   所以这里只允许 0 <-> 1 循环；并且只在选择视图里生效（RFNET_IDLE）：
											   组网信息视图在屏时，按键不得移动高亮，也不得发出任何 cmd。 */
											if(RfNet.Status==RFNET_IDLE || RfNet.Status==RFNET_DONE)
											{
												switch(UI.Addr)
												{
													case 0: /* 0 开始组网 */
														UI.Addr=Set_Addr2;   /* -> 1 返回 */
														break;
													case 1: /* 1 返回 */
														UI.Addr=Set_Addr1;   /* -> 0 开始组网 */
														break;
													default:
														UI.Addr=Set_Addr1;
														break;
												}
											}
											break;
										case Opt2: /* 设备绑定：0/1/2/3 */
											switch(UI.Addr)
											{
												case 0: UI.Addr=Set_Addr2; break;
												case 1: UI.Addr=Set_Addr3; break;
												case 2: UI.Addr=Set_Addr4; break;
												case 3: UI.Addr=Set_Addr1; break;
												default: UI.Addr=Set_Addr1; break;
											}
											break;
										case Opt3: /* 安装调试：0/1/2 */
											switch(UI.Addr)
											{
												case 0: UI.Addr=Set_Addr2; break;
												case 1: UI.Addr=Set_Addr3; break;
												case 2: UI.Addr=Set_Addr1; break;
												default: UI.Addr=Set_Addr1; break;
											}
											break;
										case Opt4:
												switch(UI.Addr)
								      	{
													case 0:
														backups_Setup.transmit_number[0]++;
													  if(backups_Setup.transmit_number[0]>122)
														{
															backups_Setup.transmit_number[0]=0;
														}
														break;
													case 1:
														backups_Setup.transmit_number[1]++;
													  if(backups_Setup.transmit_number[1]>122)
														{
															backups_Setup.transmit_number[1]=0;
														}
														break;
													case 2:
														backups_Setup.transmit_number[2]++;
													  if(backups_Setup.transmit_number[2]>122)
														{
															backups_Setup.transmit_number[2]=0;
														}
														break;
										     case 3:
													 UI.Addr=4;
													 break;
												 case 4:
													  UI.Addr=3;
													break;
												 default:
													 break;
											  }
											break;
										case Opt5:
												switch(UI.Addr)
												{
													case 0:
														UI.Addr=2;
														break;
													case 1:
														backups_Setup.transmitted_power++;
													  if(backups_Setup.transmitted_power>22)
														{
															backups_Setup.transmitted_power=-3;
														}
														break;
													case 2:
														UI.Addr=4;
														break;
													case 3:
														backups_Setup.brightness+=5;
													  if(backups_Setup.brightness>120)
														{
															backups_Setup.brightness=5;
														}
														break;
													case 4:
														UI.Addr=6;
														break;
													case 5:
														if(backups_Setup.On_Off_flag==0)
														{
															backups_Setup.On_Off_flag=1;
														}
														else{
															backups_Setup.On_Off_flag=0;
														}
														break;
													case 6:
														UI.Addr=7;
														break;
													case 7:
														UI.Addr=0;
														break;
													default:
													break;
												}
											break;
										case Opt6: /* 信息汇总：0/1/2/3/4 */
											switch(UI.Addr)
											{
												case 0: UI.Addr=Set_Addr2; break;
												case 1: UI.Addr=Set_Addr3; break;
												case 2: UI.Addr=Set_Addr4; break;
												case 3: UI.Addr=Set_Addr5; break;
												case 4: UI.Addr=Set_Addr1; break;
												default: UI.Addr=Set_Addr1; break;
											}
											break;
										case Opt7:
											break;
										default:
											break;
									}
									break;
						 }
             printf("KEY1_Short !\r\n");
          break;
          case KEY2_PRES: // set key
					switch(UI.Setting_flag)
					{
						case 0:
						  /* ---- V1_0_55：从主页进入设置页，一律回到最初的位置 -------------------
						     现场问题：在设置页（或某个二级子页）静置到息屏，唤醒后回到主页，再从
						     主页按 K2 进设置，屏幕直接停在【上次静置时的那一页】——左侧高亮项、右侧
						     内容都还是旧的。
						     原因：息屏超时那条路径只清了 Setting_flag / UI.Addr，UI.Sub2（二级子页）
						     与 DisOpt（左侧菜单高亮）被原样留在了那里。
						     这里是【从主页进入设置】的唯一必经之路，且按下即生效（不经过 1 秒计时器），
						     所以把复位放这里最稳也最快：无论残留在哪个状态位、由哪条路径留下，进来
						     就一定是最初的位置。 */
						  UI.Sub2          = Sub2_None;   /* 关掉上一次的二级子页 */
						  UI.Addr          = Set_Addr1;   /* 右侧内容回到第一项   */
						  DisOpt           = Opt0;        /* 左侧高亮回到第一项   */
						  UI.Setting_flag  = 1u;          /* 固定在一级菜单列表   */
						  UI.dis_data_flag = 1;           /* 立即重画一次         */
						  break;
						case 1:
                            /* 拦截残留组网焦点，关机时不允许进入子页。 */
                            if (Setup.On_Off_flag == 0u && DisOpt == Opt1)
                            {
                                DisOpt = Opt0;
                                UI.Addr = Set_Addr1;
                                UI.Sub2 = Sub2_None;
                                UI.dis_data_flag = 1u;
                                break;
                            }
								switch(DisOpt)
								{
										case Opt7:
						     	     UI.Setting_flag=0;
										   UI.dis_data_flag=1;
										   DisOpt=0;
										   UI.Sub2=Sub2_None;
										   UI.Addr=Set_Addr1;
											break;
										default:
											 UI.Setting_flag++;
									  	 UI.Addr=0;
									  	 UI.Sub2=Sub2_None;   /* 进入 rank3 页面 */
											switch(DisOpt)
											{
												 case Opt0:
												  	backups_Setup.partition    =	 Setup.partition;
										        backups_Setup.Host_number  =   Setup.Host_number;
													break;
													case Opt2:
														break;
													case Opt4:
														memcpy(backups_Setup.transmit_number,Setup.transmit_number,3);
														break;
													case Opt5:
													  backups_Setup.brightness =	Setup.brightness;
										        backups_Setup.transmitted_power = Setup.transmitted_power;
													  backups_Setup.On_Off_flag=Setup.On_Off_flag;
													  break;
													default:
														break;
											}
											break;
								}
							break;
						case 2:
							UI.dis_data_flag = 1;
							if(UI.Sub2 != Sub2_None)
							{
								sub2_select();          /* 二级子页：激活当前项 */
								break;
							}
							switch(DisOpt)
							{
								case Opt0:
									switch(UI.Addr)
									{
										case 0:
											UI.Addr++;
											break;
										case 1:
										  UI.Addr=0;
											break;
										case 2:
                     UI.Addr++;
											break;
										case 3:
                     UI.Addr=2;
											break;
										case 4: // save
										Setup.partition =	backups_Setup.partition;
										Setup.Host_number=  backups_Setup.Host_number;
										UI.Addr=0;
										UI.Setting_flag=1;
										GetNewsend_addr();  /* 台号/分站号保存：默认路由与新身份一起写入。 */
										Save_data();
										save_data_flash_flag=1;
										break;
										case 5: // no save
										UI.Addr=0;
										UI.Setting_flag=1;
											break;
										default:
											break;
									}
									break;
								case Opt1:
                                    /* 关机时即使残留在组网页，也不得启动测试。 */
                                    if (Setup.On_Off_flag == 0u)
                                    {
                                        UI.Setting_flag = 1u;
                                        DisOpt = Opt0;
                                        UI.Addr = Set_Addr1;
                                        UI.Sub2 = Sub2_None;
                                        break;
                                    }
									/* ★ V1_0_7（P1）：删掉「一键解绑 / 保存绑定」两个残留项（原因同 KEY1/K3）。
									   CH584M 的组网页只画「开始组网(0) / 返回(1)」，所以这里只保留 0 和 1：
									     0 开始组网 -> 开始测试（本地置状态，不发 cmd）
									     1 返回     -> 回菜单列表 */
									switch(UI.Addr)
									{
										case 0: /* 0 开始组网（结果页则为"重新组网"） */
										{
										    UI.Addr=Set_Addr1;   /* 焦点留在 0 */
										    /* ★★★ V1_0_51：无论是"开始组网"还是"重新组网"，都先把三个地址的计数
										       彻底清零，绝不把上一轮的结果带进新一轮（现场要求"拒绝残留"）。
										       注意：只有"选择态"和"结果页"才响应按键；组网进行中一律忽略，
										       避免手一抖就把正在跑的测试重启或退出。 */
										    if(RfNet.Status==RFNET_IDLE || RfNet.Status==RFNET_DONE)
										    {
										        uint8_t k;
										        for(k=0u;k<3u;k++)
										        {
										            RfNet.Repeaters[k].TestCnt    = 0u;
										            RfNet.Repeaters[k].SuccessCnt = 0u;
										            RfNet.Repeaters[k].FailureCnt = 0u;
										        }
										        RfNet.TestIndex  = 0u;
										        RfNet.Status     = RFNET_CYCLEINDEX;
										        UI.dis_data_flag = 1u;
										    }
										    break;
										}
										case 1: /* 1 返回：回到菜单列表 */
										{
										    if(RfNet.Status==RFNET_IDLE || RfNet.Status==RFNET_DONE)
										    {
										        UI.Addr=Set_Addr1;
										        UI.Setting_flag=1;
										    }
										    break;
										}
										default:
										    break;
									}
									break;
								case Opt2: /* 设备绑定：0 计数 / 1 一键解绑 /
								              2 绑定设备 / 3 返回 */
									switch(UI.Addr)
									{
										case 0: /* 0 已绑定设备数：就地绘制 */
											break;
										case 1: /* 1 一键解绑 -> 二级子页 */
											UI.Sub2 = Sub2_Unbind;
											UI.Addr = Set_Addr1;
											UI.dis_data_flag = 1;
											break;
										case 2: /* 2 绑定设备 -> 二级子页
												           （本页同时打开扫描窗口） */
											UI.Sub2 = Sub2_BindDev;
											UI.Addr = Set_Addr1;
											UI.SubPage = 1;
											UI.dis_data_flag = 1;
											break;
										case 3: /* 3 返回 -> 菜单列表 */
											UI.Addr = Set_Addr1;
											UI.Setting_flag = 1;
											break;
										default:
											break;
									}
									break;
								case Opt3: /* 安装调试：0 信号 / 1 电压 / 2 返回 */
							    switch(UI.Addr)
									{
											case 0:                     /* 0 信号 -> 20 通道 RSSI（单页全屏）*/
													UI.Sub2 = Sub2_Signal;
													UI.dis_data_flag = 1;
												break;
											case 1:                     /* 1 电压 -> 20 通道电压（单页全屏）*/
													UI.Sub2 = Sub2_Volt;
													UI.dis_data_flag = 1;
												break;
											case 2:                     /* 2 返回 -> 菜单列表 */
													UI.Addr=Set_Addr1;
									    	UI.Setting_flag=1;
												break;
										  default:
											  break;
									}
									break;
								case Opt4:
									switch(UI.Addr)
									{
										  case 0:
									    case 1:
										  case 2:
													UI.Addr++;
													break;
											case 3: // save
											  UI.Addr=0;
										  	UI.Setting_flag=1;
											  memcpy(Setup.transmit_number,backups_Setup.transmit_number,3);
										  	Save_data();
												save_data_flash_flag=1;
											break;
										 case 4: // no save
											  UI.Addr=0;
											  UI.Setting_flag=1;
											break;
										 default:
											break;
									}
									break;
								case Opt5:
									switch(UI.Addr)
									{
										case 0:
											UI.Addr++;
											break;
										case 1:
										  UI.Addr=0;
											break;
										case 2:
                     UI.Addr++;
											break;
										case 3:
                     UI.Addr=2;
											break;
										case 4:
                     UI.Addr++;
											break;
										case 5:
                     UI.Addr=4;
											break;
										case 6: // save
										Setup.brightness =	backups_Setup.brightness;
										Setup.transmitted_power=  backups_Setup.transmitted_power;
										Setup.On_Off_flag=backups_Setup.On_Off_flag;
										UI.Addr=0;
										UI.Setting_flag=1;
										save_data_flash_flag=1;
										Save_data();
										break;
										case 7: // no save
										UI.Addr=0;
										UI.Setting_flag=1;
											break;
										default:
											break;
									}
									break;
								case Opt6: /* 信息汇总：0 名称 / 1 名称错误 /
								              2 电压错误 / 3 恢复出厂 / 4 返回 */
									switch(UI.Addr)
									{
										case 0:                     /* 0 绑定名称 -> 单页全屏 20 条 */
											 UI.Sub2 = Sub2_BindNam;
											 UI.dis_data_flag = 1;
											break;
										case 1:                     /* 1 名称错误计数：就地绘制 */
										case 2:                     /* 2 电压错误计数：就地绘制 */
											break;
										case 3:                     /* 3 恢复出厂 -> 确认页 */
											 UI.Sub2 = Sub2_Factory;
											 UI.Addr = Set_Addr1;
											 UI.dis_data_flag = 1;
											break;
										case 4:                     /* 4 返回 -> 菜单列表 */
											 UI.Addr = Set_Addr1;
											 UI.Setting_flag = 1;
											break;
										 default:
											break;
									}
									break;
								case Opt7: /* 7 返回主页：必须真正离开这一页 */
									UI.Setting_flag=0;
									UI.Addr=Set_Addr1;
									UI.Sub2=Sub2_None;
									DisOpt=Opt0;
									UI.dis_data_flag=1;
									break;
								default:
									break;
							}
							break;
					}

            printf("KEY2_Short !\r\n");
          break;
					case KEY3_PRES:
							switch(UI.Setting_flag)
							{
								case 0:   /* 主页：K1/K3 翻 20 通道列表 */
									UI.MainPage = (UI.MainPage == 2) ? 1 : 2;
									UI.dis_data_flag = 1;
								break;
								case 1:
									/* ★ V1_0_7：同 KEY1，菜单级显式置刷新标志 */
									UI.dis_data_flag = 1;
									UI.Sub2 = Sub2_None;   /* 退出子页 */
									switch(DisOpt)
									{
										case Opt0:
											DisOpt  = Opt7;
											break;
										default:
											DisOpt --;
                                            /* 反向切换也跳过组网测试。 */
                                            if (Setup.On_Off_flag == 0u && DisOpt == Opt1)
                                            {
                                                DisOpt = Opt0;
                                            }
											break;
									}
									break;
								case 2:
									UI.dis_data_flag=1;
									if(UI.Sub2 != Sub2_None)
									{
										sub2_nav();          /* 二级子页：切换项目（三张纯显示页里 sub2_nav() 无操作） */
										break;
									}
									switch(DisOpt)
									{
										case Opt0:
											switch(UI.Addr)
											{
												case 0:
													UI.Addr=5;
													break;
												case 1:
														if(backups_Setup.partition>0)
														{
																backups_Setup.partition--;
														}
														else
														{
															backups_Setup.partition=64;
														}
														break;
												case 2:
														UI.Addr=0;
														break;
												case 3:
														if(backups_Setup.Host_number>0)
														{
															backups_Setup.Host_number--;
														}
														else
														{
															backups_Setup.Host_number=121;
														}
													break;
												case 4:
													UI.Addr=2;
													break;
												case 5:
													UI.Addr=4;
													break;
												default:
													break;
											}
											break;
										case Opt1:
											/* ★ V1_0_7（P1）：与 KEY1 同一守卫 —— 组网页只有 0/1 两项，反向循环 */
											if(RfNet.Status==RFNET_IDLE || RfNet.Status==RFNET_DONE)
											{
												switch(UI.Addr)
												{
													case 0: /* 0 开始组网 */
														UI.Addr=Set_Addr2;   /* -> 1 返回 */
														break;
													case 1: /* 1 返回 */
														UI.Addr=Set_Addr1;   /* -> 0 开始组网 */
														break;
													default:
														UI.Addr=Set_Addr1;
														break;
												}
											}
											break;
										case Opt2: /* 设备绑定：0/1/2/3 */
											switch(UI.Addr)
											{
												case 0: UI.Addr=Set_Addr4; break;
												case 1: UI.Addr=Set_Addr1; break;
												case 2: UI.Addr=Set_Addr2; break;
												case 3: UI.Addr=Set_Addr3; break;
												default: UI.Addr=Set_Addr1; break;
											}
											break;
										case Opt3: /* 安装调试：0/1/2 */
											switch(UI.Addr)
											{
												case 0: UI.Addr=Set_Addr3; break;
												case 1: UI.Addr=Set_Addr1; break;
												case 2: UI.Addr=Set_Addr2; break;
												default: UI.Addr=Set_Addr1; break;
											}
											break;
										case Opt4:
												switch(UI.Addr)
								      	{
														case 0:
														if(backups_Setup.transmit_number[0]==0)
														{
															backups_Setup.transmit_number[0]=122;
														}
														else{
																backups_Setup.transmit_number[0]--;
														}
														break;
													case 1:
														if(backups_Setup.transmit_number[1]==0)
														{
															backups_Setup.transmit_number[1]=122;
														}
														else{
																backups_Setup.transmit_number[1]--;
														}
														break;
													case 2:
													if(backups_Setup.transmit_number[2]==0)
														{
															backups_Setup.transmit_number[2]=122;
														}
														else{
																backups_Setup.transmit_number[2]--;
														}
														break;
										     case 3:
													 UI.Addr=4;
													 break;
												 case 4:
													  UI.Addr=3;
													break;
												 default:
													 break;
											  }
											break;
										case Opt5:
											switch(UI.Addr)
												{
													case 0:
														UI.Addr=7;
														break;
													case 1:
													  if(backups_Setup.transmitted_power>-4)
														{
															backups_Setup.transmitted_power--;
														}
														else
														{
															backups_Setup.transmitted_power=22;
														}
														break;
													case 2:
														UI.Addr=0;
														break;
													case 3:
														  if(backups_Setup.brightness>5)
														{
															backups_Setup.brightness-=5;
														}
														else
														{
															backups_Setup.brightness=100;
														}
														break;
													case 4:
														UI.Addr=2;
														break;
													case 5:
														if(backups_Setup.On_Off_flag==0)
														{
															backups_Setup.On_Off_flag=1;
														}
														else{
															backups_Setup.On_Off_flag=0;
														}
														break;
													case 6:
														UI.Addr=4;
														break;
													case 7:
														UI.Addr=6;
														break;
													default:
													break;
												}
											break;
										case Opt6: /* 信息汇总：0/1/2/3/4 */
											switch(UI.Addr)
											{
												case 0: UI.Addr=Set_Addr5; break;
												case 1: UI.Addr=Set_Addr1; break;
												case 2: UI.Addr=Set_Addr2; break;
												case 3: UI.Addr=Set_Addr3; break;
												case 4: UI.Addr=Set_Addr4; break;
												default: UI.Addr=Set_Addr1; break;
											}
											break;
										case Opt7:
											break;
										default:
											break;
									}
									break;
							}
						printf("KEY3_Short !\r\n");
          break;
          default:
              break;
       }
              num_on=0;
       /* ★ V1_0_7：这里原来会开"防重复触发窗口"；现在抗抖动在 10ms 中断里按真实
          时间做（连续 2 拍 = 20ms 才认），不需要它，也避免吞掉正常连按。 */   }
   /* ★ 长按：按住时间够久（或者回调次数够多）就算，不依赖主循环速率 */
   else if(num_on>0&&key_is_long_press())
   {
     UI.dis_data_flag = 1;   /* ★ V1_0_7：长按动作同样保证刷新（页面立刻跟着变） */
     if(old_key_flag!=key_value)
     {
         switch(key_value)
         {
            case 0:
            break;
            case KEY1_PRES:
              printf("KEY1_Loog !\r\n");
            break;
            case KEY2_PRES:
              printf("KEY2_Loog !\r\n");
            break;
						case KEY3_PRES:
              printf("KEY3_Loog !\r\n");
            break;
         }
         s_long_fired    = 1U;        /* ★ 本次按下已按长按处理，松手那拍不再走短按 */
         key_repeat_tick = Rt_tick;   /* ★ 进入长按：连发计时从这一刻起算 */
     }
		 else
		 {
       /* ★ 连发改成按真实时间（100ms 节拍）计间隔，不再数回调次数，
          这样主循环快慢都不会改变长按连发的速度。 */
       num_on2++;
       if((uint32_t)(Rt_tick - key_repeat_tick) >= KEY_REPEAT_TICKS)
       {
         key_repeat_tick = Rt_tick;
				  switch(key_value)
         {
            case 0:
            break;
            case KEY1_PRES:
						UI.dis_data_flag=1;
           	if(UI.Setting_flag==2&&UI.Sub2==Sub2_None)   /* 二级子页里不得改设置值 */
						{
								switch(DisOpt)
								{
									case Opt0:
										switch(UI.Addr)
										{
											  case 1:
														backups_Setup.partition+=10;
														if(backups_Setup.partition>64)
														{
															backups_Setup.partition=0;
														}
													break;
												case 3:
														backups_Setup.Host_number+=10;
														if(backups_Setup.Host_number>121)
														{
															backups_Setup.Host_number=0;
														}
													break;
												default:
													break;
										}
										break;
									case Opt4:
										   switch(UI.Addr)
								      	{
													case 0:
														backups_Setup.transmit_number[0]+=10;
													  if(backups_Setup.transmit_number[0]>122)
														{
															backups_Setup.transmit_number[0]=0;
														}
														break;
													case 1:
														backups_Setup.transmit_number[1]+=10;
													  if(backups_Setup.transmit_number[1]>122)
														{
															backups_Setup.transmit_number[1]=0;
														}
														break;
													case 2:
														backups_Setup.transmit_number[2]+=10;
													  if(backups_Setup.transmit_number[2]>122)
														{
															backups_Setup.transmit_number[2]=0;
														}
														break;
													default:
													 break;
											 }
										break;
									case Opt5:
										    switch(UI.Addr)
												{
													case 0:
														UI.Addr=2;
														break;
													case 1:
														backups_Setup.transmitted_power++;
													  if(backups_Setup.transmitted_power>22)
														{
															backups_Setup.transmitted_power=-3;
														}
														break;
													case 3:
														backups_Setup.brightness+=10;
													  if(backups_Setup.brightness>120)
														{
															backups_Setup.brightness=5;
														}
														break;
													default:
									        	break;
													}
										break;
									default:
										break;
								}
						}
              printf("KEY1_Loog !\r\n");
            break;
            case KEY2_PRES:
              printf("KEY2_Loog !\r\n");
            break;
						case KEY3_PRES:
									UI.dis_data_flag=1;
               	if(UI.Setting_flag==2&&UI.Sub2==Sub2_None)   /* 二级子页里不得改设置值 */
								{
										switch(DisOpt)
										{
											case Opt0:
												switch(UI.Addr)
												{
													case 1:
															if(backups_Setup.partition>10)
															{
																	backups_Setup.partition-=10;
															}
															else
															{
																backups_Setup.partition=32;
															}
															break;
													case 3:
															if(backups_Setup.Host_number>10)
															{
																backups_Setup.Host_number-=10;
															}
															else
															{
																backups_Setup.Host_number=121;
															}
														break;
													default:
														break;
												}
												break;
											case Opt4:
												switch(UI.Addr)
								      	{
														case 0:
														if(backups_Setup.transmit_number[0]<10)
														{
															backups_Setup.transmit_number[0]=122;
														}
														else{
																backups_Setup.transmit_number[0]-=10;
														}
														break;
													case 1:
														if(backups_Setup.transmit_number[1]<10)
														{
															backups_Setup.transmit_number[1]=122;
														}
														else{
																backups_Setup.transmit_number[1]-=10;
														}
														break;
													case 2:
													if(backups_Setup.transmit_number[2]<10)
														{
															backups_Setup.transmit_number[2]=122;
														}
														else{
																backups_Setup.transmit_number[2]-=10;
														}
														break;
												 default:
													 break;
											  }
												break;
											case Opt5:
												switch(UI.Addr)
												{
													case 0:
														UI.Addr=7;
														break;
													case 1:
													  if(backups_Setup.transmitted_power>-4)
														{
															backups_Setup.transmitted_power--;
														}
														else
														{
															backups_Setup.transmitted_power=22;
														}
														break;
													case 3:
														  if(backups_Setup.brightness>10)
														{
															backups_Setup.brightness-=10;
														}
														else
														{
															backups_Setup.brightness=100;
														}
														break;
													default:
													break;
												}
												break;
											default:
												break;
										}
								}
              printf("KEY3_Loog !\r\n");
            break;
         }
       }
     }
		 old_key_flag=key_value;
	 }
   /* ★ 长按结束后收尾：与上面的长按判定保持一致（用 s_long_fired 收口） */
   if(key_value==0&&num_on>0&&s_long_fired!=0U)
   {
       num_on=0;
		   num_on2=0;
       s_long_fired=0U;
   }
}
