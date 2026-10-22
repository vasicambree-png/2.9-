/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for the usart.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "stdio.h"

extern UART_HandleTypeDef huart1;   // USART1 -> CH584M display
extern UART_HandleTypeDef huart2;   // USART2 -> debug printf

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void UART1_SendString(const char *str);

/* uart receive struct */
typedef struct {
	  uint8_t  rx_data;
    uint8_t  rx_buffer[128];
    uint16_t rx_length;
    uint8_t  rx_flag;
}Rx_pro;

void UART1_SendString_Explicit(const char *str, uint16_t len);
void print_uart_sr(UART_HandleTypeDef *huart);
void MX_DMA_Init(void);
void hex_p(uint8_t *buf, int len);

/* ==========================================================================
 * cmd 0x03 ：传感器数据读取（STM32 请求 / CH584M 应答）
 * --------------------------------------------------------------------------
 * 帧格式契约，已冻结（见 协议_0x03_传感器数据读取_规格冻结.md）：
 *   AA EE | LEN(2 字节，大端) | CMD | DATA(N) | SUM | 0A
 *   LEN = 1(CMD) + N(DATA)          SUM = (CMD + DATA) & 0xFF
 *   整帧长度 = LEN + 6
 *
 *   请求（STM32 -> CH584M）：AA EE 00 01 03 03 0A               7 字节
 *   应答（CH584M -> STM32）：AA EE 00 51 03 <80 字节> <SUM> 0A  87 字节
 *   80 个数据字节 = 20 通道 × { 类型 1B | 数据 2B 大端 | 电压 1B }
 *   通道 i 位于应答偏移 5 + i*4，映射到 Ch_senser[i]
 *   无效通道 = 类型 0x00（T_None）/ 数据 0x0000 / 电压 0x00
 *   类型字节用的是与 Data_tpye（LoraPro.h）完全相同的数值，
 *   所以不做任何转换，直接强转进 Ch_senser[i].tpye。
 *
 * 接收路径：USART1 RXNE 中断 -> sensor_rx_isr_handler()（入队 + 重新启动）
 *           主循环           -> sensor_rx_task()（分帧 + 校验 + 填表）
 * 发送路径：常态（未息屏）不再发送 —— 0x03 只在息屏读数据序列里发一次，
 *           见本文件末尾的"息屏读数据子系统"sleep_read_service()
 * ========================================================================== */
#define SENSOR_CMD              0x03U   /* 0x03 传感器读取命令的 CMD           */
#define SENSOR_FRAME_HEAD0      0xAAU   /* 帧头第 0 字节                       */
#define SENSOR_FRAME_HEAD1      0xEEU   /* 帧头第 1 字节                       */
#define SENSOR_FRAME_TAIL       0x0AU   /* 帧尾                                */
#define SENSOR_REQ_LEN_FIELD    0x0001U /* 请求的 LEN = 1（只有 CMD）           */
#define SENSOR_RSP_LEN_FIELD    0x0051U /* 应答的 LEN = 81（CMD + 80）          */
#define SENSOR_REQ_LEN          7U      /* 请求整帧长度 = 1 + 6                 */
#define SENSOR_RSP_LEN          87U     /* 应答整帧长度 = 81 + 6                */
#define SENSOR_RSP_DATA_LEN     80U     /* 20 通道 × 4 字节                     */
#define SENSOR_CH_NUM           20U     /* 通道数由契约固定                     */

/* ==========================================================================
 * cmd 0x06 ：CH584M -> STM32「一轮采集完成」（主动上报，不是应答）
 * --------------------------------------------------------------------------
 *   上报（CH584M -> STM32）：AA EE 00 51 06 <80 字节> SUM 0A      87 字节
 * 载荷与 0x03 的应答逐字节相同（20 通道 ×
 * { 类型 1B | 数据 2B 大端 | 电压 1B }），SUM 从 CMD 字节开始累加，
 * LEN = 81、整帧长度 87，与 0x03 完全一样。
 * 因此它由完全相同的代码路径校验并生效 —— 没有第二个解析器，
 * 也没有第二份通道表更新代码。
 * ========================================================================== */
#define SENSOR_CMD_CYCLE        0x06U   /* 主动上报一轮采集结果的 CMD */
#define SENSOR_CYCLE_LEN_FIELD  0x0051U /* 上报的 LEN = 81（CMD + 80） */
#define SENSOR_CYCLE_LEN        87U     /* 上报整帧长度 = 81 + 6       */

/* ==========================================================================
 * 息屏读数据子系统（冻结契约 协议_息屏省电数据链与页面体系_冻结.md §3.2）
 * --------------------------------------------------------------------------
 * 常态（未息屏）不再由 STM32 轮询 0x03，数据由 CH584M 主动上报 0x06；
 * 息屏后 CH584M 整机断电，每隔 SLP_READ_PERIOD_SEC 秒才上电一次，
 * 发一发 0x03 读一轮数据，读到应答（或超时）马上再断电，其余时间 STM32
 * 处于 STOP —— 整个息屏期间 STM32 一帧 0x01 都不发，屏幕不刷新。
 *
 * ★ 硬件事实（从源码里查出来的，不是猜的）：CH584M 的供电 =
 *   STM32 PB4「屏幕供电控制」（Core/Src/gpio.c:83）。
 *   PB4 的读写全工程只有下面这两个函数一处（lcd_power_on/lcd_power_off），
 *   极性只由 LCD_PWR_ON_LEVEL / LCD_PWR_OFF_LEVEL 两个宏决定：
 *   实测若相反（PB4 直接驱动 PMOS 栅极、低电平导通），把这两个宏对调即可
 *   （冻结契约 §7.1，待用户确认）。
 *   注意 Core/Inc/gpio.h 里那两个历史遗留、无人调用的
 *   LCD_ON_Power / LCD_OFF_Power 宏极性正好相反，不要参考它们。
 * ========================================================================== */
#define SLP_READ_ENABLE       1      /* 1 = 启用息屏读数据；0 = 退回旧行为 */
#define SLP_READ_PERIOD_SEC   600U   /* 息屏采集及 LoRa 周期统一为 600 秒（10 分钟） */
#define SLP_FIRST_READ_DELAY_SEC  SLP_READ_PERIOD_SEC  /* 息屏后第一次读数据前先等多久；
                                         调试时改小（例如 10）可快速验证息屏读数据链 */
#define UI_BOOT_WAIT_MS       400U   /* 上电/复位后等 CH584M 启动(实测~304ms)再发首帧 */
#define SLP_BOOT_WAIT_MS      500U   /* V1_0_37: 600->500, 缩短息屏获取时间(CH584实测启动~300-500ms) */
#define SLP_READ_TIMEOUT_MS   36000U  /* 等 0x03 应答的上限（> CH584M 的 30 s，
                                         也 > 在途守卫的 35 s，便于区分两种收尾） */
#define SLP_SLEEP_CHUNK_MS    50U    /* 息屏空闲 STOP 的每段长度。
                                         ★ V1_0_12：500 -> 100ms。
                                         为什么必须改：息屏时主循环每遍都要走这一段 STOP，
                                         它直接决定了主循环周期；而 LoRa 的 CAD 是挂在
                                         100ms 节拍上的（见 LoraPro.c 的 Timer_task）。
                                         留 500ms 会把 CAD 周期顶到 500ms 以上，前导 192ms
                                         就盖不住空中任意时刻的包。改成 100ms 后与 CAD
                                         节拍对齐：一睡一醒正好一拍。
                                         仍远小于看门狗(≈2.08s)，见 Sleep.c 的说明。 */

#define LCD_PWR_ON_LEVEL      GPIO_PIN_RESET     /* PB4 供电电平（§7.1 默认） */
#define LCD_PWR_OFF_LEVEL     GPIO_PIN_SET   /* PB4 断电电平（§7.1 默认） */

/* ==========================================================================
 * 息屏读数据子系统（V1_0_2 修正：**非阻塞状态机**）
 * --------------------------------------------------------------------------
 * ★ 旧实现（V1_0_1 / 用户改的 V1_0_2）的致命问题：
 *   Sleep.c 的 sleep_read_idle() 在息屏后**独占主循环**：
 *       while (sleep_mode == 2) { 500ms STOP; sleep_read_task(); }
 *   而 sleep_read_task() 自己又阻塞等 3 s（等 CH584M 启动）
 *   + 最多 35 s（等 0x03 应答）。于是息屏期间：
 *     1) main() 的 while(1) 一步都走不动 -> Lora_task() / Time_check_task()
 *        不再被调用 -> **LoRa 收发与上报全部停摆**；
 *     2) Light_check() 不再被调用 -> **光唤醒彻底失效**；
 *     3) 那几十秒全靠循环里手动 re_iwdg() 续命，任何一次等待漏喂，
 *        IWDG（Prescaler 64 / Reload 1200 / LSI 约 37 kHz -> ≈2.08 s）
 *        立刻复位 —— 现场看到的就是"息屏读数据时直接看门狗复位"。
 *
 * ★ 新实现：sleep_read_service() 是一个**每次主循环经过只推进一步**的
 *   状态机（IDLE -> BOOT -> WAIT_RSP -> PWR_OFF，每步几十毫秒就返回），
 *   由 Sleep.c 的 sleep_check() case 2 调用。主循环因此在息屏期间照常转：
 *   LoRa、光检测、按键、喂狗全部不受影响。
 *     ① IDLE     ：等 SLP_FIRST_READ_DELAY_SEC / SLP_READ_PERIOD_SEC 到期
 *     ② 上电      ：lcd_power_on() + 重新初始化 USART1 + 重启 1 字节接收
 *     ③ BOOT     ：用 Rt_tick（LPTIM1，STOP 下仍在走）计时等 SLP_BOOT_WAIT_MS
 *     ④ WAIT_RSP ：发一次 0x03，等应答 <= SLP_READ_TIMEOUT_MS
 *     ⑤ PWR_OFF  ：断电 + 关 USART1 + 复位状态，下一周期重新起算
 *
 *   ★ 计时一律用 Rt_tick，**不能用 HAL_GetTick()**：SysTick 在 STOP 下
 *     不计数，用它计时会永远等不到超时。
 *   ★ 读取窗口内 g_slp_keep_awake = 1，Sleep.c 的 sleep_timer() /
 *     sleep_stop_ms() 据此**不进 STOP** —— USART1 的时钟源是 HSI，
 *     STOP 下停振，一进 STOP 必然丢掉 CH584M 应答的字节。
 * ========================================================================== */
extern uint8_t g_slp_keep_awake;   /* 1 = 息屏读数据窗口，禁止进 STOP */

void lcd_power_on(void);      /* 唯一的 PB4 上电点：CH584M 上电         */
void lcd_power_off(void);     /* 唯一的 PB4 断电点：CH584M 彻底断电     */
void sleep_read_begin(void);  /* 进入息屏时调用：复位息屏读数据状态机   */
void sleep_read_service(void);/* 主循环：推进一步（非阻塞，绝不在这里等）*/
void sleep_read_chunk_tick(void); /* 睡完一段 500ms STOP 后调用（备用计时基准）*/

uint16_t build_sensor_request(uint8_t *buf);  /* 组出 7 字节请求帧         */
void sensor_request_send(void);               /* 组帧 + 发送（主循环）     */
void sensor_rx_start(void);                   /* 1 字节中断接收的首次启动  */
void sensor_rx_isr_handler(void);             /* 中断：只入 FIFO + 重新启动 */
void sensor_rx_task(void);                    /* 主循环：分帧 + 解析       */
void sensor_rx_stats_report(void);            /* 错误计数有变化才打印      */

/* ==========================================================================
 * cmd 0x03 ：「请求在途」守卫   （冻结契约 §3.1）
 * --------------------------------------------------------------------------
 * CH584M 现在不再立刻应答 0x03 请求了：它会等到本轮里每一台仍然在线的已绑定
 * 设备都送来一次新采样，只有在过了 30 s 之后才用上一轮的值应答
 * （冻结契约 §2.6）。在这样一条请求在途期间，CH584M 会忽略后面任何 0x03
 * 请求，所以按 1 Hz 轮询只会堆起一堆过期应答，
 * 把旧数据反复写回 Ch_senser[]。
 *
 * 因此本驱动最多只保留一条 0x03 请求在途：
 *   - sensor_request_send() 在发送成功后立刻置位该标志；
 *   - 校验通过的 0x03 应答会清掉它（sensor_parse_frame()）；主动上报的 0x06
 *     不会清，因为它不是对我们那条请求的应答；
 *   - SENSOR_REQ_TIMEOUT_SEC（35 s > CH584M 的 30 s）会在主循环里作为安全网
 *     清掉它，所以丢一次应答绝不会把轮询卡死。
 * 所有触发点都走 sensor_request_send()，
 * 所以守卫覆盖了全部触发点（目前是 1 秒软定时器，
 * 以及将来可能加的 20 ms 触发）。
 * ========================================================================== */
#define SENSOR_REQ_TIMEOUT_SEC   35U                       /* > CH584M 的 30 s */
#define SENSOR_REQ_TIMEOUT_TICKS (SENSOR_REQ_TIMEOUT_SEC * 10U) /* Rt_tick=100 ms   */

/* ==========================================================================
 * cmd 0x04 ：解绑全部   /   cmd 0x05 ：把绑定保存到 Flash
 * --------------------------------------------------------------------------
 * 帧格式契约，已冻结（见 0x04/0x05 规格冻结文档）：
 *   AA EE | LEN(2 字节，大端) | CMD | DATA(N) | SUM | 0A
 *   LEN = 1(CMD) + N(DATA)          SUM = (CMD + DATA) & 0xFF
 *   整帧长度 = LEN + 6
 *
 *   请求（STM32 -> CH584M）：AA EE 00 01 04 04 0A                 7 字节
 *                             AA EE 00 01 05 05 0A                 7 字节
 *   应答（CH584M -> STM32）：AA EE 00 02 <CMD> <status> <SUM> 0A  8 字节
 *   只有帧尾 0x0A、SUM 和 LEN==2 全部通过才接受应答；
 *   其它情况整帧丢弃，不产生任何副作用。
 *   这两条命令共用 0x03 的接收链（USART1 中断 -> FIFO -> 分帧器），
 *   在那里按 CMD 分派。
 * ========================================================================== */
/* ==========================================================================
 * cmd 0x07 ：恢复出厂设置
 * --------------------------------------------------------------------------
 * 帧格式与校验规则和 0x04 / 0x05 完全相同（帧头 AA EE、帧尾 0A、
 * 请求 LEN == 1、校验和覆盖 CMD..最后一个数据字节）：
 *
 *   请求（STM32 -> CH584M）：AA EE 00 01 07 07 0A                 7 字节
 *   应答（CH584M -> STM32）：AA EE 00 02 07 <status> <SUM> 0A     8 字节
 *
 *   0x07 状态码：00 = 已清除（原本有绑定）/ 01 = 原本没有绑定
 *
 * 非法帧整帧丢弃：不回任何东西，也不产生副作用。
 * （cmd 0x08「读取绑定统计」已按冻结契约 §3.2 删除 —— 两端都已去掉，
 *  所以不再存在任何 0x08 帧。）
 * ========================================================================== */
#define BINDING_CMD_UNBIND      0x04U   /* 解绑所有绑定及其 Flash 记录    */
#define BINDING_CMD_SAVE        0x05U   /* 持久化当前绑定表               */
#define BINDING_CMD_FACTORY     0x07U   /* 恢复出厂（绑定 + Flash 记录）  */
#define BINDING_REQ_LEN_FIELD   0x0001U /* 0x04/0x05/0x07 请求的 LEN      */
#define BINDING_RSP_LEN_FIELD   0x0002U /* 0x04/0x05/0x07 应答的 LEN      */
#define BINDING_REQ_LEN         7U      /* 请求整帧长度 = 1 + 6           */
#define BINDING_RSP_LEN         8U      /* 应答整帧长度 = 2 + 6           */
#define BINDING_TX_TIMEOUT      100U    /* 单位 ms，与 0x03 请求相同      */

/* 0x04 应答状态码 */
#define BINDING_ST_UNBIND_OK    0x00U   /* 绑定已清除                     */
#define BINDING_ST_UNBIND_NONE  0x01U   /* 原本没有绑定（仍算成功）       */
/* 0x05 应答状态码 */
#define BINDING_ST_SAVE_OK      0x00U   /* 已写入并回读校验通过           */
#define BINDING_ST_SAVE_NONE    0x01U   /* 没有绑定，无需保存             */
#define BINDING_ST_SAVE_FAIL    0x02U   /* Flash 写入或回读校验失败       */
/* 0x07 应答状态码 */
#define BINDING_ST_FACT_OK      0x00U   /* 已清除，原本有绑定             */
#define BINDING_ST_FACT_NONE    0x01U   /* 原本没有绑定，无需清除         */

uint16_t build_binding_request(uint8_t *buf, uint8_t cmd); /* 7 字节请求帧       */
void send_unbind_all(void);                               /* cmd 0x04，阻塞发送 */
void send_save_binding(void);                             /* cmd 0x05，阻塞发送 */
void send_factory_reset(void);                            /* cmd 0x07，阻塞发送 */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
