/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "ui.h"
#include "LoraPro.h"    /* Channel_C / Data_tpye / Ch_senser（cmd 0x03 的载荷用到） */
#include "iwdg.h"       /* re_iwdg()：息屏读数据窗口里要喂狗（STOP 下 IWDG 仍在计数） */
#include "New_Key.h"   /* Flash_Set Setup（诊断用） */
#include <stdio.h>

UART_HandleTypeDef huart1;   // USART1 -> CH584M display
UART_HandleTypeDef huart2;   // USART2 -> debug printf
DMA_HandleTypeDef hdma_usart1_tx;

#define BUFFER_SIZE  256
uint8_t uart_tx_buffer[BUFFER_SIZE];

volatile uint32_t tx_buffer_put = 0;
volatile uint32_t tx_buffer_out = 0;
volatile uint8_t is_dma_busy = 0;
uint32_t last_transfer_len = 0;
#define PRINTF_ON   1

/* printf redirect to USART2 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* ==========================================================================
 * ★★ 第二轮修正：printf / UART 阻塞发送的超时必须有限
 * --------------------------------------------------------------------------
 * 原来 fputc 传的超时是 0xFFFF = **65535 ms**（不是 65 ms！），
 * UART1_SendString* 传的是 HAL_MAX_DELAY = **0xFFFFFFFF（无限等）**。
 * HAL_UART_Transmit 是轮询式阻塞发送：如果 TXE/TC 一直不置位
 * （USART 时钟被关、引脚被别的外设占用、硬件异常），
 * 一次 printf 的**单个字符**就能卡 65 秒，而 IWDG 只有 ≈2.08 s ——
 * 结果是"打印一下就把整机打到看门狗复位"，而且是在最不该出错的地方。
 * 现在统一给有限超时：
 *   · 调试口 USART2 单字节：**1 ms**（115200 下 1 字节只要 0.087 ms，1ms 仍有 11 倍余量）；
 *   · 协议口 USART1 整帧：200 ms（最长 87 字节 @115200 约 7.6 ms，200ms 有 26 倍余量）。
 * 超时后 HAL_UART_Transmit 直接返回 HAL_TIMEOUT，调用方不关心返回值也不影响功能，
 * 但整机绝不会因此卡死。
 *
 * ★★ 第三轮（看门狗审查）为什么把 10ms 又收紧到 1ms：
 *   10ms 对**单字符**是 115 倍余量，但 printf 是**逐字符**调用本函数的，
 *   而 hex_p() 一次会打印 400 个字符（LoraPro.c 里最多 98 字节的数据块）。
 *   一旦 USART2 的 TXE 出问题，400 字符 × 10ms = **4 秒不喂狗**
 *   （IWDG 只有约 2.08s）-> 必然看门狗复位。
 *   1ms 时最坏 400ms，仍然远小于看门狗。
 * ========================================================================== */
#define DBG_TX_TIMEOUT_MS    1U     /* USART2 单字节 printf 的超时（见上面的算式） */
#define PROTO_TX_TIMEOUT_MS  200U   /* USART1 一帧的超时 */

PUTCHAR_PROTOTYPE
{
  #if PRINTF_ON
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, DBG_TX_TIMEOUT_MS);
    return ch;
  #endif
}

/* debug hex dump */
void hex_p(uint8_t *buf, int len)
{
	int i = 0;
    for (i = 0; i < len; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
}

void UART1_SendString_Explicit(const char *str, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, PROTO_TX_TIMEOUT_MS);
}

void UART1_SendString(const char *str)
{
    size_t len = strlen(str);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, PROTO_TX_TIMEOUT_MS);
}

void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 4);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

/* ==========================================================================
 * MX_USART1_UART_Init ：协议口（对 CH584M）初始化
 * --------------------------------------------------------------------------
 * ★★★ V1_0_2 修正（第四轮）：HAL 失败**不再**调 Error_Handler()
 * --------------------------------------------------------------------------
 * 为什么必须改：本函数不只是开机调一次 —— 息屏读数据子系统**每一轮**
 * 都会调用它（usart.c 的 sleep_read_service()"上电"那一步，因为
 * ALL_IO_CLOSE() 进 STOP 前把 USART1 整个 DeInit 掉了）。
 * 一旦失败就 Error_Handler() -> __disable_irq() + while(1) ->
 * **每 2 秒被看门狗复位一次**的复位循环，且串口日志全无。
 * 而且 `HAL_UART_Init()` 内部等 TEACK/REACK 的超时是
 * HAL_UART_TIMEOUT_VALUE ≈ **9.3 小时**（HAL 库常量），中途不喂狗 ——
 * 正常不会触发（要时钟开着才会进那个等待），但一旦触发就是必然复位。
 * 现在改成"打印 + 返回"：上层（sleep_read_service）的等待会自然超时，
 * 本轮记一次失败计数并收尾，主循环照常跑，按键/光检测/LoRa 都不受影响。
 * ========================================================================== */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    printf("usart1 init fail\r\n");   /* ★ 不致命：见上面的说明 */
  }
}

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_3;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      /* V1_0_60：这里原来调 Error_Handler()（__disable_irq + while(1)），
         表现就是"打完 BOR 就静默 2 秒被 IWDG 复位"，串口一行日志都没有。
         串口 DMA 起不来不该让整机死等：先清一次再试一遍，仍失败就打印后继续 ——
         上层发送会失败（s_ui_tx_fail 置位、UI_display 会补发），但按键/LoRa/喂狗全活。 */
      HAL_DMA_DeInit(&hdma_usart1_tx);
      if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
      {
        printf("usart1 dma init fail, continue\r\n");
      }
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_DMA_DeInit(&hdma_usart1_tx);   /* V1_0_60：连 DMA 句柄一起清，别留在 BUSY */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

#define LCD_TX_UART             huart1
extern volatile uint8_t  LCD_TX_state;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
   if(huart == &LCD_TX_UART)
   {
     LCD_TX_state=0;
   }
   if (huart->Instance == USART2)
   {
   }
}

void print_uart_sr(UART_HandleTypeDef *huart)
{
    printf("UART_FLAG_RWU       UART_FLAG_SBKF -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_RWU), __HAL_UART_GET_FLAG(huart, UART_FLAG_SBKF));
    printf("UART_FLAG_CMF       UART_FLAG_BUSY -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_CMF), __HAL_UART_GET_FLAG(huart, UART_FLAG_BUSY));
    printf("UART_FLAG_ABRF      UART_FLAG_ABRE -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_ABRF), __HAL_UART_GET_FLAG(huart, UART_FLAG_ABRE));
    printf("UART_FLAG_CTS       UART_FLAG_LBDF -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_CTS), __HAL_UART_GET_FLAG(huart, UART_FLAG_LBDF));
    printf("UART_FLAG_TXE         UART_FLAG_TC -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE), __HAL_UART_GET_FLAG(huart, UART_FLAG_TC));
    printf("UART_FLAG_RXNE      UART_FLAG_RTOF -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE), __HAL_UART_GET_FLAG(huart, UART_FLAG_RTOF));
    printf("UART_FLAG_IDLE       UART_FLAG_ORE -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE), __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE));
    printf("UART_FLAG_NE          UART_FLAG_FE -->  %X -->  %X\r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_NE), __HAL_UART_GET_FLAG(huart, UART_FLAG_FE));
    printf("UART_FLAG_PE                       --> %X \r\n", __HAL_UART_GET_FLAG(huart, UART_FLAG_PE));
}

/* ==========================================================================
 * cmd 0x03 ：传感器数据读取
 *   请求组帧 + 中断接收 + 分帧；帧格式常量与对外接口都在 usart.h 里有说明
 *
 * 数据流：
 *   常态（未息屏）：不发 0x03 —— 数据由 CH584M 每轮采集完成后用 cmd 0x06
 *                   主动上报，走下面这条完全相同的解析路径（契约 §3.1）
 *   息屏：sleep_read_service()（读取窗口内）-> sensor_request_send()
 *                               -> HAL_UART_Transmit
 *   USART1 RXNE 中断            -> HAL_UART_RxCpltCallback（stm32l0xx_it.c）
 *                               -> sensor_rx_isr_handler()（入 FIFO + 重新启动接收）
 *   主循环                       -> sensor_rx_task()（分帧 + 校验）
 *                               -> Ch_senser[0..19]
 *
 * ========================================================================== */

/* ---- 本文件内部可调参数 -------------------------------------------------- */
#define SENSOR_FIFO_SIZE     128U   /* 中断 -> 主循环 的环形缓冲，长度为 2 的幂 */
#define SENSOR_FIFO_MASK     (SENSOR_FIFO_SIZE - 1U)
#define SENSOR_LINE_SIZE     128U   /* 分帧器使用的线性缓冲                     */
#define SENSOR_MAX_FRAME     96U    /* 允许的最大帧长（规格里的帧是 87 字节）   */
#define SENSOR_TX_TIMEOUT    100U   /* 单位 ms；115200 波特率下 7 字节耗时 0.61 ms */

extern uint32_t Rt_tick;            /* 100 ms 系统节拍，定义在 stm32l0xx_it.c   */

/* ---- 接收字节 + 无锁环形缓冲（生产者：中断，消费者：主循环） */
 uint8_t   s_rx_byte;         /* HAL_UART_Receive_IT 的接收目标   */
static volatile uint8_t  s_fifo[SENSOR_FIFO_SIZE];
static volatile uint16_t s_fifo_head;       /* 只在中断里写                     */
static volatile uint16_t s_fifo_tail;       /* 只在主循环里写                   */

/* ---- 分帧器 -------------------------------------------------------------- */
static uint8_t  s_line[SENSOR_LINE_SIZE];   /* 已收到但尚未消费的字节           */
static uint16_t s_line_len;

/* ---- 诊断计数 ------------------------------------------------------------ */
static uint32_t s_tx_cnt;                   /* 已发出的请求帧数                 */
static uint32_t s_tx_err;                   /* HAL_UART_Transmit 发送失败次数   */
static uint32_t s_frame_ok;                 /* 已生效的应答帧数                 */
static uint32_t s_frame_err;                /* 被丢弃的帧数（帧尾 / SUM / LEN 校验不通过） */
static uint32_t s_other_frame;              /* 其它命令的合法帧数               */
static uint32_t s_byte_drop;                /* 重新同步时丢弃的字节数           */
static volatile uint32_t s_fifo_ovf;                 /* 接收环形缓冲溢出次数             */
static volatile uint32_t s_rx_err;                   /* USART1 硬件接收错误次数          */
static volatile uint32_t s_rearm;                    /* 在任务里重新启动接收的次数       */
static uint32_t s_reported_sum;             /* 上次上报时的错误计数总和         */

/* ---- 诊断：cmd 0x06（CH584M 主动上报的「一轮采集完成」） ------------------ */
static uint32_t s_cycle_ok;                 /* 已校验并生效的 0x06 上报数       */

/* ---- cmd 0x03「请求在途」守卫（冻结契约 §3.1） --------------------------
 * CH584M 只有在所有仍然在线、已绑定的设备都送来一次新采样之后（或者已过
 * 30 s，冻结契约 §2.6），才会应答一次 0x03 请求；而在一条请求在途期间，
 * 它会忽略后面来的 0x03 请求，所以在途期间再发只会得到过期数据，
 * 因此同一时刻最多只能有一条请求在途。 */
volatile uint8_t g_sensor_data_valid = 0U;   /* V1_0_58：开机后是否已收到第一帧完整采集 */
static uint8_t  s_req_pending;              /* 1 = 有一条 0x03 请求正在等应答 */
static volatile uint32_t s_req_pending_tick;         /* 该请求发出时的 Rt_tick          */
static uint32_t s_req_blocked;              /* 在途期间被跳过的 1 s 触发次数   */
extern uint8_t sleep_mode;
static uint8_t s_sleep_request_active;
static uint32_t s_req_reply_ok;
static uint32_t s_req_timeout;              /* 由 35 s 兜底释放的在途标志次数  */

/* ---- 诊断：cmd 0x04（解绑全部）/ cmd 0x05（保存绑定） -------------------- */
static uint32_t s_bind_tx;                  /* 已发出的 0x04/0x05 请求数        */
static uint32_t s_bind_tx_err;              /* 请求发送失败次数                 */
static uint32_t s_bind_rsp_ok;              /* 被接受并生效的应答数             */
static uint32_t s_bind_rsp_err;             /* 被丢弃的应答数（LEN / 总长不对） */
static uint8_t  s_bind_last_cmd;            /* 最后一次被接受结果的 CMD         */
static uint8_t  s_bind_last_status;         /* 最后一次被接受结果的状态码       */
static uint8_t  g_has_binding = 1U;      /* 第 11 轮：CH584M 当前是否有绑定设备。
                                       1 = 有/未知（默认，保守）；0 = 确定没有绑定。
                                       由 0x04/0x05/0x07 应答的状态码更新，
                                       收到有效 0x03/0x06 上报时置回 1。
                                       用于息屏读取：无绑定时直接跳过读取窗口，
                                       不让 STM32 白等一个超时（用户明确要求）。 */

/* --------------------------------------------------------------------------
 * build_sensor_request ：组出 7 字节请求帧 AA EE 00 01 03 03 0A
 * 返回写入的字节数（SENSOR_REQ_LEN）
 * ------------------------------------------------------------------------ */
uint16_t build_sensor_request(uint8_t *buf)
{
    uint16_t idx = 0U;
    uint16_t i;
    uint8_t  sum = 0U;

    if (buf == NULL)
    {
        return 0U;
    }

    buf[idx++] = SENSOR_FRAME_HEAD0;                        /* 第 0 字节：AA              */
    buf[idx++] = SENSOR_FRAME_HEAD1;                        /* 第 1 字节：EE              */
    buf[idx++] = (uint8_t)(SENSOR_REQ_LEN_FIELD >> 8);      /* 第 2 字节：LEN 高字节 = 0x00 */
    buf[idx++] = (uint8_t)(SENSOR_REQ_LEN_FIELD & 0xFFU);   /* 第 3 字节：LEN 低字节 = 0x01 */
    buf[idx++] = SENSOR_CMD;                                /* 第 4 字节：CMD     = 0x03 */

    /* SUM = CMD + DATA（取低 8 位）；本请求不带数据字节 */
    for (i = 4U; i < idx; i++)
    {
        sum = (uint8_t)(sum + buf[i]);
    }
    buf[idx++] = sum;                                       /* 第 5 字节：SUM     = 0x03 */
    buf[idx++] = SENSOR_FRAME_TAIL;                         /* 第 6 字节：0A             */

    return idx;                                             /* 共 7 字节                 */
}

/* --------------------------------------------------------------------------
 * sensor_req_pending_expire ：给「请求在途」标志兜底的 35 s 安全网。
 * CH584M 最迟 30 s 就会应答一次 0x03 请求（冻结契约 §2.6），所以超过
 * SENSOR_REQ_TIMEOUT_SEC（35 s）标志还置着，就说明应答丢了。
 * 这里主动释放它，而不是让 1 s 轮询被永久卡住。
 * 只在主循环上下文调用（中断不会碰这个状态）。
 * ------------------------------------------------------------------------ */
static void sensor_req_pending_expire(void)
{
    if (s_req_pending != 0U)
    {
        if ((uint32_t)(Rt_tick - s_req_pending_tick) >= SENSOR_REQ_TIMEOUT_TICKS)
        {
            s_req_pending = 0U;
            s_req_timeout++;
        }
    }
}

/* --------------------------------------------------------------------------
 * sensor_request_send ：阻塞式发送请求帧。
 * 由"息屏读数据"窗口 sleep_read_service()（本文件末尾）调用 —— 常态（未息屏）
 * 已经不再调用它（冻结契约 §3.1），所以它是全工程唯一的 0x03 触发点。
 * CH584M 显示帧走的是同一条阻塞发送路径，而且同样由主循环发出，
 * 所以这两个发送方永远不会重叠。
 * 每个 0x03 触发点都被守卫住（冻结契约 §3.1）：只要有请求在途，就不再组帧、
 * 不再发送，因为 CH584M 会忽略它，
 * 并继续拿旧数据应答。
 * ------------------------------------------------------------------------ */
void sensor_request_send(void)
{
    uint8_t  req[SENSOR_REQ_LEN];
    uint16_t len;

    sensor_req_pending_expire();        /* 超过 35 s 就释放该标志 */

    if (!s_req_pending)                 /* 守卫：没有请求在途 -> 才发送 */
    {
        len = build_sensor_request(req);
        if (len != SENSOR_REQ_LEN)
        {
            s_tx_err++;
            return;
        }

        if (HAL_UART_Transmit(&huart1, req, len, SENSOR_TX_TIMEOUT) == HAL_OK)
        {
            s_tx_cnt++;
            s_req_pending      = 1U;    /* 置位：应答还没回来……        */
            s_req_pending_tick = Rt_tick; /* ……同时启动 35 s 兜底计时    */
        }
        else
        {
            s_tx_err++;                 /* 什么都没发出去 -> 不会来应答，
                                           所以标志保持清零 */
        }
    }
    else
    {
        s_req_blocked++;                /* 已有一条请求在途：跳过本次触发，
                                           不要把 CH584M 淹没在它根本不理会
                                           的请求里 */
    }
}

/* --------------------------------------------------------------------------
 * 中断接收：一次收一个字节，这样分帧器就不依赖波特率 / 帧时序。
 * 中断里绝不碰传感器表。
 * ------------------------------------------------------------------------ */

/* 首次启动接收，在 MX_USART1_UART_Init() 之后调用；上一次会话残留的字节
   （深度睡眠会重新初始化 USART1）在这里被丢掉 */
void sensor_rx_start(void)
{
    s_fifo_head = 0U;
    s_fifo_tail = 0U;
    s_line_len  = 0U;

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

/* 让 1 字节接收始终处于启动状态。HAL_UART_Receive_IT() 只有在 huart1.RxState 为
   READY 时才接受新的接收请求，所以因硬件错误而结束的那次接收在这里由主循环
   重新启动，而不是一直死在那里。 */
static void sensor_rx_ensure_armed(void)
{
    if (huart1.RxState == HAL_UART_STATE_READY)
    {
        if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) == HAL_OK)
        {
            s_rearm++;
        }
    }
}

/* 中断上下文：把收到的字节入队并重新启动接收，别的什么都不做
   （不解析、不打印、不访问 Ch_senser[]）。 */
void sensor_rx_isr_handler(void)
{
    uint16_t next;

    next = (uint16_t)((s_fifo_head + 1U) & SENSOR_FIFO_MASK);
    if (next == s_fifo_tail)
    {
        s_fifo_ovf++;                       /* 缓冲满：丢掉最新到的这个字节 */
    }
    else
    {
        s_fifo[s_fifo_head] = s_rx_byte;
        s_fifo_head = next;
    }

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

/* USART1 硬件接收错误（溢出 / 帧错误 / 噪声）：HAL 会停掉中断接收；
   sensor_rx_task() 会在主循环里重新启动。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_rx_err++;
    }
    if (huart->Instance == USART2)
    {
    }
}

/* 丢弃分帧缓冲开头的 n 个字节 */
static void sensor_line_drop(uint16_t n)
{
    if (n >= s_line_len)
    {
        s_line_len = 0U;
        return;
    }
    memmove(&s_line[0], &s_line[n], (size_t)(s_line_len - n));
    s_line_len = (uint16_t)(s_line_len - n);
}

/* cmd 0x04 / 0x05 的应答处理，由 sensor_parse_frame() 分派。
   实现在本文件末尾。 */
static uint8_t binding_parse_response(const uint8_t *f, uint16_t total);

/* --------------------------------------------------------------------------
 * 校验一个完整帧并应用它
 *   0 ：校验通过，Ch_senser[0..19] 已更新
 *   1 ：其它命令的合法帧          -> 已消费，忽略
 *   2 ：cmd 0x03 但 LEN != 0x0051 -> 已消费，计入错误
 *   3 ：非法帧（帧尾或 SUM 不过）  -> 由调用方重新同步
 *   4 ：cmd 0x04/0x05 的 LEN/长度错 -> 整帧丢弃
 * 只有帧尾、长度、校验和全部通过之后才写 Ch_senser[]，所以被截断或损坏的帧
 * 绝不可能部分覆盖传感器表。
 * ------------------------------------------------------------------------ */
static uint8_t sensor_parse_frame(const uint8_t *f, uint16_t total)
{
    uint16_t len_field;
    uint16_t i;
    uint8_t  sum = 0U;

    if (f[total - 1U] != SENSOR_FRAME_TAIL)
    {
        return 3U;                                  /* 帧尾                   */
    }

    for (i = 4U; i < (uint16_t)(total - 2U); i++)   /* 从 CMD 到最后一个 DATA 字节 */
    {
        sum = (uint8_t)(sum + f[i]);
    }
    if (sum != f[total - 2U])
    {
        return 3U;                                  /* 校验和                 */
    }

    if (f[4] != SENSOR_CMD && f[4] != SENSOR_CMD_CYCLE)
    {
        /* cmd 0x04（解绑全部）、0x05（保存绑定）和 0x07（恢复出厂设置）
           复用完全相同的帧格式，所以在这里分派、共用同一条接收路径
           （cmd 0x08 已删除，冻结契约 §3.2） */
        if ((f[4] == BINDING_CMD_UNBIND) || (f[4] == BINDING_CMD_SAVE) ||
            (f[4] == BINDING_CMD_FACTORY))
        {
            return binding_parse_response(f, total);
        }
        return 1U;                                  /* 不是我们要的命令        */
    }

    /* cmd 0x03 的应答和 cmd 0x06 的主动上报逐字节完全相同
       （LEN = 0x0051、87 字节，载荷与校验和范围都一样），所以两者都在这里
       由这同一条代码路径校验并生效 —— 冻结契约 §2.1 明确禁止为 0x06
       再写第二个解析器 */
    len_field = (uint16_t)(((uint16_t)f[2] << 8) | f[3]);
    if ((len_field != SENSOR_RSP_LEN_FIELD) || (total != SENSOR_RSP_LEN))
    {
        return 2U;                                  /* 命令长度               */
    }

    /* 整帧校验通过：现在才可以安全地更新传感器表 */
    for (i = 0U; i < SENSOR_CH_NUM; i++)
    {
        const uint8_t *p = &f[5U + (i * 4U)];

        /* 类型字节原样携带 Data_tpye 的数值
           （T_None=0 ... T_KK=13），所以直接强转即可 */
        Ch_senser[i].tpye       = (Data_tpye)p[0];
        Ch_senser[i].CH_number  = (uint16_t)(i + 1U);  /* 从 1 开始编号，
                                                          与原有 printf 顺序一致 */
        Ch_senser[i].CH_data    = (uint16_t)(((uint16_t)p[1] << 8) | p[2]);
        Ch_senser[i].voltage    = (uint16_t)p[3];      /* 高字节保持 0           */
        if (Ch_senser[i].tpye != T_None)
        {
            Ch_senser[i].fail_cnt = 0U;                /* 该通道在线             */
        }
        Ch_senser[i].write_time = Rt_tick;             /* 供超时逻辑使用         */
		if (i == (SENSOR_CH_NUM - 1U)) { g_sensor_data_valid = 1U; }   /* V1_0_58：整帧 0x03/0x06 已落地 */
    }

    /* 认出了 cmd 0x03 的应答 -> 释放「请求在途」守卫（冻结契约 §3.1）。
       只有 0x03 才清它：0x06 是主动上报的一轮采集结果，
       跟我们发出的那条请求没有任何关系。 */
    if (f[4] == SENSOR_CMD)
    {
        if (s_req_pending != 0U)
        {
            s_req_reply_ok++;
            if ((s_sleep_request_active != 0U) && (sleep_mode == 2U))
            {
                /* Capture this exact response, not a later 0x06/cache update.
                   The unchanged CH584 protocol can still return timeout cache. */
                Lora_QueueCollectedReport(&f[5]);
            }
        }
        s_sleep_request_active = 0U;
        s_req_pending = 0U;
    }
    else
    {
        s_cycle_ok++;                                  /* 已生效的 0x06 上报   */
    }
    s_frame_ok++;
    g_has_binding = 1U;   /* Only sensor data frames imply an active binding here. */

    /* 冻结契约 §3.1：0x03 应答 / 0x06 主动上报成功更新 Ch_senser[] 之后，
       置一次数据刷新标志，让 CH584M 尽快重画一次。
       息屏期间 UI.lcd_open == 0，这一位只是攒着 —— 屏幕不会刷新。 */
    UI.dis_data_flag = 1U;

    return 0U;
}

/* --------------------------------------------------------------------------
 * sensor_rx_task ：主循环上下文。
 *   0. 0x03「请求在途」标志一旦过了 35 s 兜底时间就释放它
 *   1. 把中断里排队的字节全部搬进线性分帧缓冲
 *   2. 逐个取出完整帧、校验、填 Ch_senser[0..19]
 * 任何对不上的内容都逐字节丢掉，并从下一个 AA EE 重新开始搜索，
 * 所以一个损坏的帧不可能把它后面的帧藏起来。
 * ------------------------------------------------------------------------ */
static void sensor_rx_process(void);

void sensor_rx_task(void)
{
    static uint8_t active = 0U;

    /* Radio sleep may pump this task while a binding reply is being applied.
       Keep the shared framing buffer owned by the outer call until it returns.
       RX interrupts still enqueue bytes for the next main-loop invocation. */
    if (active != 0U) { return; }
    active = 1U;
    sensor_rx_process();
    active = 0U;
}

static void sensor_rx_process(void)
{
    uint16_t len_field;
    uint32_t total;
    uint8_t  rc;

    sensor_rx_ensure_armed();
    sensor_req_pending_expire();     /* 0x03 守卫的 35 s 兜底 */

    while (s_fifo_tail != s_fifo_head)
    {
        uint8_t b = s_fifo[s_fifo_tail];

        s_fifo_tail = (uint16_t)((s_fifo_tail + 1U) & SENSOR_FIFO_MASK);

        if (s_line_len >= SENSOR_LINE_SIZE)
        {
            sensor_line_drop(1U);        /* 87 字节的帧下不会发生 */
            s_fifo_ovf++;
        }
        s_line[s_line_len] = b;
        s_line_len++;
    }

    for (;;)
    {
        if (s_line_len < 2U)
        {
            return;
        }
        if ((s_line[0] != SENSOR_FRAME_HEAD0) || (s_line[1] != SENSOR_FRAME_HEAD1))
        {
            s_byte_drop++;               /* 按帧头重新同步 */
            sensor_line_drop(1U);
            continue;
        }
        if (s_line_len < 4U)
        {
            return;                      /* LEN 还没收全 */
        }

        len_field = (uint16_t)(((uint16_t)s_line[2] << 8) | s_line[3]);
        total     = (uint32_t)len_field + 6U;
        if ((len_field == 0U) || (total > SENSOR_MAX_FRAME))
        {
            s_frame_err++;
            sensor_line_drop(1U);        /* 帧头非法，重新同步 */
            continue;
        }
        if ((uint32_t)s_line_len < total)
        {
            return;                      /* 等剩下的字节到齐 */
        }

        rc = sensor_parse_frame(s_line, (uint16_t)total);
        if (rc == 0U)
        {
            sensor_line_drop((uint16_t)total);      /* 已校验并生效          */
        }
        else if (rc == 1U)
        {
            s_other_frame++;
            sensor_line_drop((uint16_t)total);      /* 已消费，不是我们的    */
        }
        else if (rc == 2U)
        {
            s_frame_err++;
            sensor_line_drop((uint16_t)total);      /* 整帧丢弃              */
        }
        else if (rc == 4U)
        {
            /* 0x04/0x05 应答的 LEN 不对：binding_parse_response()
               里已经计过数了，没有任何状态被应用 */
            sensor_line_drop((uint16_t)total);      /* 整帧丢弃              */
        }
        else
        {
            s_frame_err++;
            sensor_line_drop(1U);                   /* 重新同步              */
        }
    }
}

/* --------------------------------------------------------------------------
 * 只在某个错误计数发生变化时才打印计数（最高 1 Hz）。
 * 由 Time_1s() 调用；printf 走 USART2（调试口）。
 *   frame ：被丢弃的帧数（帧尾 / SUM / LEN）
 *   drop  ：寻找 AA EE 时跳过的字节数
 *   ovf   ：接收环形缓冲溢出；rxerr ：USART1 硬件接收错误
 *   tx    ：已发出的请求帧数；ok ：已生效的应答帧数
 *
 * cmd 0x04/0x05/0x07 的计数只由用户操作触发，所以每次触发后单独打印一行：
 *   cmd04/05/07 tx ：已发出的请求数；rsp ：被接受的应答数
 *   err            ：被丢弃的应答数；last ：最后一次被接受结果的 CMD/状态
 * cmd 0x03 守卫的计数也一并上报（冻结契约 §3.1）：
 *   pend ：有请求正在等它（被延迟的）应答；blk ：被守卫跳过的触发次数；
 *   tmo ：被 35 s 兜底释放掉的标志次数。
 * ------------------------------------------------------------------------ */
void sensor_rx_stats_report(void)
{
    uint32_t sum = s_frame_err + s_byte_drop + s_fifo_ovf + s_rx_err + s_tx_err
                 + s_bind_tx + s_bind_tx_err + s_bind_rsp_ok + s_bind_rsp_err
                 + s_cycle_ok
                 + s_req_blocked + s_req_timeout;

    if (sum == s_reported_sum)
    {
        return;
    }
    s_reported_sum = sum;

    printf("cmd03 err: frame=%u drop=%u ovf=%u rxerr=%u other=%u tx=%u ok=%u rearm=%u\r\n",
           (unsigned int)s_frame_err,  (unsigned int)s_byte_drop,
           (unsigned int)s_fifo_ovf,   (unsigned int)s_rx_err,
           (unsigned int)s_other_frame,(unsigned int)s_tx_cnt,
           (unsigned int)s_frame_ok,   (unsigned int)s_rearm);

//    /* 冻结契约 §3.1：pend=1 -> 还有一条请求在等应答，
//       blk = 被守卫跳过的 1 s 触发次数，tmo = 被 35 s 兜底释放的标志次数 */
    printf("cmd03 guard: pend=%u blk=%u tmo=%u (answer delayed by the CH584M)\r\n",
           (unsigned int)s_req_pending, (unsigned int)s_req_blocked,
           (unsigned int)s_req_timeout);

    printf("cmd06 report: ok=%u (shares the 0x03 parse/apply path)\r\n",
           (unsigned int)s_cycle_ok);

    printf("cmd04/05/07: tx=%u txerr=%u rsp=%u err=%u last=%02X/%02X\r\n",
           (unsigned int)s_bind_tx,      (unsigned int)s_bind_tx_err,
           (unsigned int)s_bind_rsp_ok,  (unsigned int)s_bind_rsp_err,
           (unsigned int)s_bind_last_cmd,(unsigned int)s_bind_last_status);
}
/* ================= TEST ONLY: USART1 frame parser self-test =================
 * 把合成字节流直接灌进接收 FIFO，再调用真实的 sensor_rx_task()，
 * 用于在真机上验证"错误流中仍能解析出正确指令"（乱码/截断/坏校验/坏LEN 后重新同步）。
 * 为不阻塞主循环、也不撞看门狗，本测试**每个主循环只跑一个用例**（step 式），
 * 由 main() 在主循环里反复调用，直到返回 0 表示全部跑完。
 * ========================================================================= */
/* 分块喂入：每块不超过环形缓冲剩余空间，且不超过 32 字节
   （保证 s_line_len 不会顶到 SENSOR_LINE_SIZE 而触发滑窗丢字节）。
   块与块之间调用一次 sensor_rx_task() 把数据搬走并分帧，
   模拟真实的"逐字节到达 + 频繁排空"节奏。 */
static void tp_push(const uint8_t *d, uint16_t n)
{
    uint16_t off = 0U;

    while (off < n)
    {
        uint16_t used = (uint16_t)((s_fifo_head - s_fifo_tail) & SENSOR_FIFO_MASK);
        uint16_t free = (uint16_t)(SENSOR_FIFO_SIZE - 1U - used);
        uint16_t chunk = (uint16_t)(n - off);
        uint16_t i;

        if (free == 0U)
        {
            sensor_rx_task();
            continue;
        }
        if (chunk > free)  { chunk = free; }
        if (chunk > 32U)   { chunk = 32U; }

        __disable_irq();
        for (i = 0U; i < chunk; i++)
        {
            s_fifo[s_fifo_head] = d[off + i];
            s_fifo_head = (uint16_t)((s_fifo_head + 1U) & SENSOR_FIFO_MASK);
        }
        __enable_irq();

        off = (uint16_t)(off + chunk);
        re_iwdg();
        sensor_rx_task();
    }
}

static uint16_t tp_build_rsp(uint8_t *f, uint8_t fill)
{
    uint16_t i;
    uint8_t  sum = 0U;
    f[0] = SENSOR_FRAME_HEAD0;
    f[1] = SENSOR_FRAME_HEAD1;
    f[2] = (uint8_t)(SENSOR_RSP_LEN_FIELD >> 8);
    f[3] = (uint8_t)(SENSOR_RSP_LEN_FIELD & 0xFFU);
    f[4] = 0x7FU;   /* 故意用非本机命令：只走分帧/帧尾/校验和，不产生任何副作用 */
    for (i = 5U; i < (uint16_t)(5U + SENSOR_RSP_DATA_LEN); i++) { f[i] = fill; }
    for (i = 4U; i < (uint16_t)(5U + SENSOR_RSP_DATA_LEN); i++) { sum = (uint8_t)(sum + f[i]); }
    f[5U + SENSOR_RSP_DATA_LEN] = sum;
    f[5U + SENSOR_RSP_DATA_LEN + 1U] = SENSOR_FRAME_TAIL;
    return SENSOR_RSP_LEN;
}

#define TP_CASES  7U

static uint8_t  s_tp_step = 0U;
static uint32_t s_tp_fail = 0U;
static uint32_t s_tp_ok0, s_tp_err0, s_tp_drp0;
static uint8_t  s_tp_mask = 0U;   /* 每个用例的通过位 */

/* 返回 1 = 还有用例没跑完，0 = 本轮自测已结束 */
/* 把自测结果以最短的一行重复打印出来，便于在容易丢字的日志通道上可靠读到 */
void usart_parser_dump(void)
{
    printf("[PT] m=%02X f=%u o=%u e=%u d=%u\r\n",
           (unsigned)s_tp_mask, (unsigned)s_tp_fail,
           (unsigned)(s_other_frame - s_tp_ok0),
           (unsigned)(s_frame_err - s_tp_err0),
           (unsigned)(s_byte_drop - s_tp_drp0));
}
uint8_t usart_parser_selftest_step(void)
{
    uint8_t  buf[SENSOR_RSP_LEN * 10U + 96U];
    uint8_t  f1[SENSOR_RSP_LEN];
    uint16_t m = 0U;
    uint16_t k;
    uint32_t before, got = 0U, expect = 0U;
    const char *name = "?";

    if (s_tp_step == 0U)
    {
        s_tp_ok0  = s_other_frame;
        s_tp_err0 = s_frame_err;
        s_tp_drp0 = s_byte_drop;
        s_tp_mask = 0U;
        printf("\r\n[PARSER-TEST] ==== begin ====\r\n");
    }
    if (s_tp_step >= TP_CASES)
    {
        re_iwdg();
        printf("[PT] DONE m=%02X f=%u o=%u e=%u d=%u\r\n",
               (unsigned)s_tp_mask, (unsigned)s_tp_fail,
               (unsigned)(s_other_frame - s_tp_ok0),
               (unsigned)(s_frame_err - s_tp_err0),
               (unsigned)(s_byte_drop - s_tp_drp0));
        re_iwdg();
        s_tp_step = 0U;
        s_tp_fail = 0U;
        return 0U;
    }

    tp_build_rsp(f1, 0x00U);
    switch (s_tp_step)
    {
      case 0U: name = "clean";          expect = 1U;
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               break;
      case 1U: name = "garbage+frame";  expect = 1U;
               for (k = 0U; k < 32U; k++) { buf[m++] = (uint8_t)(0x30U + k); }
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               break;
      case 2U: name = "badSUM+frame";   expect = 1U;
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               buf[85] = (uint8_t)(buf[85] ^ 0x5AU);
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               break;
      case 3U: name = "badLEN+frame";   expect = 1U;
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               buf[2] = 0xFFU; buf[3] = 0xFFU;
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               break;
      case 4U: name = "strayheads+frm"; expect = 1U;
               for (k = 0U; k < 8U; k++) { buf[m++] = 0xAAU; }
               buf[m++] = 0xAAU; buf[m++] = 0xEEU; buf[m++] = 0x00U; buf[m++] = 0x02U;
               buf[m++] = 0xAAU; buf[m++] = 0xEEU; buf[m++] = 0x00U; buf[m++] = 0x00U;
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               break;
      case 5U: name = "half-frame";     expect = 0U;
               for (k = 0U; k < 40U; k++) { buf[m++] = f1[k]; }
               break;
      case 6U: name = "rest+frame";     expect = 2U;
               for (k = 40U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               for (k = 0U; k < SENSOR_RSP_LEN; k++) { buf[m++] = f1[k]; }
               break;
      default: break;
    }

    before = s_other_frame;
    if (s_tp_step != 6U)
    {
        __disable_irq();
        s_fifo_tail = s_fifo_head;
        s_line_len  = 0U;
        __enable_irq();
    }
    tp_push(buf, m);          /* 分块喂入，内部会排空并分帧 */
    re_iwdg();
    re_iwdg();
    got = s_other_frame - before;
    if (got == expect) { s_tp_mask |= (uint8_t)(1U << s_tp_step); }
    else               { s_tp_fail++; }
    s_tp_step++;
    return 1U;
}

/* ==========================================================================
 * cmd 0x04 ：解绑全部绑定   /   cmd 0x05 ：把绑定保存到 Flash
 * --------------------------------------------------------------------------
 * 发送：Key_task()（软定时器，主循环）-> send_unbind_all()/send_save_binding()
 * 接收：与 cmd 0x03 完全同一条 USART1 中断链
 *        sensor_rx_isr_handler() -> sensor_rx_task() -> sensor_parse_frame()
 *      -> binding_parse_response()（按 CMD 分派）
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * build_binding_request ：组出 7 字节请求帧
 *   cmd 0x04 -> AA EE 00 01 04 04 0A   （解绑全部）
 *   cmd 0x05 -> AA EE 00 01 05 05 0A   （保存绑定）
 *   cmd 0x07 -> AA EE 00 01 07 07 0A   （恢复出厂设置）
 * 返回写入的字节数（BINDING_REQ_LEN）；参数非法时返回 0
 * （cmd 0x08 已按冻结契约 §3.2 删除）
 * ------------------------------------------------------------------------ */
uint16_t build_binding_request(uint8_t *buf, uint8_t cmd)
{
    uint16_t idx = 0U;
    uint16_t i;
    uint8_t  sum = 0U;

    if ((buf == NULL) ||
        ((cmd != BINDING_CMD_UNBIND) && (cmd != BINDING_CMD_SAVE) &&
         (cmd != BINDING_CMD_FACTORY)))
    {
        return 0U;
    }

    buf[idx++] = SENSOR_FRAME_HEAD0;                        /* 第 0 字节：AA              */
    buf[idx++] = SENSOR_FRAME_HEAD1;                        /* 第 1 字节：EE              */
    buf[idx++] = (uint8_t)(BINDING_REQ_LEN_FIELD >> 8);     /* 第 2 字节：LEN 高字节 = 0x00 */
    buf[idx++] = (uint8_t)(BINDING_REQ_LEN_FIELD & 0xFFU);  /* 第 3 字节：LEN 低字节 = 0x01 */
    buf[idx++] = cmd;                                       /* 第 4 字节：CMD 04/05/07    */

    /* SUM = CMD + DATA（取低 8 位）；这些请求不带数据字节 */
    for (i = 4U; i < idx; i++)
    {
        sum = (uint8_t)(sum + buf[i]);
    }
    buf[idx++] = sum;                                       /* 第 5 字节：SUM = CMD       */
    buf[idx++] = SENSOR_FRAME_TAIL;                         /* 第 6 字节：0A              */

    return idx;                                             /* 共 7 字节                  */
}

/* --------------------------------------------------------------------------
 * binding_request_send ：阻塞式发送一条 0x04/0x05/0x07 请求。
 * 用的是与 0x03 请求、CH584M 显示帧同一个 huart1，
 * 并且由同样跑在主循环里的 Key_task() 调用，
 * 所以两个发送方永远不会重叠。
 * ------------------------------------------------------------------------ */
static void binding_request_send(uint8_t cmd)
{
    uint8_t  req[BINDING_REQ_LEN];
    uint16_t len;

    len = build_binding_request(req, cmd);
    if (len != BINDING_REQ_LEN)
    {
        s_bind_tx_err++;
        return;
    }

    if (HAL_UART_Transmit(&huart1, req, len, BINDING_TX_TIMEOUT) == HAL_OK)
    {
        s_bind_tx++;
        printf("cmd%02X tx: ", (unsigned int)cmd);
        hex_p(req, (int)len);
    }
    else
    {
        s_bind_tx_err++;
        printf("cmd%02X tx failed\r\n", (unsigned int)cmd);
    }
}

/* cmd 0x04 ：清空全部绑定（RAM + Flash 记录），7 字节请求 */
void send_unbind_all(void)
{
    binding_request_send(BINDING_CMD_UNBIND);
}

/* cmd 0x05 ：把当前绑定表持久化到 Flash，7 字节请求 */
void send_save_binding(void)
{
    binding_request_send(BINDING_CMD_SAVE);
}

/* cmd 0x07 ：把绑定存储恢复出厂，7 字节请求。
   冻结契约 §1.4 / 2.2：CH584M 会执行 observer_clear_all_bindings() +
   binding_store_clear()；应答状态码说明原来是否有绑定。 */
void send_factory_reset(void)
{
    binding_request_send(BINDING_CMD_FACTORY);
}

/* cmd 0x08「读取绑定统计」（send_bind_stat_request()）以及它的 1 秒轮询任务
   （bind_stat_poll_task()）已按冻结契约 §3.2 删除 ——
   两端都不再存在这条命令。 */

/* --------------------------------------------------------------------------
 * binding_parse_response ：校验并应用一个 0x04 / 0x05 / 0x07 应答帧。
 * 帧尾和 SUM（从 CMD 到最后一个 DATA 字节）在调用本函数之前已由
 * sensor_parse_frame() 校验。
 *   0 ：接受（帧已消费，结果已保存/打印）
 *   4 ：LEN 或总长不对 -> 调用方丢弃整帧，唯一的副作用是错误计数
 *       （不碰任何状态）
 * cmd 0x08 分支以及「每次 0x04 / 0x05 / 0x07 之后重新读一次统计」这个触发
 * 已按冻结契约 §3.2 删除。
 * ------------------------------------------------------------------------ */
static uint8_t binding_parse_response(const uint8_t *f, uint16_t total)
{
    uint16_t len_field;
    uint8_t  cmd    = f[4];
    uint8_t  status = f[5];

    len_field = (uint16_t)(((uint16_t)f[2] << 8) | f[3]);

    if ((len_field != BINDING_RSP_LEN_FIELD) || (total != BINDING_RSP_LEN))
    {
        s_bind_rsp_err++;
        printf("cmd%02X rsp LEN err: len=%u total=%u\r\n",
               (unsigned int)cmd, (unsigned int)len_field, (unsigned int)total);
        return 4U;
    }

    /* 整帧已校验通过：现在才可以安全地记录并上报结果 */
    s_bind_last_cmd    = cmd;
    s_bind_last_status = status;
    s_bind_rsp_ok++;

    if (cmd == BINDING_CMD_UNBIND)
    {
        switch (status)
        {
            case BINDING_ST_UNBIND_OK:
                printf("cmd04 rsp: unbind OK (00), all bindings and the flash record cleared\r\n");
                g_has_binding = 0U;   /* 全部解绑 -> 确定没有绑定 */
                break;
            case BINDING_ST_UNBIND_NONE:
                printf("cmd04 rsp: unbind OK (01), nothing was bound\r\n");
                g_has_binding = 0U;   /* 本来就没绑定 */
                break;
            default:
                printf("cmd04 rsp: unknown status %02X\r\n", (unsigned int)status);
                break;
        }
    }
    else if (cmd == BINDING_CMD_SAVE)
    {
        switch (status)
        {
            case BINDING_ST_SAVE_OK:
                printf("cmd05 rsp: save OK (00), binding table written and verified\r\n");
                g_has_binding = 1U;   /* 保存成功 -> 有绑定 */
                break;
            case BINDING_ST_SAVE_NONE:
                printf("cmd05 rsp: nothing bound, nothing saved (01)\r\n");
                break;
            case BINDING_ST_SAVE_FAIL:
                printf("cmd05 rsp: save FAILED (02), flash write or read back check failed\r\n");
                break;
            default:
                printf("cmd05 rsp: unknown status %02X\r\n", (unsigned int)status);
                break;
        }
    }
    else /* BINDING_CMD_FACTORY：恢复出厂设置 */
    {
        switch (status)
        {
            case BINDING_ST_FACT_OK:
                printf("cmd07 rsp: factory reset OK (00), binding store was cleared\r\n");
                g_has_binding = 0U;   /* 恢复出厂 -> 绑定已清 */
                break;
            case BINDING_ST_FACT_NONE:
                printf("cmd07 rsp: factory reset OK (01), nothing was bound\r\n");
                break;
            default:
                printf("cmd07 rsp: unknown status %02X\r\n", (unsigned int)status);
                break;
        }
    }


    /* ★★★ V1_0_56 现场修复（另一侧的防御性修复）：0x04/0x05/0x07 是「绑定流程的终点」，
       现场 bug（绑定后返回主页 LoRa 彻底不发）就出现在这之后。
       本函数本身只改 g_has_binding / 计数（不动任何 LoRa 门控），但绑定期间用户在
       设置页停留很久，0x0A/0x0D 事件留下的 send_flag / token_flag / Lora_Tx_state /
       Lora_Rx_state 残值会一直被带进主页 —— 这里显式归位一次（幂等，不改用户参数，
       也不依赖复位）。 */
    if (((cmd == BINDING_CMD_UNBIND) &&
         ((status == BINDING_ST_UNBIND_OK) || (status == BINDING_ST_UNBIND_NONE))) ||
        ((cmd == BINDING_CMD_SAVE) &&
         ((status == BINDING_ST_SAVE_OK) || (status == BINDING_ST_SAVE_NONE))) ||
        ((cmd == BINDING_CMD_FACTORY) &&
         ((status == BINDING_ST_FACT_OK) || (status == BINDING_ST_FACT_NONE))))
    {
        /* A queued snapshot belongs to the old binding set. RF/UI recovery alone
           keeps it; a confirmed binding result explicitly invalidates it. */
        Lora_CancelCollectedReport();
        s_sleep_request_active = 0U;
        s_req_pending = 0U;
    }
    Lora_HomeEntry_ReArm();

    return 0U;
}


/* ==========================================================================
 * 息屏读数据子系统（V1_0_2 修正：**非阻塞状态机**）
 * --------------------------------------------------------------------------
 * 每次息屏读取窗口里的时序（由 sleep_read_service() 分步推进，绝不阻塞）：
 *   ① lcd_power_on()            PB4 = 供电，CH584M 上电重启
 *                               （它的绑定表从自己的 Flash 恢复）
 *   ② 等 SLP_BOOT_WAIT_MS       CH584M 启动 + 首轮采集（期间也把
 *                               CH584M 主动发来的 0x06 收掉）
 *   ③ sensor_request_send()     发 0x03（在途守卫生效）
 *   ④ 等 ≤ SLP_READ_TIMEOUT_MS  收到应答 -> Ch_senser[0..19] 已更新；
 *                               超时 -> 记一次失败计数，照常断电
 *   ⑤ lcd_power_off()           PB4 = 断电，CH584M 彻底断电
 *   ⑥ 下一周期沿原计划每 SLP_READ_PERIOD_SEC 秒推进，读取耗时不累积
 *
 * ★ 与旧实现的根本区别：**主循环不会被占住**。
 *   旧实现的 sleep_read_idle()（Sleep.c）在息屏后 while(sleep_mode==2)
 *   死循环，里面调用旧 sleep_read_task()，而后者阻塞 3 s + 最多 35 s。
 *   结果是息屏期间 Lora_task()/Time_check_task()/Light_check() 全部停摆
 *   （LoRa 停摆、光唤醒失效），而那几十秒只靠手动喂狗续命 -> 漏喂就是
 *   看门狗复位（≈2.08 s）。现在每一步只花几十毫秒就返回主循环，
 *   LoRa / 光检测 / 按键 / 喂狗全都照常。
 *
 * ★ 全程不发 0x01 帧：息屏期间 UI.lcd_open == 0，只有它会让
 *   soft_timer_loop_async() 跑起来 —— 屏幕不刷新（内存屏保持原画面）。
 *
 * ★ 为什么要在这里重配 USART1：息屏进 STOP 之前 ALL_IO_CLOSE() 已经把
 *   USART1 整个 DeInit 掉了（省电），读取窗口里必须重新初始化并重启
 *   1 字节中断接收 —— sensor_rx_start() 会顺手丢掉上一轮的残留字节，
 *   读完再 DeInit 回去。
 *
 * ★ 为什么读取窗口内不许进 STOP：USART1 的时钟源是 HSI，STOP 下停振，
 *   一进 STOP 就必然丢掉 CH584M 应答的字节。窗口里置 g_slp_keep_awake=1，
 *   Sleep.c 的 sleep_timer()/sleep_stop_ms() 据此不进 STOP，只做
 *   **有界**短等待（中断照常响应，LoRa CAD 的 DIO 中断语义不变）。
 * ========================================================================== */

extern uint8_t sleep_mode;              /* 定义在 Sleep.c：0 正常 / 2 息屏 / 3 唤醒 */

/* 1 = 息屏读数据窗口：Sleep.c 的 sleep_timer()/sleep_stop_ms() 据此不进 STOP。
   USART1 的时钟源是 HSI（main.c 的 RCC_PERIPHCLK_USART1 = RCC_USART1CLKSOURCE_HSI），
   STOP 下 HSI 停振 -> 一进 STOP 就会丢掉 CH584M 应答的字节。
   这个窗口每 SLP_READ_PERIOD_SEC 才出现一次、每次几十秒，代价可接受。 */
uint8_t g_slp_keep_awake = 0U;

/* ---- 息屏读数据状态机 ------------------------------------------------ */
typedef enum
{
    SLP_ST_IDLE = 0,    /* 等下一个读取周期                */
    SLP_ST_BOOT,        /* 已给 CH584M 上电，等它启动 + 首轮采集 */
    SLP_ST_WAIT_RSP,    /* 已发 0x03，等应答               */
    SLP_ST_PWR_OFF      /* 收尾：断电 + 关 USART1 + 复位状态 */
} slp_read_state_e;

static slp_read_state_e s_sleep_state = SLP_ST_IDLE;
static uint32_t s_cad_at_on = 0U, s_rx_at_on = 0U, s_light_at_on = 0U;   /* 诊断: 读取窗口起点快照 */
static uint32_t s_sleep_state_tick;     /* 进入当前状态时的 Rt_tick          */
static uint32_t s_sleep_rsp0;
static uint32_t s_sleep_tmo0;           /* 进入 WAIT_RSP 时的 s_req_timeout 快照 */
static uint32_t s_sleep_pwr_cycles;     /* 已执行的"上电->读->断电"轮数         */
static uint32_t s_sleep_read_ok;        /* 收到 0x03 应答、数据已更新的轮数      */
static uint32_t s_sleep_read_tmo;       /* 应答超时 / 请求没发出去 / 被唤醒打断的轮数 */
static uint32_t s_sleep_next_read_tick;
static uint8_t  s_sleep_sched_valid;   /* ★ V1_0_46：读取计划是否已排过（跨"息屏重入"保留） */ /* 记录"下次读取时刻"（Rt_tick，100 ms）*/

/* Advance from the planned deadline, not from slow collection/RF completion.
   Missed periods are coalesced; never issue a burst of catch-up requests. */
static void sleep_read_advance_schedule(void)
{
    uint32_t period = (uint32_t)SLP_READ_PERIOD_SEC * 10U;
    if ((int32_t)(Rt_tick - s_sleep_next_read_tick) >= 0)
    {
        uint32_t late = (uint32_t)(Rt_tick - s_sleep_next_read_tick);
        s_sleep_next_read_tick += (late / period + 1U) * period;
    }
    else
    {
        /* STOP-chunk fallback fired before the primary tick deadline. */
        s_sleep_next_read_tick = Rt_tick + period;
    }
}

/* 把"毫秒"换成"Rt_tick 节拍数"（向上取整；Rt_tick = 100 ms） */
#define SLP_MS_TO_TICKS(ms)  ((((uint32_t)(ms)) + 99U) / 100U)

/* --------------------------------------------------------------------------
 * ★ 备用计时基准：息屏 STOP 的段数。
 *   主基准是 Rt_tick（LPTIM1 的 100 ms 节拍）——LPTIM1 的时钟源是
 *   LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC，STOP 下 APB 停振时会自动切到 LPOSC，
 *   理论上 STOP 期间照常计数（整个工程的 LoRa 计时也依赖这一点）。
 *   但这一条没有硬件可实测，所以再留一条退路：Sleep.c 每真睡完一段
 *   SLP_SLEEP_CHUNK_MS 就调一次 sleep_read_chunk_tick()，段数到了也照样触发
 *   一次读取。两条基准谁先到算谁，**绝不会出现"息屏后再也不读数据"**。
 *   注意 0.410 那个常数是按 LSI 估的而本工程 RTC 走 LSE，所以这条退路的
 *   实际周期会偏长约 19%（600 s 名义 -> 约 720 s）。
 * ------------------------------------------------------------------------ */
#define SLP_CHUNKS_PER_PERIOD \
    ((((uint32_t)SLP_READ_PERIOD_SEC * 1000U) + ((uint32_t)SLP_SLEEP_CHUNK_MS - 1U)) \
     / (uint32_t)SLP_SLEEP_CHUNK_MS)

static uint32_t s_sleep_chunks;         /* 距上次读取已经睡过的 STOP 段数 */

/* 由 Sleep.c 的 sleep_check() case 2 在"真睡完一段"后调用（备用基准） */
void sleep_read_chunk_tick(void)
{
    if (s_sleep_chunks < 0xFFFFFFFFU)
    {
        s_sleep_chunks++;
    }
}

/* --------------------------------------------------------------------------
 * lcd_power_on / lcd_power_off ：全工程唯一的 PB4 操作点
 *   PB4 = "屏幕供电控制" = CH584M（显示 + 蓝牙 + 采集）的整机供电。
 *   极性只在 usart.h 的两个宏里定义（冻结契约 §7.1）：on = SET，
 *   off = RESET。要改极性只改那两个宏，别在这里写死电平。
 * ------------------------------------------------------------------------ */
/* V1_0_28: measure how long the CH584M (BLE+screen) is actually powered each cycle.
 * lcd_power_off() -> lcd_power_on() is the OFF time; lcd_power_on() -> lcd_power_off()
 * is the POWERED time. During the read window the MCU is kept awake, so HAL_GetTick
 * advances normally and this measurement is exact in ms. */
volatile uint32_t g_ble_pwr_on_ms  = 0U;   /* accumulated POWERED milliseconds */
volatile uint32_t g_ble_pwr_last_ms = 0U;  /* V1_0_30: POWERED ms of the LAST cycle */
volatile uint32_t g_ble_t_boot_ms = 0U;  /* V1_0_31: power-on -> request sent */
volatile uint32_t g_ble_t_rsp_ms  = 0U;  /* V1_0_31: request sent -> response done */
static   uint32_t s_ble_t_on = 0U, s_ble_t_req = 0U;
volatile uint32_t g_ble_pwr_cycles = 0U;   /* number of power-on cycles       */
static   uint32_t s_ble_pwr_t0     = 0U;   /* HAL_GetTick at the last power-on */

void lcd_power_on(void)
{
    s_ble_pwr_t0 = HAL_GetTick(); g_ble_pwr_cycles++;   /* V1_0_28 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, LCD_PWR_ON_LEVEL);
}

void lcd_power_off(void)
{
    g_ble_pwr_last_ms = (uint32_t)(HAL_GetTick() - s_ble_pwr_t0); g_ble_pwr_on_ms += g_ble_pwr_last_ms;   /* V1_0_30 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, LCD_PWR_OFF_LEVEL);
}

/* --------------------------------------------------------------------------
 * sleep_read_begin ：息屏序列开始时由 Sleep.c sleep_check() case 1 调用。
 *   1) 复位状态机并清掉可能残留的"0x03 请求在途"标志 —— 上一轮读数据
 *      如果是在等应答时被触摸唤醒打断的，这个标志会留着，进门先收拾干净；
 *   2) 记下"下次读取时刻"：息屏后先等 SLP_FIRST_READ_DELAY_SEC 秒
 *      （默认与 SLP_READ_PERIOD_SEC 相同 = 10 分钟），后续沿原计划
 *      每 SLP_READ_PERIOD_SEC 秒推进；
 *   3) 清 g_slp_keep_awake（此时还没进读取窗口，允许进 STOP）。
 * ------------------------------------------------------------------------ */
void sleep_read_begin(void)
{
    s_sleep_request_active = 0U;
    s_sleep_state          = SLP_ST_IDLE;
    s_req_pending          = 0U;
    g_slp_keep_awake       = 0U;
    s_sleep_chunks         = 0U;
    /* ★★★ V1_0_46 修正：这里原来是**无条件**重排下一次读取时刻
       （s_sleep_next_read_tick = 现在 + 600 秒）。而 sleep_read_begin() 在
       **每次进入息屏**时都会被调用，于是只要设备因光唤醒 / 按键唤醒等原因
       醒过一次，600 秒就重新计时 —— 唤醒比读取周期更频繁时，
       **周期读取永远不会发生**。

       实测证据：修复前两台设备共 74 次"600 秒息屏"，完成的读取周期 **0 次**；
       把周期改成 10s / 60s 时则每次都能读到（因为唤醒间隔大于周期）。
       现场确认：A 机在 05:58:12 进入息屏（下次读取应为 06:08:12），
       但 06:05:40 被光敏唤醒、06:05:47 重新进入息屏 —— 计划被推到 06:15:47，
       那次本该在 06:08:12 发生的读取就此丢失。

       现在改为：**只在尚未排过计划时**（冷启动）排一次；
       之后无论中间被唤醒多少次、重进息屏多少次，计划时刻都不变。
       若唤醒时计划已到期，重新进入息屏后会立刻触发读取（IDLE 判据是"到点即读"）。
       计划本身在每次读取完成 / 被判定跳过时都会顺延一个周期，因此不会只读一次。 */
    if (s_sleep_sched_valid == 0U)
    {
        s_sleep_next_read_tick = Rt_tick + ((uint32_t)SLP_FIRST_READ_DELAY_SEC * 10U);
        s_sleep_sched_valid    = 1U;
    }

    /* ★ 打印里带上排程信息，便于核对"重进息屏不会推后计划" */
    printf("sleep read: begin, period=%us first=%us boot_wait=%ums timeout=%ums sched=%u next=%lu\r\n",
           (unsigned int)SLP_READ_PERIOD_SEC,
           (unsigned int)SLP_FIRST_READ_DELAY_SEC,
           (unsigned int)SLP_BOOT_WAIT_MS,
           (unsigned int)SLP_READ_TIMEOUT_MS,
           (unsigned int)s_sleep_sched_valid,
           (unsigned long)s_sleep_next_read_tick);
}

/* --------------------------------------------------------------------------
 * sleep_read_service ：息屏读数据状态机。**非阻塞**，由 Sleep.c 的
 *   sleep_check() case 2 在每次主循环里调用一次，只推进一步就返回。
 *
 * ★ 为什么必须非阻塞（V1_0_2 修正的核心）：
 *   旧实现把"上电 -> 等 3 s -> 发 0x03 -> 等最多 35 s -> 断电"整整
 *   几十秒堵在 sleep_read_task() 里，主循环一步都走不动 -> 息屏期间
 *   Lora_task() / Time_check_task() / Light_check() 全部停摆（LoRa 停摆、
 *   光唤醒失效），而 IWDG（≈2.08 s）只靠手动喂狗续命 —— 漏一次就复位。
 *
 * ★ 超时一律用 Rt_tick（LPTIM1 的 100 ms 节拍，STOP 下仍在走），
 *   绝不用 HAL_GetTick()：SysTick 在 STOP 下不计数，会永远等不到超时。
 *
 * ★ 被触摸/光唤醒（sleep_mode != 2）时的收尾约定与旧实现一致：
 *   - 立刻把状态机复位、放开 g_slp_keep_awake（否则 LoRa 以后再也进不了 STOP）；
 *   - 不主动 lcd_power_off()，因为 CH584M 已经上电，马上要交给 ALL_init()
 *     走正常唤醒流程（它会重新上电并重发 0x01 帧点亮屏幕）。
 * ------------------------------------------------------------------------ */
void sleep_read_service(void)
{
    uint32_t elapsed;

    /* 已经被唤醒：收尾退出，让 sleep_check() case 3 去跑 ALL_init() */
    if (sleep_mode != 2U)
    {
        uint8_t was_reading = 0U;
        if ((s_sleep_state == SLP_ST_BOOT) || (s_sleep_state == SLP_ST_WAIT_RSP))
        {
            s_sleep_read_tmo++;             /* 本轮被唤醒打断 */
            was_reading = 1U;
        }
        s_sleep_state    = SLP_ST_IDLE;
        s_sleep_request_active = 0U;
        s_req_pending    = 0U;
        g_slp_keep_awake = 0U;
        s_sleep_chunks   = 0U;
        /* ★★★ V1_0_46：这里原来**无条件**把下次读取再推后一整个周期。
           于是"被唤醒"这件事本身也会推迟周期读取 —— 与 sleep_read_begin() 里
           那处 bug 叠加，只要唤醒比读取周期更频繁，读取就永远不会发生。
           现在只在**本轮读取已在进行中**（上电/等待应答）被打断时才顺延一个周期
           （避免刚被吵醒就立刻又去开屏）；若本来是 IDLE（还没到点），
           则**保持原计划不动**，到点照读。 */
        if (was_reading != 0U)
        {
            sleep_read_advance_schedule();
            s_sleep_sched_valid    = 1U;
        }
        return;
    }

    switch (s_sleep_state)
    {
        case SLP_ST_IDLE:
            /* 到点判据用**带符号差**：s_sleep_next_read_tick 是"下次读取的时刻"，
               没到就继续等。用无符号比较会因为"减出来是负数被当成巨大值"
               而把周期算错（第一轮会变成 首延迟 + 一个周期）。
               两个时刻之差远小于 2^31 拍（100ms/拍 -> 约 6.8 年），不会绕回。 */
            if (((int32_t)(Rt_tick - s_sleep_next_read_tick) < 0) &&
                (s_sleep_chunks < SLP_CHUNKS_PER_PERIOD))
            {
                return;                     /* 周期没到：什么都不做，交给主循环 */
            }

            /* 第 11 轮：CH584M 确定没有绑定设备 -> 本轮不进读取窗口。
               0x06 与 0x03 应答只在「有绑定」时才有内容，没绑定时发了也没人回，
               STM32 只会白等一个超时。直接推迟一个完整周期，电源/串口一律不碰。 */
            if (g_has_binding == 0U)
            {
                sleep_read_advance_schedule();
                s_sleep_chunks         = 0U;
                return;
            }

            /* ① CH584M 上电（它的绑定表从自己的 Flash 恢复） */
            s_sleep_pwr_cycles++;
            { extern volatile uint32_t g_cad_cnt, g_rx_hit_cnt, g_light_chk_cnt;
              s_cad_at_on = g_cad_cnt; s_rx_at_on = g_rx_hit_cnt; s_light_at_on = g_light_chk_cnt; }
            lcd_power_on();

            /* 息屏进 STOP 前 ALL_IO_CLOSE() 已把 USART1 整个 DeInit 掉了，
               这里重新初始化 + 重启 1 字节中断接收（顺手丢掉残留字节） */
            MX_USART1_UART_Init();
            sensor_rx_start();

            g_slp_keep_awake   = 1U;        /* 读取窗口内禁止进 STOP */
            s_sleep_state      = SLP_ST_BOOT;
            s_ble_t_on = HAL_GetTick();   /* V1_0_31 */
            s_sleep_state_tick = Rt_tick;
            s_sleep_chunks     = 0U;
            break;

        case SLP_ST_BOOT:
            /* ② 期间 CH584M 会主动上报 0x06，正好在这里收掉 */
            sensor_rx_task();

            if ((uint32_t)(Rt_tick - s_sleep_state_tick) <
                SLP_MS_TO_TICKS(SLP_BOOT_WAIT_MS))
            {
                return;                     /* 还没等够，下一轮主循环再来看 */
            }

            /* ③ 发 0x03（在途守卫生效：有请求在途时不会重复发） */
            /* ★★★ V1_0_33：发请求之前**再清一次 RX 缓冲**，把"CH584M 上电瞬态"彻底隔离。              * ------------------------------------------------------------------              * 现场数据：通宵 7 小时里 cmd03 err 报过 drop=23 / rxerr=10，全部集中在              *   CH584M 刚上电的那一瞬间：PB4 上电时它的 TX 脚还在初始化，STM32 的              *   USART1 会收到噪声字节（帧错误/噪声标志 -> rxerr，垃圾字节 -> drop）。              * 影响判定：              *   · 页面刷新 = STM32->CH584M 的 **TX**（0x01 帧），与本项**无关**；              *   · 取数据：噪声发生在"发请求之前"，且帧带长度+校验和、解析器按帧头重同步，              *     坏数据不会写进 Ch_senser[]（实测 62/62 轮 ok、tmo=0，零失败）。              * 但存在理论风险：若噪声恰好紧贴 0x03 应答前面，会干扰分帧（有超时/重试兜底，              *   最坏是这一轮读到旧值）。              * 修法：boot 等待结束、请求发出**之前**调用 sensor_rx_start()：              *   清空环形缓冲/分帧缓冲并重新武装 1 字节中断 —— 上电瞬态被彻底丢在窗口之外，              *   从此 drop/rxerr 只会在"没有有效帧在途"时计数，**不影响任何一帧有效数据**。              * 说明：物理层 bit 级零丢失无法承诺（电气/EMI 问题），但**协议层有效数据零丢失**              *   由此成立。 */             sensor_rx_start();
            s_sleep_rsp0 = s_req_reply_ok;
            s_sleep_request_active = 1U;
            sensor_request_send();
            if (s_req_pending == 0U)
            {
                s_sleep_request_active = 0U;
                printf("[COLLECT] request failed tick=%lu\r\n", (unsigned long)Rt_tick);
                s_sleep_read_tmo++;         /* 请求没发出去，本轮算失败 */
                s_sleep_state      = SLP_ST_PWR_OFF;
                g_ble_t_rsp_ms = (uint32_t)(HAL_GetTick() - s_ble_t_req);   /* V1_0_31 */
                s_sleep_state_tick = Rt_tick;
                break;
            }
            s_sleep_tmo0       = s_req_timeout;
            s_sleep_state      = SLP_ST_WAIT_RSP;
            s_ble_t_req = HAL_GetTick(); g_ble_t_boot_ms = (uint32_t)(s_ble_t_req - s_ble_t_on);   /* V1_0_31 */
            s_sleep_state_tick = Rt_tick;
            break;

        case SLP_ST_WAIT_RSP:
            /* ④ 等应答，最多 SLP_READ_TIMEOUT_MS（> CH584M 的 30 s） */
            sensor_rx_task();               /* 分帧 + 校验 + 填 Ch_senser[0..19] */

            if (s_req_pending == 0U)
            {
                if ((s_req_timeout == s_sleep_tmo0) && (s_req_reply_ok != s_sleep_rsp0))
                {
                    s_sleep_read_ok++;      /* 真的是 0x03 应答把它清掉的 */
                }
                else
                {
                    s_sleep_read_tmo++;     /* Guard expiry is not a response. */
                    printf("[COLLECT] no response tick=%lu\r\n", (unsigned long)Rt_tick);
                }
                s_sleep_state      = SLP_ST_PWR_OFF;
                g_ble_t_rsp_ms = (uint32_t)(HAL_GetTick() - s_ble_t_req);   /* V1_0_31 */
                s_sleep_state_tick = Rt_tick;
                break;
            }

            elapsed = (uint32_t)(Rt_tick - s_sleep_state_tick);
            if (elapsed >= SLP_MS_TO_TICKS(SLP_READ_TIMEOUT_MS))
            {
                s_sleep_request_active = 0U;
                s_sleep_read_tmo++;         /* 超时 */
                printf("[COLLECT] timeout tick=%lu\r\n", (unsigned long)Rt_tick);
                s_req_pending = 0U;         /* 不许把在途标志带出本轮 */
                s_sleep_state = SLP_ST_PWR_OFF;
                g_ble_t_rsp_ms = (uint32_t)(HAL_GetTick() - s_ble_t_req);   /* V1_0_31 */
                s_sleep_state_tick = Rt_tick;
            }
            break;

        case SLP_ST_PWR_OFF:
        default:
            /* ⑤ 断电 + 关串口省电（与 ALL_IO_CLOSE() 同一对调用） */
            sensor_rx_task();               /* 落袋最后一帧 */
            HAL_UART_MspDeInit(&huart1);
            HAL_UART_DeInit(&huart1);
            lcd_power_off();

            s_sleep_request_active = 0U;
            g_slp_keep_awake       = 0U;    /* 窗口结束，恢复允许进 STOP */
            s_sleep_state          = SLP_ST_IDLE;
            s_sleep_chunks         = 0U;
            sleep_read_advance_schedule();

            /* 息屏期间 Time_1s() 不跑，所以息屏的计数只能在这里打 */
            printf("sleep read: cycle=%u ok=%u tmo=%u next=%u period=%us\r\n",
                   (unsigned int)s_sleep_pwr_cycles, (unsigned int)s_sleep_read_ok,
                   (unsigned int)s_sleep_read_tmo,
                   (unsigned int)s_sleep_next_read_tick,
                   (unsigned int)SLP_READ_PERIOD_SEC);
    /* V1_0_30: CH584M (BLE+screen) real powered time per read cycle, readable from the
     * serial log every 600s -- avoids the J-Link reset-on-connect problem. */
    printf("ch584 pwr: last=%ums avg=%ums cycles=%u boot=%ums rsp=%ums\r\n",
           (unsigned int)g_ble_pwr_last_ms,
           (unsigned int)((g_ble_pwr_cycles != 0U) ? (g_ble_pwr_on_ms / g_ble_pwr_cycles) : 0U),
           (unsigned int)g_ble_pwr_cycles,
           (unsigned int)g_ble_t_boot_ms,
           (unsigned int)g_ble_t_rsp_ms);
             { extern volatile uint32_t g_cad_cnt, g_rx_hit_cnt, g_light_chk_cnt; extern uint8_t Lora_Rx_state; extern Flash_Set Setup; extern ALL_UI UI;
               printf("read-window: cad=+%u rx=+%u light=+%u abs=%u lorast=%u set=%u on=%u\r\n",
                      (unsigned)(g_cad_cnt - s_cad_at_on), (unsigned)(g_rx_hit_cnt - s_rx_at_on),
                      (unsigned)(g_light_chk_cnt - s_light_at_on), (unsigned)g_cad_cnt,
                      (unsigned)Lora_Rx_state, (unsigned)UI.Setting_flag, (unsigned)Setup.On_Off_flag); }
            break;
    }
}
