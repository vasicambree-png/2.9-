#include "LoraPro.h"
#include "ui.h"
#include "string.h"
#include "usart.h"
#include "usart.h"
#include "New_Key.h"
#include "L_B_ADC.h"
#include "Sleep.h"
#include "soft_timer.h"   /* ★ V1_0_8：等待收包时要跑 soft_timer_loop_async()（按键/1s 任务） */


#ifndef LORA_FRAME_LOG
#define LORA_FRAME_LOG   0
#endif
#if LORA_FRAME_LOG
#define LORA_FRAME_PRINT(...)     printf(__VA_ARGS__)
#define LORA_FRAME_HEX(buf,len)   hex_p((buf),(len))
#else
#define LORA_FRAME_PRINT(...)     do { } while (0)
#define LORA_FRAME_HEX(buf,len)   do { } while (0)
#endif
#include "iwdg.h"
#include "lptim.h"
#include "gpio.h"

#include "Sx126x_Driver.h"
//#define Lora_Send_out_time 500  //单位ms
Time_Node  New_Node={0};
Time_Node Auxiliary_node={0};
CRfNet RfNet;
CRfRepeater repeater = {0}; // 静态分配并初始化所有成员为0
CRfRepeater *TestRepeater = &repeater; // 指针指向有效内存

extern S_LoRaConfig G_LoRaConfig;
uint8_t toke_flag=0;
uint8_t send_flag=0;
extern volatile uint8_t g_sensor_data_valid;   /* V1_0_58：见 usart.c */
uint16_t send_time1=0;
//uint8_t new_send_addr=0;

//CSx1268       Sx1268;
//CRfStatus     RfStatus;
//CUpLoadStatus UpLoadStatus;

L_TxStatus    Lora_Tx_state;

//0 ：空闲；1：写完后状态；2：写成功状态；3：读状态; 4: 读成功状态  5:正在cad状态； 6:CAD检测到数据 ; 7：cad超时  ;8 :RX超时
volatile uint8_t  Lora_Rx_state=0;

lora_d Lo_data={0};
u16 Sys_Tick=0;
uint32_t Time_send_data=0;
uint32_t Sx_tick=0;
Channel_C Ch_senser[21] = {0};
extern Flash_Set  Setup;
uint16_t Voltage=0;
extern ALL_UI UI;

Lora_time Lora_work_time={0};
uint8_t End_data_flag=0;

Woke_state Lora_Woke;
Woke_state Old_Lora_Woke;
extern volatile uint32_t Rt_tick;
uint32_t lp_Sx_tick=0;
uint32_t time_new=0;
uint8_t  t_Num=0;
extern LPTIM_HandleTypeDef   LptimHandle;
uint16_t now_host_addr=0;
extern uint16_t Vdd_value;
extern uint16_t Vdd_value_temp;
extern volatile uint8_t sleep_mode;
extern  void SystemClock_Config(void);
uint8_t Frist_work_flag=0;
uint8_t Old_Frist_work_flag=0;
uint8_t cad_Init_flag=0;
uint8_t old_cad_Init_flag=0;
uint8_t Tx11=0;
//同步时间
Time_c rx_lora={0};
/* Time_RX_lora（见下）：CAD 检到前导之后等整包收完的上限，唯一使用者是
   check_work_time()。窗口必须 > 剩余前导 + 头 + 载荷；换射频参数
   （SF/BW/前导符号数）后必须按 T_sym = 2^SF/BW 重算。 */
/* 射频模式切换闸门：进 STOP 前记 100ms 节拍，出 STOP 后若一拍都没跨过
   （被 LPTIM/按键/光 EXTI 立刻叫醒），说明 CAD 可能还没做完，先补一个
   有界等待再改射频模式，否则会卡在等 BUSY（手册 Table 8-2，取 5ms 上限）。 */
#define LORA_RADIO_SETTLE_MS   5U

extern volatile uint8_t  g_stop_woke_early;  /* Sleep.c：上一觉是否没睡满 100ms 节拍 */
extern volatile uint32_t g_stop_early_cnt;   /* Sleep.c：累计"早醒"次数（诊断）     */
volatile uint32_t g_radio_settle_cnt = 0;    /* 本文件：实际补等的次数（诊断）      */
#define Time_RX_lora 500 /*ms。V1_0_57：发射前导已统一为 2000 符号 = 128ms（SF5/BW500），
                            整包最坏 = 128(前导) + 6(载荷) + ramp ≈ 140ms；CAD 命中后
                            还要等剩余前导+载荷，所以 500ms 仍是"很宽但不必改"的值
                            （多出来的时间只是多收一会儿，不影响时序正确性）。
                            窗口内每一轮都泵串口/按键，不再死等，见 CAD_work）*/
volatile uint8_t wait_rx_lora_flag=0;

volatile  uint8_t Lora_Rx_flag=0;
extern volatile uint8_t sleep_mode;

#define Time_out_20   12000  //单位100ms
#define Time_out_data   18000  //单位100ms
#define Time_data_task   600  //单位100ms
/* 周期上传：6000 个 100ms 节拍 = 10 分钟，正常/测试模式一致。
   台号错峰仅用于开机首轮，偏移限制在一个周期内；后续每轮完整等待 10 分钟。 */
#define Time_Lora_task   (SLP_READ_PERIOD_SEC * 10U)
#define LORA_STAGGER_SLOT_100MS   40U
#define LORA_STAGGER_PHASE   (((uint32_t)Setup.Host_number * LORA_STAGGER_SLOT_100MS) % Time_Lora_task)

/* ★★★ V1_0_56（现场修复）：LoRa 两个时间常数 —— 必须定义在 Lora_send()/Lora_reve()
   **之前**，因为 Tx_oneself 与 Lora_reve 都要用到。 */
#define LORA_RX_STUCK_TICKS    30U   /* 30 × 100ms = 3s：任何一次 CAD/收发都远小于它 */
#define LORA_BOOT_QUIET_TICKS  20U   /* 20 × 100ms = 2s：复位后不主动抢发的静默窗口 */
#define LORA_FIRST_DATA_WAIT_TICKS  600U  /* V1_0_58：复位后最多等 60s 的第一帧完整采集（有界，防卡死） */
/* 周期结束、应答及恢复路径均以当前 Rt_tick 重新计时，不扣减台号相位，避免缩短周期或连续发送。 */

/* V1_0_57：发射参数与"确认窗口"集中定义（原来 4 处硬编码，语义各说各话） */
#define LORA_PREAMBLE_STATION      12U    /* 目标=分站(122)：分站常听，短前导即可        */
#define LORA_PREAMBLE_NEIGHBOR   2000U    /* 目标=邻居节点。SF5/BW500 下 2000 符号 = 128ms，
                                             而邻居的 CAD 采样周期是 100ms（见 CAD_SLOW_TICKS=1）
                                             -> 128/100 = 1.28 倍余量，与旧设备 rx_state==0
                                             分支的 2000 完全一致：够覆盖，又不像 6000 那样
                                             白占 384ms 空中（6000 在旧机实际用不到）。
                                             ★ 本值按用户对旧设备的实测结论定为 2000。       */
#define LORA_TX_CONFIRM_TICKS       8U    /* 发完等应答：8 拍 = 800ms。按 2000 符号重算：
                                             本机发完 <= 128ms(前导) + 6ms(载荷) + ramp
                                             ~= 140ms；邻居 CAD 命中延迟 <= 100ms；命中后
                                             剩余前导+载荷 ~= 35ms；邻居转发上游(再 128ms)
                                             + 回程 ~= 300~450ms -> 合计 ~= 500~700ms。
                                             5 拍(500ms) 偏紧，8 拍有余量。硬约束：一轮
                                             9 次 x (800+140)ms ~= 8.5s < LORA_TX_STUCK_TICKS
                                             (12s)，仍然有界。                                */
#define LORA_TX_STUCK_TICKS       120U    /* 发射状态兜底：任一发射态停留 > 12s 强收尾。
                                             正常最坏 ~8.5s，12s 足够宽，绝不误伤。            */
#define LORA_CAD_BUSY_MAX          30U    /* 连续 30 次 CAD 都报"信道忙" -> 强发一次          */
#define LORA_STUCK_MS_AWAKE      60000U   /* 看门狗第二时基（HAL_GetTick，只计清醒 ms）：
                                             万一 LPTIM1 停走导致 Rt_tick 冻住，主循环还在
                                             喂狗，单靠 Rt_tick 的看门狗会变成"喂着狗死等"。
                                             正常一次收/发等待清醒时间 <= 12s，取 60s 上限。   */

uint32_t check_tick =0;

uint8_t rx_state=0;
uint32_t  lrx_tick=0;

uint8_t  dis_Rssi[4]={0xFF,0xFF,0xFF};
uint8_t send_adrr_cnt=0;
/* 0 = 当前发送地址还没重发过；1 = 已重发一次，再超时就换下一个地址。 */
static uint8_t s_same_addr_retry = 0U;
/* V1_0_57（活动性审查）：新增的 6 个状态量 + 3 个诊断计数，全部只在本文件内用。
   目的只有一条：让"发不出去"这件事**一定**有一个有限的出口。 */
static uint8_t  s_boot_stagger_done = 0U;   /* 开机首包错峰只做一次                     */
static uint32_t s_tx_wait_tick      = 0U;   /* 发射等待专用起算点（不与 RfRxTick 混用）  */
static uint8_t  s_cad_busy_cnt      = 0U;   /* 连续"CAD 判忙"次数 */
static uint32_t s_rx_wait_rt0 = 0U;   /* 收整包等待的 Rt_tick 起点：给 while 加硬件时基硬上限（V1_0_58） */
static uint8_t  s_force_rf_exit     = 0U;   /* 需在下一次非重入上下文里补做射频退出      */
static uint8_t  s_tx_state_last     = 0U;   /* 发射状态看门狗：上一拍的 Lora_Tx_state    */
static uint32_t s_tx_state_tick     = 0U;   /* 发射状态看门狗：进入当前状态的时刻        */
volatile uint32_t g_cad_force_cnt     = 0U; /* 诊断：连续 CAD 忙 -> 强发次数             */
volatile uint32_t g_lora_tx_stuck_cnt = 0U; /* 诊断：发射状态被强制收尾次数              */
volatile uint32_t g_force_rf_exit_cnt = 0U; /* 诊断：回主页后补做射频退出次数            */
static uint8_t  s_rx_state_last      = 0U;   /* 上一拍的射频状态（Lora_reve 看门狗）      */
static uint32_t s_rx_state_tick      = 0U;   /* 进入当前射频状态的时刻（Rt_tick）         */
static uint32_t s_tx_state_hms      = 0U;   /* 发射状态看门狗的 HAL_GetTick 基线（与 LPTIM1 无关）*/
static uint32_t s_rx_state_hms      = 0U;   /* 射频状态看门狗的 HAL_GetTick 基线         */
static uint32_t s_early_cnt_seen    = 0U;   /* 已处理过的"STOP 早醒"次数（一次性锁存）    */
volatile uint32_t g_radio_norm_cnt  = 0U;   /* 诊断：已观察到新的早醒计数的次数            */
/* 自愈：连续 LORA_HEAL_FAIL_ROUNDS 轮全失败，或 LORA_HEAL_NO_RX_TICKS 拍
   （Rt_tick=100ms）没收到有效数据 -> 回归默认三地址表，避免被动态地址带死。 */
extern void GetNewsend_addr(void);
static uint8_t  s_all_fail_cnt  = 0U;
static uint32_t s_last_valid_ms = 0U;
#define LORA_HEAL_FAIL_ROUNDS   10U
#define LORA_HEAL_NO_RX_TICKS   6000U   /* 6000 拍 = 10 分钟；要 20 分钟改成 12000U */

/* 现场诊断：每 LORA_DIAG_MS 输出一行机器可解析状态（[BOOT]/[DIAG]/[UP]/[SELFHEAL]）。
   逐帧日志用 LORA_FRAME_LOG 打开（每条阻塞约 26ms，会改变时序）。 */
#define LORA_DIAG_MS            30000U
volatile uint32_t g_diag_cycles = 0U;   /* 上报轮次（每轮开始加一） */
volatile uint32_t g_diag_allfail = 0U;  /* 整轮三地址全失败次数 */
volatile uint32_t g_diag_lasthearslot = 255U; /* 最近一次"听到的更小上游" */
volatile uint8_t Ble_flag_data=0;
uint32_t TickLeav=0;
uint32_t Leav_tick=0;

/* Main-loop-owned collection jobs. Keep queued and in-flight payloads separate:
   an old acknowledgement must not consume a newer collection. No wire change. */
#define LORA_REPORT_DATA_LEN       80U
#define LORA_REPORT_RETRY_TICKS   300U  /* 30 seconds between failed rounds */
static uint8_t s_report_pending = 0U;
static uint32_t s_report_seq = 0U;
static uint32_t s_report_tx_seq = 0U;
static uint8_t s_report_data[LORA_REPORT_DATA_LEN];
static uint8_t s_report_tx_data[LORA_REPORT_DATA_LEN];
static uint8_t s_self_retry_wait = 0U;
static uint32_t s_self_retry_tick = 0U;
static uint32_t s_self_retry_seq = 0U;
static uint8_t s_forward_turn = 0U;
static uint32_t s_forward_ready_tick = 0U;
static uint8_t s_report_host = 0U;
static uint8_t s_report_partition = 0U;
static uint32_t s_report_tick = 0U;
static uint32_t s_report_tx_tick = 0U;
/* Per-collection scheduling; never rebased by RF ACK/failure or UI re-arm. */
#define LORA_REPORT_RELEASE_TICKS (((uint32_t)SLP_BOOT_WAIT_MS + (uint32_t)SLP_READ_TIMEOUT_MS + 99U) / 100U)
static uint32_t s_report_release_tick = 0U;
static uint32_t s_report_token_seq = 0U;
static uint32_t s_report_token_tick = 0U;
static uint16_t s_report_token_delay = 0U;
static uint8_t s_report_tx_host = 0U;
static uint8_t s_report_tx_partition = 0U;

void Lora_CancelCollectedReport(void)
{
    s_report_pending = 0U;
    s_report_tx_seq = 0U;
    s_self_retry_wait = 0U;
    s_self_retry_seq = 0U;
    s_report_token_seq = 0U;
}

uint32_t Lora_SleepCollectionDelayTicks(void)
{
    uint32_t first = (uint32_t)SLP_FIRST_READ_DELAY_SEC * 10U;
    if (first == 0U) { return 0U; }
    return first - (LORA_STAGGER_PHASE % first);
}

void Lora_QueueCollectedReport(const uint8_t *channels, uint32_t planned_tick)
{
    uint8_t i;
    uint16_t at;
    if (channels == NULL) { return; }
    for (i = 0U; i < 20U; i++)
    {
        at = (uint16_t)i * 4U;
        s_report_data[at] = channels[at];
        s_report_data[at + 1U] = (channels[at] == 0U) ? 0xFCU : channels[at + 1U];
        s_report_data[at + 2U] = (channels[at] == 0U) ? 0xFCU : channels[at + 2U];
        s_report_data[at + 3U] = (uint8_t)((uint16_t)channels[at + 3U] / 2U * 10U);
    }
    s_report_seq++;
    if (s_report_seq == 0U) { s_report_seq = 1U; }
    s_report_host = Setup.Host_number;
    s_report_partition = Setup.partition;
    s_report_tick = Rt_tick;
    s_report_release_tick = planned_tick + LORA_REPORT_RELEASE_TICKS;
    s_report_token_seq = 0U;
    s_report_pending = 1U;
    printf("[UP] READY seq=%lu tick=%lu\r\n",
           (unsigned long)s_report_seq, (unsigned long)Rt_tick);
}

static void lora_self_report_complete(void)
{
    if (s_report_tx_seq != 0U)
    {
        if (s_report_tx_seq == s_report_seq) { s_report_pending = 0U; }
        printf("[UP] DONE seq=%lu pending=%u tick=%lu\r\n",
               (unsigned long)s_report_tx_seq, (unsigned)s_report_pending,
               (unsigned long)Rt_tick);
    }
    s_report_tx_seq = 0U;
    s_self_retry_wait = 0U;
}

static void lora_self_report_retry(void)
{
    /* Only the failed generation has already passed the scheduling gate. */
    s_self_retry_seq = s_report_tx_seq;
    s_report_tx_seq = 0U;
    s_self_retry_wait = 1U;
    s_self_retry_tick = Rt_tick;
}

static uint8_t lora_self_report_due(void)
{
    if ((s_report_pending != 0U) &&
        ((s_report_host != Setup.Host_number) || (s_report_partition != Setup.partition)))
    {
        Lora_CancelCollectedReport();
    }
    if ((Setup.Host_number == 0U) || (Setup.On_Off_flag == 0U)) { return 0U; }
    if (s_self_retry_wait != 0U)
    {
        if (TickDiff_100ms(s_self_retry_tick) < LORA_REPORT_RETRY_TICKS) { return 0U; }
    }
    if (s_report_pending != 0U)
    {
        if ((sleep_mode == 2U) && (g_slp_keep_awake != 0U)) { return 0U; }
        /* Preserve the previous awake snapshot behavior and shared RF cooldown. */
        if (sleep_mode != 2U) { return 1U; }
        if ((s_self_retry_wait != 0U) && (s_self_retry_seq == s_report_seq)) { return 1U; }
        if ((int32_t)(Rt_tick - s_report_release_tick) >= 0) { return 1U; }
        /* Only a token received for this queued generation can advance its slot.
           The first token is latched; repeats cannot postpone the fixed deadline. */
        return ((s_report_token_seq == s_report_seq) &&
                (TickDiff_100ms(s_report_token_tick) > s_report_token_delay)) ? 1U : 0U;
    }
    else if (sleep_mode == 2U) { return 0U; }
    if (s_boot_stagger_done == 0U)
    {
        s_boot_stagger_done = 1U;
        Lo_data.RfTick = Rt_tick - LORA_STAGGER_PHASE;
    }
    if ((g_sensor_data_valid == 0U) && ((uint32_t)Rt_tick < LORA_FIRST_DATA_WAIT_TICKS))
    {
        return 0U;
    }
    if ((send_time1 == 0U) || (send_time1 > 600U)) { send_time1 = 300U; }
    return (((TickDiff_100ms(Lo_data.RfTick) >= Time_Lora_task) && (send_flag == 0U)) ||
            ((send_flag == 1U) && (TickDiff_100ms(Sx_tick) > send_time1)) ||
            ((s_report_pending == 0U) && (Ble_flag_data == 1U))) ? 1U : 0U;
}
static void lora_report_token_on_rx(uint16_t origin)
{
    uint16_t gap;
    if ((sleep_mode != 2U) || (s_report_pending == 0U) ||
        (s_report_token_seq == s_report_seq) ||
        (s_report_host != Setup.Host_number) || (s_report_partition != Setup.partition) ||
        (origin <= Setup.Host_number)) { return; }
    gap = (uint16_t)(origin - Setup.Host_number);
    if (gap > 3U) { return; }
    s_report_token_seq = s_report_seq;
    s_report_token_tick = Rt_tick;
    s_report_token_delay = (uint16_t)(gap * 100U);
}

/* 校验和：从 0x55 起累加 len 字节（组包/校验共用同一算法）。 */
uint8_t CalcCheckSum(u8 *buf,u16 len)
{
  uint8_t Sum = 0x55;
  uint16_t i;
  for(i=0;i<len;i++)
  {
    Sum += buf[i];
  }
  return Sum;
}
 //组包校验和
/* 把 buf[0..len-1] 的校验和写到 buf[len]，组包时用。 */
uint8_t SetCheckSum(uint8_t *buf,uint16_t len)
{
	uint8_t result=0;
	result =CalcCheckSum(buf,len);
	*(buf+len) =result;

//  result= CalcCheckSum(buf,len);
//	printf("set_result ==%d\r\n",result);
	return result;
}

uint8_t CheckCheckSum(uint8_t *buf,uint16_t len)
{
  if(CalcCheckSum(buf,len - 1) == buf[len - 1])
    return 1;
  return 0;
}
//数据组包
uint16_t last_data=0;
/* 组包：按 Lora_Tx_state 填 0x0A 帧（自发 20 通道 / 转发 84 字节），
   固定 94 字节正文 + 校验和，返回 95（发送长度）。 */
uint16_t Load_Packaging(uint8_t *buf,uint16_t host_addr)
{
	uint16_t k_num=0xFCFC;
//	uint32_t Time=0;
	uint16_t offset=0;
	switch(Lora_Tx_state)
	{
		case Tx_oneself://自发
		  buf[8]= Setup.Host_number;
	    buf[9]= 0x0A;//CMD
			buf[10]=dis_Rssi[send_adrr_cnt];
			buf[11]=Vdd_value/2;
			buf[12]=Setup.partition;//当前设置的分站号
			buf[13]=Setup.Host_number;//当前设置的主机号
		
            if (s_report_tx_seq != 0U)
            {
                if ((s_report_tx_host != Setup.Host_number) ||
                    (s_report_tx_partition != Setup.partition))
                {
                    if (s_report_tx_seq == s_report_seq) { s_report_pending = 0U; }
                    s_report_tx_seq = 0U;
                    Lora_Tx_state = Tx_None;
                    return 0U; /* Never put an old snapshot under a new identity. */
                }
                memcpy(&buf[14], s_report_tx_data, LORA_REPORT_DATA_LEN);
                /* Preserve the existing 30-minute channel expiry for snapshots. */
                if (TickDiff_100ms(s_report_tx_tick) > Time_out_data)
                {
                    for (offset = 14U; offset < 94U; offset += 4U)
                    {
                        if (buf[offset] != 0U)
                        {
                            buf[offset + 1U] = 0xFDU;
                            buf[offset + 2U] = 0xFDU;
                            buf[offset + 3U] = 0U;
                        }
                    }
                }
                break;
            }
			offset=14;
			for(int i=0;i<20;i++)
			{
				buf[offset++]=Ch_senser[i].tpye;
				if(Ch_senser[i].tpye==0)
				{
					buf[offset++]=(k_num>>8)&0xFF;
				  buf[offset++]=k_num&0xFF;
				}
				else
				{

				  buf[offset++]=(Ch_senser[i].CH_data>>8)&0xFF;
				  buf[offset++]=Ch_senser[i].CH_data&0xFF;
				}
				buf[offset++]=Ch_senser[i].voltage/2*10;
			}
		
			break;
		case Tx_Forwarding:

	  	buf[8]= Setup.Host_number;
	    buf[9]=0x0A;//CMD
		  
		  memcpy(&buf[10],Lo_data.ForwardBuf,84);
		  Tx11=buf[13];
			break;
		default:
			break;
	}
	buf[0]=Setup.partition; //发往的分站号
	buf[1]=host_addr;//发往的主机号
	buf[2]=Setup.partition;//当前设置的分站号
	buf[3]=Setup.Host_number;//当前设置的主机号
	now_host_addr=host_addr;
	buf[94]=SetCheckSum(buf,94);
	offset=95;
	//printf("offset==%d\r\n",offset);
	return offset;
}



int compare_arrays( unsigned char *arr1,  unsigned char *arr2, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (arr1[i] != arr2[i]) {
            return 0; // 发现不一致立即返回
        }
    }
    return 1; // 全部一致
}


/* ==========================================================================
 * V1_0_57（活动性/liveness 审查）三个"逃生"小工具
 * --------------------------------------------------------------------------
 * 审查目标：证明"任何一个包最终一定能发出去"，也就是 Lora_send()/Lora_reve()
 * 里所有可能**永久停住**的点都必须配一个"有界逃生"。三条统一规则：
 *   1) 上限只用 Rt_tick(100ms 拍) 或计数，绝不用 while 死等、绝不用延时掩盖；
 *   2) 幂等（同一拍重复调用等价于一次）、可重入（不关中断、不改协议参数）；
 *   3) 不依赖复位（硬件复位路径保持原样）。
 * ========================================================================== */

/* (1) lora_force_rf_exit_once：补做"回主页时因为正在收发而没敢发射频命令"这件事。
   原来 Lora_HomeEntry_ReArm() 遇到 Lora_Rx_state==1/3 只清软件状态、不动射频 ->
   射频可能仍停在 RX/CAD，而软件已经认为它是 Sleep。这里在**非重入**的上下文
   （下一拍的 Data_cad/CAD_work）里补一次 SX126X_WOR_Execute(Sx1268_Sleep)：
   它内部 = EnsureStandbyRC + ClearIrqStatus + Sleep，本身有界（每条 SPI 命令的
   BUSY 等待上限 SX126X_BUSY_TIMEOUT_MS = 100ms）。幂等：清掉标志后什么都不做。 */
static void lora_force_rf_exit_once(void)
{
    if (s_force_rf_exit == 0U) { return; }
    s_force_rf_exit = 0U;
    g_force_rf_exit_cnt++;
    (void)SX126X_WOR_Execute(Sx1268_Sleep);
    Lora_Rx_state = 0U;
}

/* (2) lora_rx_window_ticks：收窗口长度（100ms 拍）。
   发完等应答时必须 >= LORA_TX_CONFIRM_TICKS，否则"确认窗口 800ms"的后 400ms
   里根本没有接收机在听 —— 这正是 V1_0_17 那个"确认窗口 5 拍"永远收不到应答的
   结构性原因（收窗口 400ms 先关，确认窗口 500ms 还在等）。 */
static uint8_t lora_rx_window_ticks(void)
{
    if ((Lora_Tx_state == Tx_wait_oneself) || (Lora_Tx_state == Tx_wait_Forwarding))
    {
        return (uint8_t)LORA_TX_CONFIRM_TICKS;   /* 发完等应答：800ms */
    }
    return 4U;                                   /* 空闲收包：400ms   */
}

/* (3) lora_rf_housekeeping：射频状态机的"清道夫"，每一拍都跑，**与 UI 无关**。
   为什么必须有：Lora_reve() 是 switch(Lora_Rx_state)，只有 0/3/4 三个 case
   有处理器；设置页里 case 0 又被 UI.Setting_flag 整段跳过 -> 状态 1/2/5/7/8
   **没有任何消费者**，只能等 3s 看门狗。这里把 1/2/5/7/8 就地收尾，把最坏
   停留时间从 3s 压到 1 拍；完全不发帧、不做 CAD、不碰 3/4/6：
     · 3 = 收窗口，由 case 3 的定时关闭处理（要保持接收机开着）；
     · 4 = 已收到包，由 case 4 的 RxData_check 处理（**绝不能在这里吃掉**）；
     · 6 = CAD 报"有活动"，必须留给 CAD_work()/Data_cad() 去开长接收。 */
static void lora_rf_housekeeping(void)
{
    switch (Lora_Rx_state)
    {
        case 0:  case 3:  case 4:  case 6:
            break;
        default:                              /* 1/2/5/7/8：进睡眠收尾 */
            (void)SX126X_WOR_Execute(Sx1268_Sleep);
            Lora_Rx_state = 0U;
            break;
    }
}

/* STOP early wake is normal for TxDone/RxDone/CAD interrupts. Consume only
 * the diagnostic counter; do not reconfigure RF, discard a pending packet or
 * restart ACK/backoff timers. Existing BUSY/RX/TX timeout recovery is retained. */
static void lora_early_wake_normalize(void)
{
    if (g_stop_early_cnt == s_early_cnt_seen) { return; }
    s_early_cnt_seen = g_stop_early_cnt;
    g_radio_norm_cnt++;
}

u8 Data_cad(void)
{
	
//    printf("Data_cad\r\n");
	  lora_force_rf_exit_once();   /* V1_0_57: 回主页时若射频正忙，在这里补做一次有界退出 */
	  lora_early_wake_normalize(); /* Observe early wake without disturbing RF/ACK state. */
	  CAD_MODE(); //开CAD	
 
	  work_sleep();
		if(Lora_Rx_state== 6)   //检查到信息了
		{
//			printf("LLLL\r\n");
		  SX126X_WOR_Execute(Sx1268_Sleep);
			Lora_Rx_state=0;
//					printf("VVVVVVVVVV\r\n");
		  return 0;
		}
		else
		{
			return 1;
		}
	


}
extern volatile uint8_t reset_flag;
/* 1/2/3 号台的特殊下一跳修正：它们只发分站(122)。 */
void send_next_lora(void)
{
	if(Setup.Host_number==1||Setup.Host_number==2||Setup.Host_number==3)
	{
		Setup.transmit_number[0] =122;
	}
}
/* LoRa 发送状态机：Tx_None -> Tx_oneself/Tx_Forwarding -> Tx_wait_* 。
   发送前先做 CAD/LBT，三个备用地址依次尝试，失败可按每地址重发一次。 */
void Lora_send(void)
{
	uint16_t for_send_num=0;
	switch(Lora_Tx_state)
	{
        case Tx_None:
        {
            uint8_t self_due = lora_self_report_due();
            if ((Lo_data.Forward_flag != 0U) &&
                ((int32_t)(Rt_tick - s_forward_ready_tick) >= 0) &&
                ((self_due == 0U) || (s_forward_turn != 0U)))
            {
                Lora_Tx_state = Tx_Forwarding;
                s_forward_turn = 0U;
            }
            else if (self_due != 0U)
            {
                s_self_retry_wait = 0U;
                s_self_retry_seq = 0U;
                s_report_tx_seq = (s_report_pending != 0U) ? s_report_seq : 0U;
                if (s_report_tx_seq != 0U)
                {
                    memcpy(s_report_tx_data, s_report_data, LORA_REPORT_DATA_LEN);
                    s_report_tx_tick = s_report_tick;
                    s_report_tx_host = s_report_host;
                    s_report_tx_partition = s_report_partition;
                }
                Lora_Tx_state = Tx_oneself;
                s_forward_turn = 1U;
            }
            else { break; }
            TickLeav = 0U;
            Leav_tick = Rt_tick;
            send_adrr_cnt = 0U;
            s_same_addr_retry = 0U;
            break;
        }
		case Tx_oneself: //自发
                if ((sleep_mode == 2U) && (s_report_tx_seq == 0U))
                {
                    Lora_Tx_state = Tx_None;
                    break; /* An awake/token round cannot bypass sleep collection. */
                }
				if((TickDiff_100ms(Leav_tick)<TickLeav))  	//信道上有数据
				{				
					  // TickLeav --;
						 break;
				}
				/* ★★★ V1_0_56 现场修复（绑定后返回主页 LoRa 彻底不发）：改前这里是
				   `if(reset_flag==2||reset_flag==1) { Lora_Tx_state=Tx_None; break; }`。
				   reset_flag 在整个上电周期里**不再变化**（POR=1 / IWDG=2），而 Tx_None
				   的周期判据（Lo_data.RfTick 只有真正发出去才会更新）一直成立，于是主循环
				   每一拍都在 Tx_None <-> Tx_oneself 之间来回跳 —— **一个包也发不出去**，
				   并且只有按硬件复位（NRST -> reset_flag=5）才恢复，与现场"只有硬复位能救"
				   完全一致。现在改成"只在刚复位后的 LORA_BOOT_QUIET_TICKS(2s) 内让道"：
				   保留"上电瞬间别抢信道"的原意，但**绝不可能是永久锁死**，时间一到自动
				   恢复（幂等、不依赖复位）。 */
				if((uint32_t)Rt_tick < LORA_BOOT_QUIET_TICKS)
				{
					Lora_Tx_state=Tx_None;
					break;
				}
			  /* V1_0_57：CAD 结果的**有界化**。
				   Data_cad() 的语义是 fail-open：只有"确实检测到前导(状态 6)"才返回 0，
				   "没结果 / 被误唤醒 / CAD 结果丢失"都返回 1（照发），所以它本身不会
				   永久挡住发送。但如果信道被长期判忙（邻居密集、DIO2 毛刺、CAD 结果
				   丢失），原来会一直走 return 0 分支 -> 永远不发。
				   这里给"连续判忙"加一个上限：超过 LORA_CAD_BUSY_MAX(30) 次就允许
				   强发一次，保证"每轮必有一包"。幂等，且不影响正常 LBT 语义： */
				{
				    uint8_t _cad_idle = Data_cad();            /* 1 = 信道空闲，可发 */
				    if(_cad_idle != 0U)
				    {
				        s_cad_busy_cnt = 0U;
				    }
				    else
				    {
				        if(s_cad_busy_cnt < 250U) { s_cad_busy_cnt++; }
				        if(s_cad_busy_cnt >= LORA_CAD_BUSY_MAX)
				        {
				            s_cad_busy_cnt = 0U;
				            _cad_idle      = 1U;                /* fail-open：连续忙 -> 强发 */
				            g_cad_force_cnt++;
				        }
				    }
				    if(_cad_idle == 0U) { break; }              /* 信道忙：下一拍再试 */
				}
				{
		   
					if(Lo_data.Failure>9)
					{
						Lo_data.Failure=0; 
						s_all_fail_cnt = 0U; 
						s_last_valid_ms = Rt_tick;
					}
					memset(Lo_data.Tx_buf,0,100);
					send_next_lora();
					
					{ 
						uint8_t _d=Setup.transmit_number[send_adrr_cnt];
					if(!(_d==STATION || (_d!=0u && _d!=NONEADDR && Setup.Host_number>3u && _d<Setup.Host_number))) { Setup.transmit_number[send_adrr_cnt]=STATION; } }   /* V1_0_25: 防回传闸门：只允许发更小台号或分站；1/2/3 只能发分站 */
					Lo_data.Tx_data_len= Load_Packaging(Lo_data.Tx_buf,Setup.transmit_number[send_adrr_cnt] ); //组包
                    if (Lo_data.Tx_data_len == 0U) { break; }
					G_LoRaConfig.PayloadLength=	Lo_data.Tx_data_len; //发送长度
					if(Setup.transmit_number[send_adrr_cnt]==122)
					{
						for_send_num=12;
					}
					else
					{
						if(rx_state==0)
						{
				
					    for_send_num=LORA_PREAMBLE_NEIGHBOR;   /* V1_0_57: 前导统一由 LORA_PREAMBLE_NEIGHBOR 给（2000 符号/128ms） */
						}
						else{
						  for_send_num=LORA_PREAMBLE_NEIGHBOR;   /* 同上：2000 符号/128ms */
						}
					}
        // printf("self send addr == %d;for_send_num ==%d\r\n",Setup.transmit_number[send_adrr_cnt],for_send_num);
					LSD_RF_SendPacket(Lo_data.Tx_buf,Lo_data.Tx_data_len,for_send_num);                          //发送数据
//					HAL_IWDG_Refresh(&hiwdg) ;
					re_iwdg();

       //  printf("5555555\r\n");
					SX126X_WOR_Exit();  //进入读监测;

					Lora_Tx_state=Tx_wait_oneself;
					Lo_data.RfRxTick=Rt_tick;
					s_tx_wait_tick=Rt_tick;   /* V1_0_57: dedicated wait start tick */
				}
				/* V1_0_57：原来的 else{ printf("5555555") } 空分支已合并进上面的 CAD 有界化块
				   （Data_cad()!=1 时上面直接 break，与原来的 else 空分支等价）。 */
			break;

		case Tx_wait_oneself://自发等待成功
	
				if(TickDiff_100ms(s_tx_wait_tick)>LORA_TX_CONFIRM_TICKS) /* V1_0_57: 5 ticks -> 8 ticks(800ms), and use a DEDICATED start point: the old one was Lo_data.RfRxTick, which is refreshed by the RX window (SX126X_WOR_Exit) and by 0x0A/0x0C receptions -> the timeout could never elapse ("wait for ack forever") */
				{
//					printf("Tx_wait_oneself\r\n");
					Lo_data.Failure++;
					
					TickLeav  = 2U + (((Rt_tick >> 1) + (uint32_t)Setup.Host_number * 7U) % 4U);   /* V1_0_24 P4: 4 slots (200..500ms), paired with the 4s stagger slot */
					Leav_tick = Rt_tick;   /* V1_0_24 P4: backoff gate needs a start point */
				
					if (s_same_addr_retry < 2U)
					{
					    s_same_addr_retry++;
					    Lo_data.RfRxTick  = Rt_tick;   /* 重新起算这一次的等待窗口 */
					    s_tx_wait_tick    = Rt_tick;   /* V1_0_57: 确认窗口同步重起算 */
					}
					else
					{
					    s_same_addr_retry = 0U;
					    send_adrr_cnt++;
					}
					if(send_adrr_cnt>2)
					{
						Lora_Tx_state=0; 
						send_adrr_cnt=0;  s_same_addr_retry = 0U;
						Ble_flag_data=2; 
                        lora_self_report_retry();
						send_flag=0;
						Lo_data.RfTick = Rt_tick;   /* 本轮结束，更新亮屏周期基准；息屏待上传任务独立管理。 */
						LORA_FRAME_PRINT("all fail send\r\n");
				  	g_diag_allfail++; 
						printf("[UP] FAIL allfail=%lu slot=%d/%d/%d fail=%d t=%lums\r\n",(unsigned long)g_diag_allfail,(int)Setup.transmit_number[0],(int)Setup.transmit_number[1],(int)Setup.transmit_number[2],(int)Lo_data.Failure,(unsigned long)Rt_tick);
					/* 自愈①：连续 LORA_HEAL_FAIL_ROUNDS 轮三地址全失败 -> 回归默认三地址 */
					if (s_all_fail_cnt < 250U) 
            { s_all_fail_cnt++; }
					if (s_all_fail_cnt >= LORA_HEAL_FAIL_ROUNDS)
					{
						s_all_fail_cnt = 0U;
						Lo_data.Failure = 0;
						GetNewsend_addr();
						send_adrr_cnt = 0U;
						s_same_addr_retry = 0U;
						LORA_FRAME_PRINT("[SELFHEAL] fail rounds -> restore default next hops\r\n");
					}
					}
					else
					{
						Lora_Tx_state=Tx_oneself;
					}
				}
			
			break;
		case Tx_Forwarding: //转发
			  if((TickDiff_100ms(Leav_tick)<TickLeav))  	//信道上有数据
				{				
					//printf("1111\r\n");
					  // TickLeav --;
						 break;
				}

				/* V1_0_57：CAD 结果的**有界化**。
				   Data_cad() 的语义是 fail-open：只有"确实检测到前导(状态 6)"才返回 0，
				   "没结果 / 被误唤醒 / CAD 结果丢失"都返回 1（照发），所以它本身不会
				   永久挡住发送。但如果信道被长期判忙（邻居密集、DIO2 毛刺、CAD 结果
				   丢失），原来会一直走 return 0 分支 -> 永远不发。
				   这里给"连续判忙"加一个上限：超过 LORA_CAD_BUSY_MAX(30) 次就允许
				   强发一次，保证"每轮必有一包"。幂等，且不影响正常 LBT 语义： */
				{
				    uint8_t _cad_idle = Data_cad();            /* 1 = 信道空闲，可发 */
				    if(_cad_idle != 0U)
				    {
				        s_cad_busy_cnt = 0U;
				    }
				    else
				    {
				        if(s_cad_busy_cnt < 250U) { s_cad_busy_cnt++; }
				        if(s_cad_busy_cnt >= LORA_CAD_BUSY_MAX)
				        {
				            s_cad_busy_cnt = 0U;
				            _cad_idle      = 1U;                /* fail-open：连续忙 -> 强发 */
				            g_cad_force_cnt++;
				        }
				    }
				    if(_cad_idle == 0U) { break; }              /* 信道忙：下一拍再试 */
				}
				{
					
					 memset(Lo_data.Tx_buf,0,100);
		//		   memcpy(Lo_data.Tx_buf,Lo_data.ForwardBuf,100);
					send_next_lora();
					 
					{ 
					 uint8_t _d=Setup.transmit_number[send_adrr_cnt]; if(!(_d==STATION || (_d!=0u && _d!=NONEADDR && Setup.Host_number>3u && _d<Setup.Host_number))) { Setup.transmit_number[send_adrr_cnt]=STATION; } }   /* V1_0_25: 防回传闸门：只允许发更小台号或分站；1/2/3 只能发分站 */
					 Lo_data.Tx_data_len= Load_Packaging(Lo_data.Tx_buf,Setup.transmit_number[send_adrr_cnt] ); //组包函数
					 G_LoRaConfig.PayloadLength =	Lo_data.Tx_data_len; //发送长度
					 if(Setup.transmit_number[send_adrr_cnt]==122)
					 {
							for_send_num=12;
					  }
						else
						{
								if(rx_state==0)
								{
				
					            for_send_num=LORA_PREAMBLE_NEIGHBOR;   /* V1_0_57: 前导统一由 LORA_PREAMBLE_NEIGHBOR 给（2000 符号/128ms） */
								}
								else{
									for_send_num=LORA_PREAMBLE_NEIGHBOR;   /* 同上：2000 符号/128ms */
								}

						}
           // printf("Tx_Forwarding\r\n");
						   //printf("self send addr == %d;for_send_num ==%d\r\n",Setup.transmit_number[send_adrr_cnt],for_send_num);
					  LSD_RF_SendPacket(Lo_data.Tx_buf,Lo_data.Tx_data_len,for_send_num); 

						re_iwdg();
	
					  SX126X_WOR_Exit();  //进入读监测;

					  Lora_Tx_state=Tx_wait_Forwarding;
					  Lo_data.RfRxTick=Rt_tick;
					  s_tx_wait_tick=Rt_tick;   /* V1_0_57: same as above */
			  }
			break;
		case Tx_wait_Forwarding: //转发等待成功

				if(TickDiff_100ms(s_tx_wait_tick)>LORA_TX_CONFIRM_TICKS) /* V1_0_57: 5 ticks -> 8 ticks(800ms), and use a DEDICATED start point: the old one was Lo_data.RfRxTick, which is refreshed by the RX window (SX126X_WOR_Exit) and by 0x0A/0x0C receptions -> the timeout could never elapse ("wait for ack forever") */
				{
					Lo_data.Failure++;
					if (s_same_addr_retry < 2U)   /* V1_0_24 P5: symmetric with Tx_wait_oneself */
					{
					    s_same_addr_retry++;
					    Lo_data.RfRxTick  = Rt_tick;
					    s_tx_wait_tick    = Rt_tick;   /* V1_0_57: 同上（转发重试分支） */
					}
					else
					{
					    s_same_addr_retry = 0U;
					    send_adrr_cnt++;
					}
					if(send_adrr_cnt>2)
					{
					  Lora_Tx_state=0;
						Lo_data.Forward_flag=0;
						send_adrr_cnt=0; s_same_addr_retry = 0U;
					}
					else
					{
						Lora_Tx_state=Tx_Forwarding;
					}
				}
			break;


		default:
			break;
	}

}

/* ==========================================================================
 * ★★★ V1_0_56 现场修复：射频"卡在中间态"的看门狗 + 回主页归位
 * --------------------------------------------------------------------------
 * 现场 bug：绑定蓝牙后从【设置页】按【返回主页】，LoRa 收发彻底停，只有硬复位恢复。
 * 代码依据（改前）：
 *   · 下面 case 0 用 `UI.Setting_flag==0` 把 lora_work_state()（Timer_task ->
 *     CAD_work / 自愈 / [DIAG]）整段跳过 —— 用户在设置页的整段时间里，
 *     **射频状态机没有任何消费者**，而 Lora_send() 照旧在跑、还在改射频状态；
 *   · case 3（RX 等待）的收尾只在 lcd_open==0（息屏）时才做，**亮屏时永远出不来**；
 *   · 页面切换只改 UI 变量，Lora_Rx_state / Lora_Tx_state / send_flag /
 *     token_flag / Lo_data.RfTick 一个字都不碰 -> "软返回主页"继承全部残值，
 *     而硬复位把它们通通清零，所以只有复位能恢复。
 * 这里加两件事（都不改协议/参数，只做状态归一）：
 *   ① LORA_RX_STUCK_TICKS：任何非 0 的 Lora_Rx_state 停留超过 3s 就强制收尾；
 *   ② LORA_BOOT_QUIET_TICKS：复位后 2s 内不主动抢发（替代原来那个永久锁）。
 * ========================================================================== */

volatile uint32_t g_lora_rx_stuck_cnt = 0U;   /* 强制收尾次数（诊断） */
static uint8_t  s_last_setting_flag  = 0U;    /* Lora_task：上一拍的 UI.Setting_flag */

void Lora_reve(void)
{  
		/* ★ 射频状态看门狗（见上面说明）：状态没变化且非 0 超过 3s -> 强制收尾。
		   幂等：收尾之后状态就是 0，下一拍自然回到 lora_work_state()。 */
		if(Lora_Rx_state != s_rx_state_last)
		{
			s_rx_state_last = Lora_Rx_state;
			s_rx_state_tick = Rt_tick;
			s_rx_state_hms  = HAL_GetTick();
		}
		else if((Lora_Rx_state != 0U) &&
		        /* V1_0_57：双时基 —— Rt_tick(100ms，STOP 下仍在走) 或 HAL_GetTick(SysTick)。
		           为什么要第二条：主循环每圈都 re_iwdg()，万一 LPTIM1 停走导致 Rt_tick 冻住，
		           只看 Rt_tick 的看门狗会变成"喂着狗死等"（比看门狗复位更糟）。HAL_GetTick
		           只统计清醒时间；正常一次收/发等待清醒时间 <= 12s，取 60s 上限不会误伤。 */
		        ((TickDiff_100ms(s_rx_state_tick) > LORA_RX_STUCK_TICKS) ||
		         ((uint32_t)(HAL_GetTick() - s_rx_state_hms) > LORA_STUCK_MS_AWAKE)))
		{
			g_lora_rx_stuck_cnt++;
			printf("[LORA] rx state %u stuck > %lus -> force re-arm\r\n",
			       (unsigned)Lora_Rx_state,
			       (unsigned long)((uint32_t)LORA_RX_STUCK_TICKS / 10U));
			(void)SX126X_WOR_Execute(Sx1268_Sleep);   /* 内部会把 Lora_Rx_state 置 0 */
			Lora_Rx_state   = 0U;
			s_rx_state_last = 0U;
			s_rx_state_tick = Rt_tick;
			s_rx_state_hms  = HAL_GetTick();
			/* V1_0_57：收尾时**必须同时清发送侧的残值**（用户要求）。只清 Lora_Rx_state
			   会立刻从"等应答/退避"里重入同一个阻塞：TickLeav 还挂着退避窗口、send_flag
			   还是 1（周期自发分支被永久屏蔽）、RfRxTick 还是上一轮的旧值。全部重打基准。*/
			TickLeav         = 0U;
			Leav_tick        = Rt_tick;
			Lo_data.RfRxTick = Rt_tick;
			s_tx_wait_tick   = Rt_tick;
			s_cad_busy_cnt   = 0U;
			send_flag        = 0U;
		}

		/* V1_0_57：状态 6（CAD 报"有活动"）必须被 CAD_work()/Data_cad() 消费掉。
		   万一没被消费（设置页里 case 0 被跳过、CAD 路径提前 return），它会一直占着
		   switch 的 default 分支 -> case 0 再也进不去 -> 连 CAD 都停了（只有 3s 看门狗
		   兜底）。这里给一个 2 拍(200ms) 的有界兜底：超过 2 拍没人消费就关掉。 */
		if((Lora_Rx_state == 6U) && (TickDiff_100ms(s_rx_state_tick) > 2U))
		{
		    (void)SX126X_WOR_Execute(Sx1268_Sleep);
		    Lora_Rx_state   = 0U;
		    s_rx_state_last = 0U;
		    s_rx_state_tick = Rt_tick;
		}

		/* V1_0_57：先跑"清道夫"，把 1/2/5/7/8 这些没有 case 处理器的残值在 1 拍内
		   收尾，不再依赖 3s 看门狗；与 UI.Setting_flag / 页面无关。 */
		lora_rf_housekeeping();
		switch(Lora_Rx_state)//读空闲的时候
		{
			 case 0: 
				 if(UI.Setting_flag==0&&Setup.On_Off_flag==1)
				 {
						lora_work_state();//定时工作
				 }
				break;
			 case 3: //读等待
					if(UI.lcd_open==0)
					{
						if(TickDiff_100ms(Lo_data.RfRxTick) > lora_rx_window_ticks()) /* V1_0_57: idle 4 ticks(400ms), post-TX ack wait 8 ticks(800ms) - see lora_rx_window_ticks() */
						{
//				       printf("3213213\r\n");
							 SX126X_WOR_Execute(Sx1268_Sleep);
               //re_iwdg();


						}
					}
					/* ★ V1_0_56：亮屏时也要有出口 —— 上面那段只在息屏(lcd_open==0)生效，
					   亮屏时如果包被截断/CRC 错（DIO1 不再来），状态会永久停在 3。
					   这里补一条同构的上限（LORA_RX_STUCK_TICKS=3s），与看门狗互为保险。 */
					else if(TickDiff_100ms(Lo_data.RfRxTick) > LORA_RX_STUCK_TICKS)
					{
						g_lora_rx_stuck_cnt++;
						SX126X_WOR_Execute(Sx1268_Sleep);
						Lora_Rx_state = 0;
					}
		
				 break;
		   case 4: //读成功
					  RxData_check();
            if(Lora_Rx_state!=3)
						{
							
							 re_iwdg();

			         SX126X_WOR_Execute(Sx1268_Sleep);
						 	 Lora_Rx_state=0;
               re_iwdg();

						}

 
			  break;
			 default:

				break;
		}

	
}

void check_work_time(void)
{
	rx_lora.End=HAL_GetTick();  // 使用HAL库获取当前时间
	if(	(rx_lora.End-rx_lora.Start)>Time_RX_lora)
	{
		wait_rx_lora_flag=1;
		//printf("rx_END\r\n");
	}
}
uint8_t cnt_rx=0;
//uint8_t cnt
volatile uint32_t g_cad_awake_ms = 0;  /* ★★★ V1_0_15：CAD 路径累计"醒着"的毫秒数
                                        * （纯诊断量）。用法：J-Link 采两次，Δ俯 / Δ秒
                                        * 就是 MCU 清醒占空比，再乘运行电流即得 MCU 平均功耗。
                                        * 注意 HAL_GetTick 走的是 SysTick，**在 STOP 下不走**，
                                        * 所以这个数天然只统计"醒着"的时间 —— 正好是我们要的。 */
volatile uint32_t g_rx_hit_cnt = 0;         /* ★ V1_0_23：CAD 命中（进过 RX）的累计次数 */
volatile uint32_t g_rx_hit_cnt_recent = 0;  /* 最近 1 秒内的命中数（自适应 CAD 的判据） */
volatile uint32_t g_rx_ms = 0;              /* 命中后停留在 RX 的累计毫秒（诊断） */
volatile uint32_t g_cad_cnt = 0;      /* ★★★ V1_0_12：CAD 累计次数（纯诊断量，不参与任何逻辑）
                                       * 用途：用 J-Link 在线采样 N 秒的增量来**实测 CAD 周期**
                                       *   （增量/N ≈ 10 次/秒 即周期 100ms，见 Timer_task 的说明）。
                                       * 也可以用它判断"LoRa 到底还在不在跑"——息屏期间这个数
                                       *   必须一直在涨，不涨就是 CAD 卡住了。 */
/* V1_0_27: auto gear switching by traffic countdown (see Timer_task). */
#define CAD_FAST_TICKS     1U      /* fast gear: 100ms (TickDiff unit = 100ms) */
#define CAD_SLOW_TICKS     1U      /* V1_0_57：慢档关掉 = 100ms，必须与快档同为 100ms：
                                             前导 2000 符号 = 128ms，若 CAD 周期 200ms
                                             则 128 < 200 -> 结构性漏检（就是"结构性收不到"）。
                                             全工程统一 100ms：128/100 = 1.28x 余量，同旧设备。 */
#define CAD_HOLD_CYCLES    300U    /* hold FAST for 300 cycles after a packet (~30-60s) */
static uint32_t s_cad_fast_hold = 0U;
volatile uint32_t g_cad_fast_cnt = 0U;
volatile uint32_t g_cad_slow_cnt = 0U;

/* CAD_work ：一次信道活动检测。
   启动 CAD 后进 STOP 等 DIO2/DIO3，命中则开长接收窗口收整包（见 CAD 路径注释）。
   约束：STOP 出来的第一件事可能是"改射频模式"，必须确认切换已走完。 */
void CAD_work(void)
{
    lora_force_rf_exit_once();        /* V1_0_57：若回主页时射频正忙（s_force_rf_exit=1），在这里补做一次有界退出 */
    lora_early_wake_normalize();      /* Observe early wake without disturbing RF/ACK state. */
    g_cad_cnt++;                      /* 诊断计数，不参与任何判据 */
        uint32_t h0 = HAL_GetTick();    /* 入口时刻（诊断用，STOP 下不走故只计清醒时间） */

//		printf("CAD_work\r\n");
//  	 rx_lora.Start=HAL_GetTick();  // 使用HAL库获取当前时间
      SX126X_WORInit();
    // printf("CAD_work1111\r\n");
      SX126X_WOR_Execute(Sx1268_CAD);

//	   rx_lora.out=HAL_GetTick();  // 使用HAL库获取当前时间	
//		 printf("T1==%d\r\n", rx_lora.out-rx_lora.Start);
		  work_sleep();
					/* 射频模式切换闸门：一拍都没睡满就被叫醒（g_stop_woke_early）
					   时，CAD 很可能还没做完，不能立刻改射频模式；
					   这里最多补等 LORA_RADIO_SETTLE_MS，CAD 一有结果就 break。 */
					if (g_stop_woke_early != 0U)
					{
					    uint32_t h0 = HAL_GetTick();
					    g_radio_settle_cnt++;
					    do
					    {
					        re_iwdg();
					        sensor_rx_task();          /* 环形缓冲只有 128B，别忘了搬 */
					        if (Lora_Rx_state != 5)    /* CAD 已出结果：射频不在忙了 */
					        {
					            break;
					        }
					    } while ((uint32_t)(HAL_GetTick() - h0) < LORA_RADIO_SETTLE_MS);
					}

  
	    if( Lora_Rx_state== 6)   //检查到信息了
	  	{
					//printf("Lora_Rx_state == 6\r\n");
//				  SystemClock_Config();
          SystemClock_Config();
		      SystemPower_Config();
					SX126X_WOR_Exit(); //长接收模式

					wait_rx_lora_flag=0;
          rx_lora.Start=HAL_GetTick();
          s_rx_wait_rt0 = Rt_tick;  // 使用HAL库获取当前时间
//			    work_sleep();
//				  cnt_rx=0;
//					while(Lora_Rx_state==3&&wait_rx_lora_flag==0)
					while(Lora_Rx_state==3&&wait_rx_lora_flag==0 && (TickDiff_100ms(s_rx_wait_rt0) <= 8U))
					{
           //   cnt_rx++;
						  check_work_time();
						//	work_sleep();
						  re_iwdg();
						  /* 收包等待期间每一轮都服务三件事，不能纯忙等：
						     串口(128B FIFO 会溢出)、亮屏时的按键与刷屏；1ms 限速防热自旋。 */
						  sensor_rx_task();                 /* 串口：分帧 + 校验 + 填 Ch_senser[] */
						  if(UI.lcd_open==1)                /* 亮屏：按键与显示都跟上 */
						  {
						      soft_timer_loop_async();      /* Key_task / Time_1s（按键在这里派发） */
						      UI_display();                 /* 有变化就发 0x01 帧（刷屏不落后） */
						  }
						  HAL_Delay(1);                     /* 限速：约 1000 轮/秒，够快也不空转 */
					}	 
         //  rx_lora.End=HAL_GetTick();  // 使用HAL库获取当前时间
				//	printf("RX_time == %d\r\n",rx_lora.End- rx_lora.Start);
//			    work_sleep();
					if(Lora_Rx_state == 4)   //接收到有效数据
					{
							 // printf("Xaaa\r\n");
//							 SystemClock_Config();
							  RxData_check();
					}
					else  //没有接收到数据，被误唤醒
					{
					   	  Lora_Rx_state=0;
//							printf("Lora_Rx_state == %d\r\n",Lora_Rx_state);
			         // printf("X222\r\n");
						    SX126X_WOR_Execute(0);	 
					}
		}
		if(Lora_Rx_state != 3)//如果射频状态为 RXWAITING,则表明需要继续数据
		{	 

		   SX126X_WOR_Execute(0);

		}
		/* 这里不再补 work_sleep()：本次 CAD 的等待已在上面那次 work_sleep() 里做过，
		   再睡一次会把 CAD 周期顶到 ~200ms 以上，前导就盖不住接收方的 CAD 采样。 */
		/* 诊断量：累计本次 CAD 路径的"醒着"时间（HAL_GetTick 在 STOP 下不走）。 */
		g_cad_awake_ms += (uint32_t)(HAL_GetTick() - h0);
		if (Lora_Rx_state == 6 || Lora_Rx_state == 4) { g_rx_hit_cnt++; g_rx_hit_cnt_recent = 1U; s_cad_fast_hold = CAD_HOLD_CYCLES; } else { g_rx_hit_cnt_recent = 0U; }  /* V1_0_27 */
		if (g_rx_hit_cnt_recent != 0U) { g_rx_ms += (uint32_t)(HAL_GetTick() - h0); }   /* V1_0_23: idle-RX ms */
}

uint8_t cnt_cad=0;                    /* 保留：判据不再用它，仅便于观察 */
static uint32_t s_lora_cad_tick = 0;  /* 上一次 CAD 的 100ms 节拍 */

#define SCHED_TEST_MODE        1        /* ★ 台架验证用：1 = 60s 短轮；量产改 0 */
#define SCHED_SLOT_TICKS       40U      /* 每台 4.0s（100ms 拍） */
#define SCHED_NODES            120U
#if SCHED_TEST_MODE
  #define SCHED_ROUND_TICKS    600U     /* 60s 短轮（仅台架验证） */
#else
  #define SCHED_ROUND_TICKS    (SCHED_SLOT_TICKS * SCHED_NODES)   /* 480s */
#endif

static uint32_t s_sched_phase = 0U;      /* 轮相位：轮内时刻 = (Rt_tick-phase) % 轮长 */
static uint8_t  s_sched_valid = 0U;      /* 相位是否可信（收到帧才置 1） */
volatile uint32_t g_sched_in_cnt  = 0U;  /* 诊断：窗口内 CAD 次数 */
volatile uint32_t g_sched_out_cnt = 0U;  /* 诊断：窗口外 CAD 次数 */
volatile uint8_t  g_sched_now     = 0U;  /* 诊断：当前是否在监听窗口内 */

/* 本机当前是否处在"必须监听"的窗口内（同时维护诊断计数） */
static uint8_t sched_in_window(void)
{
    uint32_t round_t;
    uint8_t  k;
    uint32_t a;
    uint32_t slot;
#if SCHED_TEST_MODE
    s_sched_valid = 1U;                  /* 台架：直接认为相位可信，才能看到切换 */
#endif
    if (s_sched_valid == 0U)
    {
        g_sched_in_cnt++;                /* 相位未知 -> 一直快档（fail-safe） */
        g_sched_now = 1U;
        return 1U;
    }
    round_t = (uint32_t)((Rt_tick - s_sched_phase) % (uint32_t)SCHED_ROUND_TICKS);
    for (k = 58U; k <= 60U; k++)
    {
        a = (uint32_t)Setup.Host_number + (uint32_t)k;
        if (a > (uint32_t)SCHED_NODES) { continue; }
        slot = (uint32_t)((a - 1U) * SCHED_SLOT_TICKS) % (uint32_t)SCHED_ROUND_TICKS;
        if (round_t >= slot && round_t < (slot + SCHED_SLOT_TICKS))
        {
            g_sched_in_cnt++;
            g_sched_now = 1U;
            return 1U;
        }
    }
    g_sched_out_cnt++;
    g_sched_now = 0U;
    return 0U;
}
/* 100ms 节拍任务：诊断输出 + 自愈 + 按快/慢档决定这一拍做 CAD 还是 work_sleep。 */
void Timer_task(void)
{
	/* 自愈②：长时间（LORA_HEAL_NO_RX_TICKS 拍）没收到有效数据 -> 回归默认三地址 */
	/* V1_0_26 现场诊断：每 LORA_DIAG_MS 输出一行机器可解析状态 */
	{
		static uint32_t s_t_diag = 0U;
		if (s_t_diag == 0U) { s_t_diag = Rt_tick; }
		else if ((uint32_t)(Rt_tick - s_t_diag) >= (LORA_DIAG_MS / 100U))
		{
			s_t_diag = Rt_tick;
			printf("[DIAG] t=%lums part=%d host=%d slot=%d/%d/%d onoff=%d fail=%d heard=%d allfail=%lu diagms=%d rx=%u txst=%u reset=%d rxstuck=%lu\r\n",
				(unsigned long)Rt_tick, (int)Setup.partition, (int)Setup.Host_number,
				(int)Setup.transmit_number[0], (int)Setup.transmit_number[1], (int)Setup.transmit_number[2],
				(int)Setup.On_Off_flag, (int)Lo_data.Failure,
				(int)((g_diag_lasthearslot == 255U) ? -1 : (int)g_diag_lasthearslot),
				(unsigned long)g_diag_allfail, (int)LORA_DIAG_MS,
				(unsigned)Lora_Rx_state, (unsigned)Lora_Tx_state,
				(int)reset_flag, (unsigned long)g_lora_rx_stuck_cnt);
			if (s_all_fail_cnt > 0U) { printf("[DIAG] consecutive_allfail=%d (heal at %d)\r\n", (int)s_all_fail_cnt, (int)LORA_HEAL_FAIL_ROUNDS); }
		}
	}	if (s_last_valid_ms == 0U) { s_last_valid_ms = Rt_tick; }
	else if ((uint32_t)(Rt_tick - s_last_valid_ms) > LORA_HEAL_NO_RX_TICKS)
	{
		s_last_valid_ms   = Rt_tick;
		s_all_fail_cnt    = 0U;
		GetNewsend_addr();
		send_adrr_cnt     = 0U;
		s_same_addr_retry = 0U;
		LORA_FRAME_PRINT("[SELFHEAL] no valid rx for 10min -> restore default next hops\r\n");
	}   
        uint32_t th = CAD_SLOW_TICKS;
        if (s_cad_fast_hold != 0U) { s_cad_fast_hold--; th = CAD_FAST_TICKS; g_cad_fast_cnt++; }
        else                       { g_cad_slow_cnt++; }
        if (TickDiff_100ms(s_lora_cad_tick) >= th)
    /* V1_0_24: slow CAD mode removed -- with Tp=2000sym(128ms) the inequality Tp>=Tc*k forces Tc<=100ms, so there is no room for an idle档. Coverage first. */
    {
        s_lora_cad_tick = Rt_tick;
        CAD_work();
    }
    else
    {
        /* 息屏:100ms STOP；亮屏/息屏读窗口:10ms 有界泵（按键、串口照常） */
        work_sleep();
    }
// 	 HAL_IWDG_Refresh(&hiwdg) ;//喂狗
}
//uint8_t rx_state=0;
//uint32_t  lrx_tick=0;
/* LoRa 工作状态分发：rx_state==0 走定时任务，==1 直接做一次 CAD。 */
void lora_work_state(void)
{

	  switch(rx_state)
		{
			case 0:
			 // printf("rx_state==0\r\n");
				Timer_task();
				break;
			case 1:
		//  	printf("rx_state==1\r\n");
//				if(TickDiff_100ms(lrx_tick) >300) //30秒
//				{
//					rx_state=0;
//				}
				CAD_work();
				
				break;
			default:
				rx_state=0;
				break;
		}

}

#define BOOT_UPLOAD_ONLY_WHEN_LCD_OFF   0   /* 1 = 这次补发只在息屏（lcd_open==0）时做 */

static uint8_t s_boot_upload_done = 0U;     /* 冷启动后是否已补发过（只发一次） */

static void boot_upload_check(void)
{
    uint8_t i;
    uint8_t bound = 0U;

    if (s_boot_upload_done != 0U) { return; }        /* 已经补发过 */
    if (Setup.On_Off_flag != 1)   { return; }        /* 关机态不上报 */
#if BOOT_UPLOAD_ONLY_WHEN_LCD_OFF
    if (UI.lcd_open != 0U)        { return; }        /* 只等息屏 */
#endif

    for (i = 0U; i < 20U; i++)
    {
        if (Ch_senser[i].tpye == 0) { continue; }    /* 未绑定通道不参与判断 */
        bound++;
        if (Ch_senser[i].write_time == 0U)
        {
            return;                                  /* 这一路本轮还没收到数据 -> 不完整 */
        }
    }
    if (bound == 0U) { return; }                     /* 没有已绑定设备：不触发 */

    s_boot_upload_done = 1U;                         /* ★ 只发一次 */
    Ble_flag_data      = 1U;                          /* ★ 复用既有"自发数据"分支 */
}

/* ==========================================================================
 * V1_0_57：发射状态看门狗（最后一道保险，不影响任何正常路径）
 * --------------------------------------------------------------------------
 * 覆盖 Tx_oneself / Tx_wait_oneself / Tx_Forwarding / Tx_wait_Forwarding 四个
 * 状态：只要"同一个状态"连续停留超过 LORA_TX_STUCK_TICKS(120 拍 = 12s)，就强制
 * 收回 Tx_None 并清干净残值。
 * 为什么 12s 足够宽（不会误伤）：Tx_wait_* 最坏 = 3 地址 x (1 + 2 重试) x 8 拍
 * = 72 拍 = 7.2s 必然收尾；Tx_oneself 的连续判忙最多 30 次(CAD 周期 100~200ms)
 * ≈ 3~6s 就强发一次；一次真正发射最长 = 2000 符号前导(128ms) + 载荷 + 5s 上限。
 * 全部 <= 7.2s < 12s，所以触发它一定是真卡住了。
 * ========================================================================== */
static void Lora_tx_watchdog(void)
{
    uint8_t waiting;
    uint8_t self_timeout;

    waiting = ((Lora_Tx_state == Tx_oneself)       ||
               (Lora_Tx_state == Tx_wait_oneself)  ||
               (Lora_Tx_state == Tx_Forwarding)    ||
               (Lora_Tx_state == Tx_wait_Forwarding)) ? 1U : 0U;

    if ((uint8_t)Lora_Tx_state != s_tx_state_last)
    {
        s_tx_state_last = (uint8_t)Lora_Tx_state;
        s_tx_state_tick = Rt_tick;
        s_tx_state_hms  = HAL_GetTick();
        return;
    }
    if (waiting == 0U) { return; }
    /* V1_0_57：双时基（理由同 Lora_reve 的 RX 看门狗）：Rt_tick 或 HAL_GetTick */
    if ((TickDiff_100ms(s_tx_state_tick) <= LORA_TX_STUCK_TICKS) &&
        ((uint32_t)(HAL_GetTick() - s_tx_state_hms) <= LORA_STUCK_MS_AWAKE)) { return; }

    g_lora_tx_stuck_cnt++;
    printf("[LORA] tx state %u stuck > %lus -> force Tx_None (send_flag=%u cadbusy=%u)\r\n",
           (unsigned)Lora_Tx_state,
           (unsigned long)((uint32_t)LORA_TX_STUCK_TICKS / 10U),
           (unsigned)send_flag, (unsigned)s_cad_busy_cnt);

    self_timeout = ((Lora_Tx_state == Tx_oneself) ||
                    (Lora_Tx_state == Tx_wait_oneself)) ? 1U : 0U;
    if (self_timeout != 0U) { lora_self_report_retry(); }
    Lora_Tx_state        = Tx_None;
    send_adrr_cnt        = 0U;
    s_same_addr_retry    = 0U;
    s_cad_busy_cnt       = 0U;
    TickLeav             = 0U;
    Leav_tick            = Rt_tick;
    send_flag            = 0U;
    if (self_timeout == 0U) { Lo_data.Forward_flag = 0U; }
    /* A forwarding timeout must not reset the self-report deadline/pending job. */
    s_tx_state_last      = (uint8_t)Tx_None;
    s_tx_state_tick      = Rt_tick;
    s_tx_state_hms       = HAL_GetTick();
}

void Lora_task(void)
{
	/* ★★★ V1_0_56 现场修复（主力防线）：只要检测到"从设置页(Setting_flag!=0)
	   回到主页(Setting_flag==0)"这个跳变，就把 LoRa 状态机整份归位一次。
	   为什么放在这里而不是各个按键分支里：
	     · 返回主页的写法有 4 处（菜单级 K2/Opt7、恢复出厂、息屏超时 Time_1s、
	       关机提示页开机），任何一处漏改都会重现现场 bug；
	     · 这个跳变判据在 Lora_task() 里每拍都跑，与"是哪条路径写出来的"无关；
	     · 归位函数 Lora_HomeEntry_ReArm() 是幂等的，重复调用等价于一次。 */
	if((s_last_setting_flag != 0U) && (UI.Setting_flag == 0U))
	{
		Lora_HomeEntry_ReArm();
	}
	s_last_setting_flag = UI.Setting_flag;

	boot_upload_check();   /* ★ V1_0_9：复位后数据收齐就补发一次（只一次） */

	Lora_tx_watchdog();   /* V1_0_57：发射状态看门狗（12s 兜底） */

	lora_early_wake_normalize();   /* Early wake is diagnostic only; preserve pending RX and ACK timing. */

	Lora_reve(); //接收

  Lora_send(); //发送
}

void SetRecv(void)
{

  TestRepeater -> FailureCnt = 0;
  TestRepeater -> SuccessCnt ++;
	//	printf(" TestRepeater -> SuccessCnt ==%d\r\n", TestRepeater -> SuccessCnt);
  RfNet.Status = RFNET_SENDTESTDATA;
}
uint8_t token_flag=0;
//处理接收的数据
/* 收包处理：校验和/长度 -> 按 CMD 处理 0x0A/0x0B/0x0C/0x0D，
   同时学习/更新三槽发送地址与转发链。 */
uint8_t RxData_check(void)
{  
	uint8_t Cmd;
  int index=0;

  u16 Addr_num=0;
  u16 for_send_num=0;
	SX126X_RxPacket(Lo_data.Rx_Buf);//接收数据包
  re_iwdg();
	
	if(G_LoRaConfig.PayloadLength==0||G_LoRaConfig.PayloadLength>98)
	{

		 return 0;
	}
	if(!CheckCheckSum(Lo_data.Rx_Buf,G_LoRaConfig.PayloadLength))                   //数据校验和不对
	{
	   return 0;
  }


//	if( Lo_data.Rx_Buf[0]==3)
//	{
//			LORA_FRAME_HEX(Lo_data.Rx_Buf,G_LoRaConfig.PayloadLength);
//	}
	
	
	 // printf("Lo_Rx == %d\r\n",G_LoRaConfig.PayloadLength);
 // Part  = Lo_data.Rx_Buf[0]; // 分站
 // Addr  = Lo_data.Rx_Buf[1]; // 要发送的地址
	//host  = Lo_data.Rx_Buf[3]; // 发送的数据地址

  Cmd   = Lo_data.Rx_Buf[9]; //指令
	

    if ((Cmd == 0x0AU) && (Lo_data.Rx_Buf[0] == Setup.partition))
    {
        lora_report_token_on_rx(Lo_data.Rx_Buf[13]);
    }
	Addr_num=Lo_data.Rx_Buf[13];

	LORA_FRAME_PRINT(" stu== %d;Send[%d]to[%d]  old== %d ;Cmd == %d\r\n",Lo_data.Rx_Buf[0],Lo_data.Rx_Buf[3],
	Lo_data.Rx_Buf[1], Lo_data.Rx_Buf[13],Cmd);
//	hex_p(Lo_data.Rx_Buf,G_LoRaConfig.PayloadLength);
	if(Lo_data.Rx_Buf[0] == Setup.partition&&Lo_data.Rx_Buf[13]> Setup.Host_number)//地址编号
  {
       if(token_flag==0&&Cmd==0x0A&&(g_sensor_data_valid!=0U))   /* V1_0_58：复位后第一帧完整采集未到之前，暂时忽略邻居令牌触发 */
       { 
				//  printf("Addr_num")
				Addr_num-=Setup.Host_number;     
					if(Addr_num<=3)  //间隔发送时间
					{     
              LORA_FRAME_PRINT("Addr_num == %d\r\n",Lo_data.Rx_Buf[13]);		  	
							send_flag=1;
							send_time1=Addr_num*100; //同步一次发送时间
							token_flag=1;
						  Sx_tick=Rt_tick;
					}
       }
  }
	if(Lo_data.Rx_Buf[0]==Setup.partition&&Lo_data.Rx_Buf[1]==Setup.Host_number) //同一个分站的属于自己的数据
	{
		    switch(Cmd)
				{
					 case 0x0C:                                                                //组网测试回复帧
					  if((RfNet.TestIndex - 1)<3)
					  {
							 dis_Rssi[RfNet.TestIndex - 1]=Lo_data.Rx_Buf[10] ;
						//	 printf("dis_Rssi111[%d]  == %d \r\n",RfNet.TestIndex - 1,dis_Rssi[RfNet.TestIndex - 1]);
							
					  }
					  SetRecv();
					  Lora_Rx_state = 3;
            Lo_data.RfRxTick = Rt_tick;
            rx_state=1;
					  lrx_tick=Rt_tick;		
           break;
					case 0x0B:                                                     //组网从机回复应答   
            memset( Lo_data.Tx_buf,0,14);						
					  Lo_data.Tx_buf[0] =  Lo_data.Rx_Buf[2];                                     //对换源地址和目的地址
						Lo_data.Tx_buf[1] =  Lo_data.Rx_Buf[3];
						Lo_data.Tx_buf[2] =  Lo_data.Rx_Buf[0];                                     //对换源地址和目的地址
						Lo_data.Tx_buf[3] =  Lo_data.Rx_Buf[1];
						Lo_data.Tx_buf[9] =  0x0C;                                                //对组网测试帧的回复帧
						Lo_data.Tx_buf[10] = G_LoRaPara.Ack_RSSI;                                 //回复信号强度
						SetCheckSum(Lo_data.Tx_buf,11);                                           //计算校验和
					  G_LoRaConfig.PayloadLength = 12;                             //加载数据包长度
					//	printf("Part == 0x0B\r\n");
						if(Lo_data.Tx_buf[1] ==122)
					  {
							for_send_num=12;
				  	}
						else
						{
							for_send_num=12;   /* FIX: 回复给正在RX监听的请求方, 6符号短前导(对齐MSP430 ACK) */

						}
						SX126X_Awake(Lo_data.Tx_buf,for_send_num);                                          //发送测试回复帧
						SX126X_WOR_Exit();
           break;
                    case 0x0A:
                        /* One existing forward buffer; do not restart an active
                           self/forward attempt when another packet arrives. */
                        if (Lo_data.Forward_flag == 0U)
                        {
                            memcpy(Lo_data.ForwardBuf, &Lo_data.Rx_Buf[10], 84U);
                            Lo_data.Forward_flag = 1U;
                            s_forward_ready_tick = Rt_tick + 2U; /* Original 200ms give-way. */
                        }
                        Lora_Rx_state = 3U;
                        Lo_data.RfRxTick = Rt_tick;
                        break;

					case 0x0D:
 						//  printf("0x0D\r\n");
						   rx_state=1;
					     dis_Rssi[send_adrr_cnt]=G_LoRaPara.Ack_RSSI ;
					    // printf("dis_Rssi222[%d]  == %d \r\n",RfNet.TestIndex - 1,dis_Rssi[RfNet.TestIndex - 1]);
							switch(Lora_Tx_state)
							{
								case Tx_wait_oneself://收到回复帧之后再更新时间
				//			  	if(Part==Setup.partition&& Lo_data.Rx_Buf[13]==Setup.Host_number) //同一个分站的属于自己的数据
				//					{
										//	printf("CD\r\n");
											Lo_data.Failure=0; s_all_fail_cnt = 0U; s_last_valid_ms = Rt_tick;
											Lora_Tx_state=0; //
								      Ble_flag_data=0;
							      	send_flag=0;
                        lora_self_report_complete();
											Lo_data.RfTick = Rt_tick;   /* 本轮结束，更新亮屏周期基准；息屏待上传任务独立管理。 */
				//					}
									break;
																						 
								case Tx_wait_Forwarding:
				//					if(Lo_data.Rx_Buf[3]==now_host_addr&&Lo_data.Rx_Buf[13] == Tx11)//第三位等于发送出去的地址
				//					{
//										 	printf("DDDDFFF\r\n");
							     	Lo_data.Forward_flag=0;
										Lo_data.Failure=0; s_all_fail_cnt = 0U; s_last_valid_ms = Rt_tick;
										Lora_Tx_state=0;
				//					}
					break;
				default:
					break;
			}
						break;
		}
	 }
	 else
	 {
		 switch(Cmd)
		 {
			 case 0x0A:
//				 	case 0x0D:
				 	if(Setup.transmit_number[0]!=122&&Setup.transmit_number[0]!=121)
					{
if( Setup.transmit_number[0]>Lo_data.Rx_Buf[3] && Lo_data.Rx_Buf[3]!=0 && Lo_data.Rx_Buf[3]!=121 && Lo_data.Rx_Buf[3]!=122)   /* V1_0_24: reject illegal upstream (0/121/122) */
							{
								index=Setup.transmit_number[0]- Lo_data.Rx_Buf[3]; g_diag_lasthearslot = Lo_data.Rx_Buf[3];
								if(index>=3&&index<60&&index%3==0)
								{
									if(Setup.transmit_number[1]==0u || Setup.transmit_number[1]==NONEADDR || Setup.transmit_number[1]>Setup.transmit_number[0])
									{ Setup.transmit_number[2]=Setup.transmit_number[1]; Setup.transmit_number[1]=Setup.transmit_number[0]; }   /* V1_0_25: slot1/slot2 同步动态化，保持 Host>slot0>slot1>slot2 */
									Setup.transmit_number[0]=Lo_data.Rx_Buf[3] ;//更新最新发送地址
						      printf("updata == %d \r\n",Setup.transmit_number[0]);
								}
								rx_state=1;
								lrx_tick=Rt_tick;					
							}
			    }
//				else{
//				//	printf("t1== %d;t2 == %d ;t3 == %d\r\n",Lo_data.Rx_Buf[3],Setup.transmit_number[0],Lo_data.Rx_Buf[1]);
//				
//				}
					if(Lo_data.Rx_Buf[3]<Setup.Host_number)
					{
						rx_state=1;
						lrx_tick=Rt_tick;		
					}
		//	dis_Rssi[send_adrr_cnt]=Lo_data.Rx_Buf[10] ;
			//	printf("dis_Rssi4444[%d]  == %d \r\n",send_adrr_cnt,dis_Rssi[send_adrr_cnt]);
			switch(Lora_Tx_state)
			{
				case Tx_oneself:
				case Tx_wait_oneself://收到回复帧之后再更新时间
						/* 残留调试打印 printf("4444444444"); 已由 V1_0_55 删除 */
			  	if(Lo_data.Rx_Buf[0]==Setup.partition&& Lo_data.Rx_Buf[13]==Setup.Host_number) //同一个分站的属于自己的数据
					{
					  send_flag=0;
					 // printf("Lo_data.Rx_Buf[3] ==%d ;Setup.transmit_number[%d] == %d\r\n",Lo_data.Rx_Buf[3],send_adrr_cnt,Setup.transmit_number[send_adrr_cnt]);
					  if(Lo_data.Rx_Buf[3]==Setup.transmit_number[send_adrr_cnt])
						{
						  dis_Rssi[send_adrr_cnt]=G_LoRaPara.Ack_RSSI ;
							//printf("G_LoRaPara.Ack_RSSI == %d\r\n",G_LoRaPara.Ack_RSSI);
						}
						 // printf("AAACCC\r\n");
						  Ble_flag_data=0;
							Lo_data.Failure=0; s_all_fail_cnt = 0U; s_last_valid_ms = Rt_tick;
					  	Lora_Tx_state=0; //
                        lora_self_report_complete();
							Lo_data.RfTick = Rt_tick;   /* 本轮结束，更新亮屏周期基准；息屏待上传任务独立管理。 */
					}
					break;
				case Tx_Forwarding:											 
				case Tx_wait_Forwarding:
				//	printf("x1== %d;x2==%d;x3==%d;x4==%d\r\n",Lo_data.Rx_Buf[3],now_host_addr,Lo_data.Rx_Buf[13],Tx11);
					if((Lo_data.Rx_Buf[3]<=now_host_addr)&&Lo_data.Rx_Buf[13] == Tx11)//第三位等于发送出去的地址
					{
					//	 printf("BBBCCC\r\n");
						if(Lo_data.Rx_Buf[3]==dis_Rssi[send_adrr_cnt])
						{
						  dis_Rssi[send_adrr_cnt]=G_LoRaPara.Ack_RSSI ;
						}
						Lo_data.Forward_flag=0;
						Lo_data.Failure=0; s_all_fail_cnt = 0U; s_last_valid_ms = Rt_tick;
						Lora_Tx_state=0;
					}
					break;
				
				default:
					break;
			}
			break;
			case 0x0D:
				if(Setup.transmit_number[0]!=122||Setup.transmit_number[0]!=121)
				{
					if(Lo_data.Rx_Buf[3]==122&&Setup.transmit_number[0]>Lo_data.Rx_Buf[1] && Lo_data.Rx_Buf[1]!=0 && Lo_data.Rx_Buf[1]!=121 && Lo_data.Rx_Buf[1]!=122)
					{
						index=Lo_data.Rx_Buf[1];
						if(index>=3&&index<60&&index%3==0)
						{
							if(Setup.transmit_number[1]==0u || Setup.transmit_number[1]==NONEADDR || Setup.transmit_number[1]>Setup.transmit_number[0])
									{ Setup.transmit_number[2]=Setup.transmit_number[1]; Setup.transmit_number[1]=Setup.transmit_number[0]; }   /* V1_0_25: 同上 */
									Setup.transmit_number[0]=Lo_data.Rx_Buf[1] ;//更新最新发送地址
						  printf("updata1 == %d \r\n",Setup.transmit_number[0]);
						}
				  	rx_state=1;
					  lrx_tick=Rt_tick;	
					}
				}
				
							switch(Lora_Tx_state)
			{
				case Tx_oneself:
				case Tx_wait_oneself://收到回复帧之后再更新时间
					printf("8888888888\r\n");
			  	if(Lo_data.Rx_Buf[0]==Setup.partition&& Lo_data.Rx_Buf[13]==Setup.Host_number) //同一个分站的属于自己的数据
					{
					  send_flag=0;
					//  printf("Lo_data.Rx_Buf[3] ==%d ;Setup.transmit_number[%d] == %d\r\n",Lo_data.Rx_Buf[3],send_adrr_cnt,Setup.transmit_number[send_adrr_cnt]);
					  if(Lo_data.Rx_Buf[3]==Setup.transmit_number[send_adrr_cnt])
						{
						  dis_Rssi[send_adrr_cnt]=G_LoRaPara.Ack_RSSI ;
						//	printf("G_LoRaPara.Ack_RSSI == %d\r\n",G_LoRaPara.Ack_RSSI);
						}
						 // printf("AAACCC\r\n");
						  Ble_flag_data=0;
							Lo_data.Failure=0; s_all_fail_cnt = 0U; s_last_valid_ms = Rt_tick;
					  	Lora_Tx_state=0; //
                        lora_self_report_complete();
							Lo_data.RfTick = Rt_tick;   /* 本轮结束，更新亮屏周期基准；息屏待上传任务独立管理。 */
					}
					break;
				case Tx_Forwarding:											 
				case Tx_wait_Forwarding:
				//	printf("x1== %d;x2==%d;x3==%d;x4==%d\r\n",Lo_data.Rx_Buf[3],now_host_addr,Lo_data.Rx_Buf[13],Tx11);
					if((Lo_data.Rx_Buf[3]<=now_host_addr)&&Lo_data.Rx_Buf[13] == Tx11)//第三位等于发送出去的地址
					{
					//	 printf("BBBCCC\r\n");
						if(Lo_data.Rx_Buf[3]==dis_Rssi[send_adrr_cnt])
						{
						  dis_Rssi[send_adrr_cnt]=G_LoRaPara.Ack_RSSI ;
						}
						Lo_data.Forward_flag=0;
						Lo_data.Failure=0; s_all_fail_cnt = 0U; s_last_valid_ms = Rt_tick;
						Lora_Tx_state=0;
					}
					break;
				
				default:
					break;
			}
					break;
		}
	 }
	// 	printf("RxData_check \r\n");
	 	return 1;
	 		
}

/* ==========================================================================
 * Lora_HomeEntry_ReArm ：把 LoRa 收发状态机拉回"已知良好"的起点
 * --------------------------------------------------------------------------
 * 调用点（全部幂等、可重入、不依赖复位）：
 *   ① Lora_task()：检测到 UI.Setting_flag 由非 0 变 0（= 返回主页）；
 *   ② usart.c 的 binding_parse_response()：收到 0x04/0x05/0x07 应答之后
 *      （绑定流程结束，把 0x0A/0x0D 事件留下的 send_flag/token_flag 残值清掉）。
 * 只做状态归一，**不改任何用户参数**（Host_number / On_Off_flag / 三槽地址一律
 * 不动），也不发任何帧、不进 STOP、不加延时。
 * ========================================================================== */
void Lora_HomeEntry_ReArm(void)
{
	uint8_t rx_before = Lora_Rx_state;
	uint8_t tx_before = (uint8_t)Lora_Tx_state;
	uint8_t radio_busy;

	/* 1 = 正在发（等 DIO1 TxDone）、3 = 正在收（等整包）：这两个状态可能是
	   "间接"调用到的（binding_parse_response 由 sensor_rx_task 调用，而
	   sensor_rx_task 也在 TX/RX 等待循环里被泵），此时**不发射频命令**，
	   只把软件状态归零，射频本身交给下一拍的 CAD / EnsureStandbyRC 同步，
	   以免把正在空口发送的包打断。 */
	radio_busy = ((Lora_Rx_state == 1U) || (Lora_Rx_state == 3U)) ? 1U : 0U;

	/* ① 射频：确认退出 CAD / RX 等待 / TX 等待，回到 Sleep（= 已知状态）。
	   V1_0_57 修正：原来 radio_busy 时**什么都不做**，只清软件状态 —— 射频可能
	   仍停在 RX/CAD 而软件认为它是 Sleep。虽然下一拍 SX126xEnsureStandbyRC() 会
	   把它拉回来，但"软/硬状态不一致"的窗口不该留。
	   现在改成：非忙时立刻退出；忙时只置一个**一次性请求** s_force_rf_exit，由
	   下一拍的 lora_force_rf_exit_once()（Data_cad / CAD_work 里，非重入上下文）
	   补做一次有界的 WOR_Execute(Sleep) —— 既不打断正在空口发送的包，也保证射频
	   最终一定回到已知状态，且不需要复位。 */
	if(radio_busy == 0U)
	{
		(void)SX126X_WOR_Execute(Sx1268_Sleep);   /* 内部会把 Lora_Rx_state 置 0 */
		s_force_rf_exit = 0U;
	}
	else
	{
		s_force_rf_exit = 1U;                     /* 下一拍补做，见上面说明 */
	}
	Lora_Rx_state   = 0U;
	s_rx_state_last = 0U;
	s_rx_state_tick = Rt_tick;

	/* ② 发送状态机：回到 Tx_None，重新开始一个完整的自发周期 */
	Lora_Tx_state     = Tx_None;
	send_adrr_cnt     = 0U;
	s_same_addr_retry = 0U;
	TickLeav          = 0U;
	Leav_tick         = Rt_tick;
	send_flag         = 0U;   /* 只有收到 0x0A/0x0D 才会置 1；残留会把 Tx_None 的
	                             "周期自发"分支永久屏蔽（那一支要求 send_flag==0） */
	token_flag        = 0U;
	Ble_flag_data     = 0U;
    s_report_tx_seq = 0U;  /* Keep any queued collection across UI/RF re-arm. */
	Lo_data.Forward_flag = 0U;

	/* ②b V1_0_57：把新增的各个看门狗 / 计数一起归位，避免"归位"之后再被旧残值
	      带偏（幂等：重复调用等价于一次）。 */
	s_tx_wait_tick      = Rt_tick;
	s_cad_busy_cnt      = 0U;
	s_tx_state_last     = (uint8_t)Tx_None;
	s_tx_state_tick     = Rt_tick;
	s_rx_state_hms      = HAL_GetTick();
	s_tx_state_hms      = HAL_GetTick();
	s_boot_stagger_done = 1U;   /* 回主页按完整周期计时，不再重复开机错峰。 */

	/* ③ 回主页后重新开始完整的 60 秒上传周期。 */
	Lo_data.RfRxTick = Rt_tick;
	Lo_data.RfTick   = Rt_tick;

	printf("[LORA] home re-arm: rx %u->0 tx %u->0 send_flag=0 rf=%lums\r\n",
	       (unsigned)rx_before, (unsigned)tx_before, (unsigned long)Rt_tick);
}


 //组网测试
/* 组网测试状态机：依次测三个备用地址，各发 10 帧统计成功率。 */
void RfNetPro(void)
{
	uint16_t send_num=0;
  switch(RfNet.Status)
  {
    case RFNET_IDLE:
      break;
    case RFNET_DONE:
      /* ★ V1_0_51：结果页停留。三个地址的结果都在 RfNet.Repeaters[] 里，
         ui.c 会把它们打包下发；按键由 New_Key.c 处理（0 重新组网 / 1 返回）。 */
      break;
    case RFNET_CYCLEINDEX:

      if(RfNet.TestIndex > 2)
      {
				UI.Addr=Set_Addr1;
				UI.dis_data_flag=1;

        /* ★★★ V1_0_51：测完**不自动返回**，停在结果页等用户选
           「重新组网 / 返回」（见 LoraPro.h 的 RFNET_DONE）。 */
        RfNet.Status = RFNET_DONE;
        break;
      }
        /* 每个地址开始测试时把 TestRepeater 指到它自己的槽位，三个地址的结果
           才能分开存（原来永远指向同一个 repeater，结果互相覆盖）。 */
        TestRepeater = &RfNet.Repeaters[RfNet.TestIndex];

			  TestRepeater -> Part = Setup.transmit_part[RfNet.TestIndex];
        TestRepeater -> Addr = Setup.transmit_number[RfNet.TestIndex];
        TestRepeater -> TestCnt    = 0;
        TestRepeater -> SuccessCnt = 0;
        TestRepeater -> FailureCnt = 0;

        RfNet.Status = RFNET_SENDTESTDATA;
			
        RfNet.TestIndex++;

       break;
    case RFNET_SENDTESTDATA:

      if((TestRepeater->TestCnt > 9) )
      {
          printf("[NET-TEST] target=%d succ=%d/%d\r\n", (int)TestRepeater->Addr, (int)TestRepeater->SuccessCnt, (int)TestRepeater->TestCnt);
          RfNet.Status = RFNET_CYCLEINDEX;
        break;
      }
      TestRepeater -> TestCnt++; //测试次数
      
      //数据目标节点
			memset(Lo_data.TestBuf,0,20);
      Lo_data.TestBuf[0] = TestRepeater -> Part;
      Lo_data.TestBuf[1] = TestRepeater -> Addr;
      //数据源节点
      Lo_data.TestBuf[2] = Setup.partition;
      Lo_data.TestBuf[3] = Setup.Host_number;
       Lo_data.TestBuf[9] = 0x0B;
      Lo_data.TestBuf[10] = 0x00;
      SetCheckSum(Lo_data.TestBuf,11);

	   // hex_p(Lo_data.TestBuf,12);
      G_LoRaConfig.PayloadLength=	12; //发送长度
			if(TestRepeater -> Addr== 122)
			{
				send_num=12;   /* 分站常听 12符号(对齐MSP430) */
			}
			else{
				
				if(rx_state==1)
				{
						send_num=2000;
				}
				else{
					  send_num=2000;   /* 普通节点CAD轮询 2000符号(128ms)覆盖 */
				}
			
		  }
			SX126X_Awake(Lo_data.TestBuf,send_num);

			SX126X_WOR_Exit();
			

      RfNet.Tick = Rt_tick; 
//			printf(" RfNet.Tick  ==%d\r\b", RfNet.Tick );
      RfNet.Status = RFNET_WAITACK;
      break;
    case RFNET_WAITACK:

      if( TickDiff_100ms(RfNet.Tick)>4)   /* FIX: 0x0C回复前导5000符号≈324ms > 原200ms超时, 改400ms否则回复永远赶不上窗口 */
			{
				if(TestRepeater -> TestCnt)
				{
		    	printf("RfNet.TestIndex ==%d\r\n",RfNet.TestIndex);
					
				}
				RfNet.Status = RFNET_SENDTESTDATA;
		 }

      break;
  }
}
uint8_t out_flag=0;
/* 通道超时扫描里的逐通道 printf 已门控：printf 走 USART2 是阻塞发送，
   20 个通道全超时时约 80ms 的主循环停顿会整块吃掉按键。
   现场排查"频道归零"时把 TIME_CHK_LOG 改成 1。 */
#ifndef TIME_CHK_LOG
#define TIME_CHK_LOG   0
#endif
#if TIME_CHK_LOG
#define TIME_CHK_PRINT(...)   printf(__VA_ARGS__)
#else
#define TIME_CHK_PRINT(...)   do { } while (0)
#endif

void work_Time_check(void)
{
	int i=0;
	for(i=0;i<20;i++)
	{
		if(Ch_senser[i].tpye)
		{
			if(TickDiff_100ms(Ch_senser[i].write_time) >Time_out_data)        
			{
				TIME_CHK_PRINT("Zero_Ch_777777777777777\r\n");
			  Ch_senser[i].Rssi=0;
				Ch_senser[i].voltage=0;
				Ch_senser[i].CH_data=0xFDFD;
				Ch_senser[i].write_time=Rt_tick;
				Ch_senser[i].fail_cnt=1;
				TIME_CHK_PRINT("Zero_Ch_%d\r\n",i+1);
			}
			else if((TickDiff_100ms(Ch_senser[i].write_time) >Time_out_20)&&Ch_senser[i].fail_cnt==0)
			{
				out_flag=1;
			}
	  }
	}
}

void Time_check_data(void)
{
	int i=0;
	for(i=0;i<20;i++)
	{
		if(Ch_senser[i].tpye)
		{
			if(TickDiff_100ms(Ch_senser[i].write_time) >Time_out_data)        
			{
				TIME_CHK_PRINT("Zero_Ch_666666666666666\r\n");
			  Ch_senser[i].Rssi=0;
				Ch_senser[i].voltage=0;
				Ch_senser[i].CH_data=0xFDFD;
				Ch_senser[i].write_time=Rt_tick;
				Ch_senser[i].fail_cnt=1;
				TIME_CHK_PRINT("Zero_Ch_%d\r\n",i+1);
			}

	  }
	}
}
uint32_t vdd_tick=0;
/* 100ms 节拍任务：通道超时归零 + 令牌超时 + 每小时重测 VDD。 */
void Time_check_task(void)
{
		//if(TickDiff_100ms(check_tick) >30)   
	if(TickDiff_100ms(check_tick) >Time_data_task)   
	{
		// printf(" host == %d;Rssi[0] == %d;Rssi[1]  == %d;Rssi[2]  == %d\r\n",Setup.Host_number,  dis_Rssi[0] ,dis_Rssi[1] ,dis_Rssi[2] );
	
	//	print_uart_sr(&hlpuart1);
		Time_check_data();
		check_tick=Rt_tick;
	}    
	if(token_flag==1&&(TickDiff_100ms(Sx_tick) >2400))//4分钟
  {
       token_flag=0;
			  printf("token_flag =0 \r\n");
       Sx_tick=Rt_tick;
  }
	if(TickDiff_100ms(vdd_tick) >60000) //一个小时
	{				
		vdd_tick=Rt_tick;
		get_vdd_adc_value_sleep();
	}
}
