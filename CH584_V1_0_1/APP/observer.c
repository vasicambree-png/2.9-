/********************************** (C) COPYRIGHT *******************************
 * File Name          : observer.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : �۲�Ӧ�ó��򣬳�ʼ��ɨ�������Ȼ��ʱɨ�裬���ɨ������Ϊ�գ����ӡɨ�赽�Ĺ㲥��ַ
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "observer.h"
#include "Usart3_task.h"
#include "Flash.h"

extern volatile uint8_t SW_SCAN_STATE ; //

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES             80

/* 扫描时长（单位 625us）。
 * ★ 契约 §2.1（2026-09-22 冻结）：**0 = 关闭扫描超时**（SDK 语义，见
 *   CH58xBLE_LIB.h:784 "Setting this parameter to 0 turns off the timeout"），
 *   观察者角色持续扫描、不再每 8 秒超时停一次。
 *   改前是 12800 ⇒ 12800 × 0.625ms = 8s：每 8 秒 stop→restart 一次，
 *   两轮之间存在扫描空档（空档时长未实测）。
 *   TGAP_DISC_SCAN_INT / TGAP_DISC_SCAN_WIND 保持 240/240（150ms/150ms = 100% 占空比），
 *   接收占空比本来就不变。
 *   万一该栈把 0 当成"不扫描"，由 observer_ble_stat_tick() 的扫描看门狗兜底重启
 *   （连续 ADV_QUIET_RESTART_SEC 秒没有任何广播包 ⇒ 重新 StartDiscovery）。 */
#define DEFAULT_SCAN_DURATION           12800
//#define DEFAULT_SCAN_DURATION           0
// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE           DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN    TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST     FALSE

uint8_t gStatus;

// Task ID for internal task/event processing
static uint8_t ObserverTaskId;

// Number of scan results and scan result index
static uint8_t ObserverScanRes;

// Scan result list
static gapDevRec_t ObserverDevList[DEFAULT_MAX_SCAN_RES];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ObserverEventCB(gapRoleEvent_t *pEvent);
static void Observer_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void ObserverAddDeviceInfo(uint8_t *pAddr, uint8_t addrType);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapRoleObserverCB_t ObserverRoleCB = {
    ObserverEventCB // Event callback
};

#define HASH_BUCKETS  MAX_DEVICES

//static int hash_head[HASH_BUCKETS];      //
//static int hash_next[MAX_DEVICES];       //

//
device_info_t device_list[MAX_DEVICES];
uint8_t device_count = 0;
//extern uint8_t ble_count;

/* 20 个通道，直接按通道号索引（下标 0..19 = 协议通道 1..20，契约 §2.2） */
device_t     CH_com_buf[MAX_CH_NUM];

/* 绑定表：每个 MAC 一条 */
scan_binding g_binding_list[MAX_BINDING_NUM];
uint8_t      g_binding_count = 0;

/* ================= 绑定名变更监测（10 分钟巡检）状态 ================= */
/* 名字快照表：下标与 g_binding_list 对齐（绑定表只追加不删除，索引稳定） */
name_snapshot_t   g_name_snap[MAX_BINDING_NUM];
/* 错误记录表（RAM，不落 EEPROM/Flash）：定长 16 条，满则环形覆盖最旧 */
name_change_err_t g_name_err_list[NAME_CHK_ERR_NUM];
uint8_t           g_name_err_count = 0;   /* 已写入条数，上限 NAME_CHK_ERR_NUM */
uint32_t          g_name_chk_sec   = 0;   /* 秒计数（由 usart 的 1s 事件驱动） */
uint32_t          g_name_chk_round = 0;   /* 已触发的巡检轮次（含被跳过的轮次） */

static uint32_t   g_name_chk_pending = 0; /* 距下一轮巡检的秒数 */
static uint8_t    g_name_err_next    = 0; /* 错误表下一条写入位置（环形索引） */
/* =================================================================== */

/* ================= 功能清单新增状态（2026-09-21） =================
 *   1) g_volt_err_count        「信息汇总 → 蓝牙电压异常警报:个数」
 *   2) g_scan_name_cache[]     「设备绑定 → 绑定设备」子页显示的"最近扫描到的蓝牙名称"
 *   3) g_round_mask            「一轮采集完成」位图（0x06 主动上报，契约 §5）
 * =================================================================== */

/* ---- 1) 电压异常计数 ----
 * 判定与去重规则见 volt_err_check()；计数在解绑/恢复出厂时清零。 */
uint8_t g_volt_err_count = 0;

/* ---- 2) 最近扫描到的蓝牙名称缓存 ----
 * ★ 为什么**不复用** device_list[MAX_DEVICES=2]（observer.c 上方）：
 *   - 容量不够：清单要求"最近扫描到的蓝牙名称"清单，2 条太少（本缓存 6 条）；
 *   - 结构体过重：device_info_t 每条含 50 字节 mfg_data，2 条 = 190 B，
 *     而本缓存只需要 MAC(6) + 名字(30) = 36 B/条，6 条 = 216 B 且语义单一；
 *   - device_list 在下层驱动 / 死代码（Flash.c 旧函数）里还有引用，改其口径
 *     会牵动无关代码。
 * 本缓存只在绑定模式（SCAN_MODE_BIND）里写入，见 SACN_DATA()。 */
scan_name_entry_t g_scan_name_cache[SCAN_NAME_CACHE_NUM];

/* ---- 3) 一轮采集完成位图 + 1 秒时基 ---- */
uint32_t g_round_mask = 0;          /* bit i = 通道 i 本"轮"已收到新数据 */
static uint32_t g_round_sec       = 0;  /* 1 秒事件累加的时间基准 */
static uint32_t g_round_last_sec  = 0;  /* 上一次上报（或清零）时的基准 */
/* =================================================================== */

/* ============ 绑定落盘标志 + 0x03「收齐本轮再应答」状态（契约 §2.3/§2.6） ============
 * 声明与语义见 observer.h 的同名注释块。这里的三个约定：
 *   1) g_store_dirty 只由 add_binding()（BLE 回调上下文）置位，
 *      只由 Usart3_task.c 的 1 秒事件清零并调用 binding_store_save() 落盘：
 *      **BLE 回调里绝不擦写 Flash**。
 *   2) g_dev_fresh[]/g_dev_miss[] 下标与 g_binding_list[] 一一对应；
 *      绑定表只**追加**（永不删中间项），所以新增一条时只需把新槽位清零，
 *      索引不会整体搬移；整表清空（0x04/0x07/开机恢复）时两表一起 memset。
 *   3) g_rsp_pending / g_rsp_start_sec 只在 1 秒事件里读改写：
 *      受理请求（也来自 1 秒事件之外的 case 0x03，但那是主循环上下文，非中断）
 *      与超时判定同源，不会与 BLE 回调竞争。
 * ============================================================================ */
volatile uint8_t g_store_dirty = 0;    /* 1 = 有新增绑定待落盘（1 秒事件消费） */

uint8_t  g_rsp_pending = 0;            /* 1 = 有 0x03 请求正在等本轮数据 */
uint8_t  g_dev_fresh[MAX_BINDING_NUM]; /* 本轮该设备是否已收到新数据 */
uint8_t  g_dev_miss[MAX_BINDING_NUM];  /* 连续未收到新数据的轮数（0..DEV_MISS_ROUNDS_MAX） */
/* 0x03 请求受理时刻的秒计数（g_rsp_sec 的值），只在本文件与 1 秒 tick 里使用 */
static uint32_t g_rsp_start_sec = 0;
/* 秒计数器：由 observer_cmd03_sec_tick()（Usart3_task.c 的 1 秒事件）推进。
 * 独立于 g_name_chk_sec，避免两处时基互相牵制（契约 §2.6 要求"记起始秒"）。 */
static uint32_t g_rsp_sec = 0;
/* ============================================================================ */

/* ================= 扫描模式（P1/P2 修复 + 功能清单页面驱动） =================
 * 定义放在 observer.c（extern 声明放在 APP/include/observer.h）。
 * 语义（契约 §4，2026-09-21 冻结）：
 *   停留在「设备绑定 → 绑定设备」二级子页 = 绑定模式（解析名字 + add_binding）
 *      条件：menu_rank == 3 && rank2_addr == 2 && UI_main.re_flag == 2
 *   其余任何页面                           = 只更新模式（按绑定表填数据）
 *   ★ 旧版"开机 30 秒窗口"已由 Usart3_task.c 删除，绑定改由页面驱动。
 * 写入方：Usart3_task.c 的 1 秒事件（START_TIMER_EVT）
 * 读取方：SACN_DATA()（广播回调）、name_change_check_round()（10 分钟巡检）
 * ======================================================== */
/* 初值取 DATA（只更新）：上电时还没收到任何 0x01 帧（menu_rank=0），
 * 按页面驱动口径就应是"只更新"；1 秒事件会在第一次 tick 时按页面重新判定。
 * （旧代码初值是 BIND，配合"开机 30 秒窗口"；窗口删除后 BIND 初值已无意义。） */
volatile uint8_t g_scan_mode   = SCAN_MODE_DATA;   /* 由 1 秒事件按当前页面切换 */
volatile uint8_t g_scan_enable = 0;                /* Wait for explicit ON after every reset. */
static uint8_t g_observer_ready = 0;
static uint8_t g_collection_state = 0;
static uint8_t g_collection_state_seen = 0;
static uint32_t g_collection_state_frames = 0;

/* ================= 采集期打印降噪开关（任务 A-3） =================
 * 逐通道 / 逐包的 PRINT 是采集期最大的软件开销：PRINT → printf → _write()
 * 是**阻塞式**串口发送（while(FIFO 满); THR = *buf++），@115200 每字节 ≈ 87 us，
 * 期间 TMOS 其它任务（含 BLE 协议栈）全部被推迟。
 *
 *   SCAN_DBG_CH_LOG == 0（默认）：
 *       逐通道 / 逐包打印**完全编译掉**（不产生代码、不占 Flash 常量、不进热路径）：
 *       格式串与 printf 调用在预处理阶段就消失，因此是真正的零开销。
 *   SCAN_DBG_CH_LOG == 1：
 *       恢复调试打印，并可用运行期开关 g_ch_log_enable 动态关掉（不必重编）。
 *
 * 无论开关如何，以下打印**始终保留**（低频 / 异常）：
 *       [BIND] 绑定成功、[WARN] 名字被占用、通道冲突、绑定表满、
 *       名字变更巡检汇总与告警、绑定的通道结构性异常
 *       （通道越界 / payload 过短 / ch_num 不匹配）。
 * ================================================================
 * ★★ V1_0_2 现场问题修正（2026-09-23）：本开关曾被改成 1（连同
 *    Usart3_task.c 的 USART3_DBG_FRAME_LOG），而这就是"CH584M 处理过多
 *    数据就卡住"的直接原因，现已改回 0：
 *      PRINT → printf → _write() 是**阻塞式**串口发送（@115200 每字节 ≈87us）。
 *      CH_LOG 打在 observer_adv_drain() 的消费路径上（每 10ms 事件最多 16 条），
 *      也在 SACN_DATA 的绑定模式分支上，单条日志 60~80 字节 = 5~7ms。
 *      16 条 × 5~7ms = 80~112ms 全挤进一个 10ms 的 TMOS 事件 ⇒ TMOS / BLE
 *      协议栈被推迟十几倍，表现为"数据一多就卡死、扫描/上报一起停"。
 *    现场需要抓逐包日志时再临时改成 1，并且只在没有其它负载时打开。
 * ================================================================ */
#ifndef SCAN_DBG_CH_LOG
#define SCAN_DBG_CH_LOG   0
#endif

#if SCAN_DBG_CH_LOG
volatile uint8_t g_ch_log_enable = 1;   /* 运行期总闸：置 0 立即静音逐通道打印 */
#define CH_LOG(...)   do { if (g_ch_log_enable) PRINT(__VA_ARGS__); } while (0)
#else
volatile uint8_t g_ch_log_enable = 0;   /* 编译期已关闭，保留符号供运行期查询 */
/* 完全展开为空：不生成任何代码、不占 Flash 常量、不进热路径。
 * 只被打印用到的局部变量由代码里的 (void)xxx; 显式标记为"已使用"，
 * 以免新增 -Wunused-but-set-variable 告警。 */
#define CH_LOG(...)   do { } while (0)
#endif
/* ================================================================================ */

/* ================= 广播回调零阻塞：日志总闸（契约 §2.3） =================
 * BLE 广播回调（SACN_DATA）里**不得**出现阻塞式 PRINT：
 *   PRINT → printf → _write() 是阻塞式串口发送（@115200 每字节 ≈ 87us，
 *   一条几十字节的日志要 3.5~8ms），期间 TMOS 其它任务（含 BLE 协议栈）全部被推迟。
 *   —— 这正是"回调内零阻塞"要消灭的结构性隐患。
 *
 * g_ble_log_enable 默认 **0**：只在回调路径上会执行的明细日志（绑定失败原因、
 * 通道结构性异常等）全部静音；现场需要排查时置 1 即可恢复。
 * "关掉明细日志"的补偿：计数摘要由 observer_ble_stat_tick() 在 **1 秒事件
 * （主循环上下文）** 打印一行 [BLE] adv: ...，不受本开关影响。
 *
 * ★ 非回调路径的 PRINT 一律**保持原样**（[LOAD] 开机恢复、0x03/0x04/0x05/0x07
 *   流程、Flash 相关）—— 它们本来就在主循环上下文，改掉反而丢信息。
 * ★ 回调内也不得用 vsnprintf/sprintf 做"延迟日志"（新 libc 依赖 + 栈开销）。 */
volatile uint8_t g_ble_log_enable = 0;   /* 1 = 恢复回调路径明细日志（默认关） */
#define BLE_LOG(...)   do { if (g_ble_log_enable) PRINT(__VA_ARGS__); } while (0)
/* ================================================================================ */

/* ============ 广播回调异步化：生产者/消费者环形队列（契约 §2.2） ============
 * 生产者：SACN_DATA()      —— BLE 回调，跑在 ObserverTaskId 的 TMOS 任务上下文
 * 消费者：observer_adv_drain() —— Usart3_task.c 的 START_IO_EVT（10ms）事件
 * 两者由 observer_main.c 的 Main_Circulation()（app_uart_process(); TMOS_SystemProcess();）
 * **协作式**调度 ⇒ 正常情况不需要关中断/临界区。
 *
 * 纪律（配合 volatile 索引，防止将来退化成"中断里生产"时撕裂）：
 *   - 单生产者**只写 tail**；单消费者**只写 head**；
 *   - **留一个空槽判满**：next(tail) == head ⇒ 满 ⇒ 丢**最新**一条并 g_adv_q_drop++；
 *   - 消费者**处理完当前槽位后**才推进 head（生产者绝不会覆盖正在处理的条目）。
 * 即便预编译 BLE 库（CH58xBLE_LIB）退化成在中断里回调任务函数，
 * uint8_t 索引读写在 RISC-V 上是单字节访问、不会撕裂，队列仍自洽（最坏只是丢包计数增加）。
 *
 * 容量：ADV_Q_DEPTH(64) 条 × sizeof(adv_q_entry_t)=**73B**（无填充）= 4672B ≈ 4.56KB（bss，
 * 由链接后的 .bss.g_adv_q = 0x1240 实测确认）；因留 1 空槽，实际可用 63 条。
 * =========================================================================== */
#define ADV_Q_DEPTH        64      /* 条目数（必须是 2 的幂：用 & 取模） */
#define ADV_Q_DATA_MAX     64      /* 单条广播数据上限（与 MAX_PAYLOAD_LEN 同口径） */
/* 扫描看门狗阈值（契约 §2.1 补充）：连续这么多秒没有任何广播包 ⇒ 重新 StartDiscovery。
 * 用宏而不是写死字面量，便于按现场调整。 */
#define ADV_QUIET_RESTART_SEC  10

typedef struct
{
    uint8_t addr[6];                   /* 广播地址 */
    uint8_t event_type;                /* GAP_DEVICE_INFO_EVENT 的 eventType */
    int8_t  rssi;                      /* 信号强度（原样透传给 fill_channel_data） */
    uint8_t len;                       /* 实际拷入 data[] 的字节数（<= ADV_Q_DATA_MAX） */
    uint8_t data[ADV_Q_DATA_MAX];      /* 广播原始数据快照 */
} adv_q_entry_t;

static adv_q_entry_t     g_adv_q[ADV_Q_DEPTH];
static volatile uint8_t  g_adv_q_head = 0;   /* 消费者写：下一个待处理条目 */
static volatile uint8_t  g_adv_q_tail = 0;   /* 生产者写：下一个可写入条目 */

/* ---- 可见性计数（打印只在主循环上下文，见 observer_ble_stat_tick） ---- */
uint16_t g_adv_q_drop          = 0;   /* 队列满丢弃次数（丢最新） */
uint8_t  g_adv_q_max           = 0;   /* 历史最大积压（条） */
uint16_t g_bind_ok_cnt         = 0;   /* 成功新增绑定次数 */
uint16_t g_bind_reject_cnt     = 0;   /* 拒绑次数（名字被占/通道不足/区间冲突/绑定表满） */
/* ---- 扫描看门狗返回码三分类（契约 §2.1 补充；语义已按 SDK 头文件定死） ----
 *   SUCCESS                      ⇒ g_scan_restart_cnt++：**之前确实是停的**，被救回来了
 *   bleAlreadyInRequestedMode(0x11) ⇒ g_scan_busy_cnt++ ：扫描本来就在跑
 *                                      （10 秒静默只是现场没有广播源）—— 正常，非故障；
 *                                      会随静默时间每 10 秒 +1，属**预期**增长
 *   其它非 SUCCESS（如 bleIncorrectMode 0x12） ⇒ g_scan_err_cnt++：真异常 */
uint16_t g_scan_restart_cnt    = 0;   /* 返回 SUCCESS：真正重启了扫描 */
uint16_t g_scan_busy_cnt       = 0;   /* 返回 bleAlreadyInRequestedMode(0x11)：扫描本就在跑 */
uint16_t g_scan_err_cnt        = 0;   /* 其它非 SUCCESS 返回码：异常 */
volatile uint8_t g_adv_seen    = 0;   /* 入队成功置 1，由 1 秒事件消费（静默判据） */

static uint16_t g_adv_quiet_sec  = 0;      /* 距上次收到广播包的秒数 */
static uint16_t s_last_ok        = 0xFFFF; /* 摘要去重影子（初值取不可能值 → 首拍打印一次） */
static uint16_t s_last_rej       = 0xFFFF;
static uint16_t s_last_drop      = 0xFFFF;
static uint16_t s_last_rst       = 0xFFFF;
static uint16_t s_last_busy      = 0xFFFF;
static uint16_t s_last_err       = 0xFFFF;
static uint8_t  s_last_qmax      = 0xFF;
/* =========================================================================== */

uint8_t All_dat_flag;
extern data_LIST Data_list1;
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Observer_Init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
/* Main-loop/TMOS context only. Keep bindings and last readings intact. */
void observer_set_collection_enabled(uint8_t state)
{
    uint8_t enabled = (state == 1u);
    uint8_t state_changed = !g_collection_state_seen || g_collection_state != state;
    bStatus_t scan_status = SUCCESS;
    g_collection_state = state;
    g_collection_state_seen = 1;
    g_collection_state_frames++;

    if (enabled != g_scan_enable)
    {
        /* Close the producer gate before cancellation can report completion. */
        g_scan_enable = enabled;
        g_adv_q_head = g_adv_q_tail;
        g_adv_seen = 0;
        g_adv_quiet_sec = 0;
        if (!enabled)
        {
            observer_round_clear(); /* Do not report a partial pre-stop round. */
            if (g_observer_ready) scan_status = GAPRole_ObserverCancelDiscovery();
        }
        else if (g_observer_ready)
        {
            scan_status = GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                                         DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                                         DEFAULT_DISCOVERY_WHITE_LIST);
        }
        PRINT("[BLE-PWR] change state=%u en=%u ready=%u scan_rc=%02X\r\n",
              (unsigned int)state, (unsigned int)g_scan_enable,
              (unsigned int)g_observer_ready, (unsigned int)scan_status);
    }
    /* Main loop only. One attempt per new valid state, even on write failure;
     * repeated home frames and display wake never repeatedly erase Flash. */
    if (state_changed && state <= 1u && power_state_store_save(state) != 0u)
        PRINT("[BLE-PWR] persist failed state=%u; runtime state retained\r\n",
              (unsigned int)state);
}

void Observer_Init()
{
      ObserverTaskId = TMOS_ProcessEventRegister(Observer_ProcessEvent);



    GAP_SetParamValue(TGAP_DISC_SCAN, DEFAULT_SCAN_DURATION);
    GAP_SetParamValue(TGAP_DISC_SCAN_INT,240);//
    GAP_SetParamValue(TGAP_DISC_SCAN_WIND,240);//

    // Setup a delayed profile startup
    tmos_set_event(ObserverTaskId, START_DEVICE_EVT);


}

/*********************************************************************
 * @fn      Observer_ProcessEvent
 *
 * @brief   Simple BLE Observer Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Observer_ProcessEvent(uint8_t task_id, uint16_t events)
{
    //  VOID task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(ObserverTaskId)) != NULL)
        {
            Observer_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        // Start the Device

        GAPRole_ObserverStartDevice((gapRoleObserverCB_t *)&ObserverRoleCB);

        return (events ^ START_DEVICE_EVT);

    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Observer_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Observer_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GATT_MSG_EVENT:
            break;

        default:
            break;
    }
}



//

/*********************************************************************
 * @fn      ObserverEventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */

__HIGH_CODE
static void ObserverEventCB(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
        {
            g_observer_ready = 1;
            if (g_scan_enable)
                GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                               DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                               DEFAULT_DISCOVERY_WHITE_LIST);
            /* ★ 契约 §2.3：本事件也在 BLE 回调（ObserverEventCB）路径上 ⇒ 走 BLE_LOG。
             *   一次性开机日志（默认静音后由 1 秒摘要行承担可观测性）。 */
            BLE_LOG("Discovering...\n");
        }
        break;

        case GAP_DEVICE_INFO_EVENT:
        {
//            ObserverAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType);
            SACN_DATA(pEvent->deviceInfo.addr,pEvent->deviceInfo.eventType,
            pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen,pEvent->deviceInfo.rssi);
        }
        break;

        case GAP_DEVICE_DISCOVERY_EVENT:
        {
            //PRINT("Discovery over...\n");

            if (g_scan_enable)
                GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                               DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                               DEFAULT_DISCOVERY_WHITE_LIST);

        }
        break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      ObserverAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void ObserverAddDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
    uint8_t i;

    // If result count not at max
    if(ObserverScanRes < DEFAULT_MAX_SCAN_RES)
    {
        // Check if device is already in scan results
        for(i = 0; i < ObserverScanRes; i++)
        {
            if(tmos_memcmp(pAddr, ObserverDevList[i].addr, B_ADDR_LEN))
            {
                return;
            }
        }

        // Add addr to scan result list
        tmos_memcpy(ObserverDevList[ObserverScanRes].addr, pAddr, B_ADDR_LEN);
        ObserverDevList[ObserverScanRes].addrType = addrType;

        // Increment scan result count
        ObserverScanRes++;
    }
}

// 三为蓝牙协议解析

/**
 * 判断两段内存是否相同。
 * @return 1 相同，0 不同
 */
static inline uint8_t MEM_SAME(const void *a, const void *b, uint32_t n)
{
    return (tmos_memcmp(a, b, n) == TRUE) ? 1 : 0;
}

static inline uint8_t MEM_DIFF(const void *a, const void *b, uint32_t n)
{
    return (tmos_memcmp(a, b, n) == FALSE) ? 1 : 0;
}

/**
 * 在 [data, data+len) 中查找子串 "SW_"。
 * 找到返回首次出现的下标，未找到返回 -1。
 */
static int MEM_FIND_SW(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; (uint16_t)i + SW_NAME_PREFIX_LEN <= len; i++) {
        if (MEM_SAME(&data[i], SW_NAME_PREFIX, SW_NAME_PREFIX_LEN))
            return (int)i;
    }
    return -1;
}

/**
 * 解析广播里的 "SW_" 名字（0x08 简称 / 0x09 全名）。
 *
 * 规则：
 *   - 名字里**只要包含 "SW_" 子串**即为候选名（允许前面有前缀、
 *     后面有后缀，如 "A-SW_MG_1_1 xxx"）；
 *   - 优先取 0x09（完整名），没有再退回 0x08（简称）。同一设备若同时广播
 *     两种名字，固定取完整名，避免解析结果在两种名字之间抖动，
 *     导致绑定表把同一 MAC 误判成"信息不同"而拒绑；
 *   - 存入 name_buf 时保留原始名字，超出 MAX_NAME_LEN-1 才截断；
 *     需要截断时优先从 "SW_" 起截，避免把核心字段切掉。
 *
 * 成功返回 1，无名字返回 0。
 */
int parse_device_name_from_adv(const uint8_t *adv_data, uint8_t data_len,
                               char *name_buf, uint8_t *name_len)
{
    static const uint8_t kNameTypes[2] = {0x09, 0x08};   /* 完整名优先 */

    for (uint8_t t = 0; t < 2; t++) {
        const uint8_t *p   = adv_data;
        const uint8_t *end = adv_data + data_len;

        while (p + 1 < end) {
            uint8_t len = p[0];
            if (len == 0 || p + 1 + len > end) break;

            uint8_t        type = p[1];
            const uint8_t *val  = p + 2;
            uint8_t        vlen = len - 1;

            if (type == kNameTypes[t] &&
                vlen  >= SW_NAME_PREFIX_LEN &&
                MEM_FIND_SW(val, vlen) >= 0)          /* ★ 子串搜索 */
            {
                uint8_t off  = 0;
                uint8_t copy = vlen;

                if (copy > (uint8_t)(MAX_NAME_LEN - 1)) {
                    int sw = MEM_FIND_SW(val, vlen);
                    if (sw > 0) off = (uint8_t)sw;    /* 必须截断时从 "SW_" 起截 */
                    copy = (uint8_t)(vlen - off);
                    if (copy > (uint8_t)(MAX_NAME_LEN - 1))
                        copy = (uint8_t)(MAX_NAME_LEN - 1);
                }
                tmos_memcpy(name_buf, val + off, copy);
                name_buf[copy] = '\0';
                *name_len = copy;
                return 1;
            }
            p += 1 + len;
        }
    }
    return 0;
}

/**
 * 解析广播里的 payload（0xFF 厂商自定义段）。
 * *payload_len 入参是容量，出参是实际长度。
 */
int parse_payload_from_adv(const uint8_t *adv_data, uint8_t data_len,
                           uint8_t *payload_buf, uint8_t *payload_len)
{
    const uint8_t *p   = adv_data;
    const uint8_t *end = adv_data + data_len;

    while (p + 1 < end) {
        uint8_t len = p[0];
        if (len == 0 || p + 1 + len > end) break;

        uint8_t type = p[1];
        if (type == 0xFF) {
            uint8_t vlen = len - 1;
            if (vlen > *payload_len) vlen = *payload_len;
            tmos_memcpy(payload_buf, p + 2, vlen);
            *payload_len = vlen;
            return 1;
        }
        p += 1 + len;
    }
    return 0;
}


static Sensor_Tpye name_to_type(const char *s, uint8_t len)
{
    if (len != 2) return TPYE_NONE;
    if (s[0]=='M' && s[1]=='G') return TPYE_MG;
    if (s[0]=='J' && s[1]=='G') return TPYE_JG;
    if (s[0]=='W' && s[1]=='Y') return TPYE_WY2;   /* 具体由通道数细分 */
    if (s[0]=='L' && s[1]=='F') return TPYE_LF;
    if (s[0]=='Q' && s[1]=='J') return TPYE_QJ;
    if (s[0]=='Y' && s[1]=='L') return TPYE_YL;
    if (s[0]=='Y' && s[1]=='W') return TPYE_YW;
    if (s[0]=='W' && s[1]=='Z') return TPYE_WZ;
    if (s[0]=='D' && s[1]=='Y') return TPYE_DY;
    if (s[0]=='K' && s[1]=='K') return TPYE_KK;
    return TPYE_NONE;
}

/**
 * 解析一个**已经以 "SW_" 开头**的名字（parse_sensor_name 的内部实现）。
 *
 *   SW_<TYPE>_<host>_<start>[_<end>]
 *   X1 = host_num  主机号
 *   X2 = start     起始通道号（**1 基，1..MAX_CH_NUM(20)**，与 STM32 侧同一套设备名）
 *   X3 = end       结束通道号（**1 基**，可选，要求 X3 >= X2 且 X3 <= 20）
 *        - 若存在：ch_num = X3 - X2 + 1（"个数"，与基数无关）
 *        - 若不存在：ch_num = 1
 *
 * ★★ 通道号 → 下标（2026-09-21 契约 §2.1，核心修复）：
 *   设备名里写的是 **1 基通道号**，而 start_ch 会被直接当作 CH_com_buf[] 的
 *   **0 基下标**使用（occupy_channels / fill_channel_data / channels_available
 *   全是 CH_com_buf[start + i]），因此本函数输出的 *start_ch 必须是
 *       *start_ch = X2 - 1
 *   证据（厂商自己的另一份实现，STM32 侧已删除的蓝牙主机 ble_data.c）：
 *       ble_data.ch_num = X2;                            // SW_MG_1_1 → X2 = 1
 *       if((ch_num>=1)&&(ch_num<=20)) memcpy(&devices[ch_num -1], ...);
 *       uint8_t start_ch = devices[i].ch_num - 1;        // 起始通道索引（0基）
 *   改前写成 *start_ch = X2（少了 -1）⇒ SW_MG_1_1（通道 1）的数据被写进
 *   CH_com_buf[1]（= 通道 2），0x03/0x06 按下标上报 ⇒ 数据出现在通道 2、
 *   通道 1 永远为空（契约 §1.1 的"整体错位一格"）。
 *
 *   例：SW_MG_1_1        → host=1, 通道号 1（= 下标 0）
 *       SW_WY_25_9_12    → host=25, 通道号 9..12（= 下标 8..11）, ch_num=4
 *
 * 注意：
 *   - 激光（TPYE_JG）的 ch_num 由 payload 决定，
 *     本函数只返回名字里能解析出的初值，后续由 add_device 覆盖。
 *   - WY 按 ch_num 细分（4→WY4，6→WY6，8→WY8，其它→WY2）。
 *
 * 越界（契约 §2.1 要求，一律解析失败返回 0）：
 *   X2 == 0 或 X2 > 20      → 拒绝（0 基下标 20 及以上协议不承载）
 *   X3 < X2 或 X3 > 20      → 拒绝
 *
 * 容错：核心字段之后**允许带任意后缀**；只有核心字段本身错误
 *       （类型未知、缺主机号或起始通道、通道号越界）才返回失败。
 *
 * 成功返回 1，失败 0。
 */
static int parse_sensor_name_at(const uint8_t *name, uint8_t name_len,
                                Sensor_Tpye *type, uint8_t *host_num,
                                uint8_t *start_ch, uint8_t *ch_num)
{
    int        i, ts, digits;
    uint32_t   v;
    Sensor_Tpye t;
    uint8_t    host, start, end;
    uint16_t   cnt;

    if (name_len < 7) return 0;
    /* 调用方已确认前 3 字节为 "SW_" */

    /* ---------- 类型 ---------- */
    i = 3; ts = i;
    while (i < name_len && name[i] != '_') i++;
    if (i >= name_len) return 0;
    t = name_to_type(&name[ts], (uint8_t)(i - ts));
    if (t == TPYE_NONE) return 0;

    /* ---------- X1 主机号 ---------- */
    i++; v = 0; digits = 0;
    while (i < name_len && name[i] >= '0' && name[i] <= '9') {
        v = v*10 + (name[i]-'0'); i++; digits++;
    }
    if (digits == 0 || i >= name_len || name[i] != '_') return 0;
    host = (uint8_t)v;

    /* ---------- X2 起始通道号（1 基，1..MAX_CH_NUM） ---------- */
    i++; v = 0; digits = 0;
    while (i < name_len && name[i] >= '0' && name[i] <= '9') {
        v = v*10 + (name[i]-'0'); i++; digits++;
    }
    if (digits == 0) return 0;
    start = (uint8_t)v;
    /* ★ 契约 §2.1：1 基通道号必须是 1..20（0 基下标 0..19）。
     *   0 → 非法（协议里没有通道 0，STM32 侧 devices[X2-1] 会变成 devices[-1] 越界）；
     *   >20 → 该通道永远不会出现在 0x03/0x06 的 20 通道应答里（契约 §1.2）。 */
    if (start == 0 || start > MAX_CH_NUM) return 0;

    /* ---------- X3 结束通道号（可选，1 基） ---------- */
    cnt = 1;
    if (i < name_len && name[i] == '_') {
        int save = i;                           /* 回退点 */
        i++; v = 0; digits = 0;
        while (i < name_len && name[i] >= '0' && name[i] <= '9') {
            v = v*10 + (name[i]-'0'); i++; digits++;
        }
        if (digits == 0) {
            /* '_' 后面不是数字：当作名字后缀，保留 ch_num = 1 */
            i = save;
        } else {
            end = (uint8_t)v;
            if (end < start || end > MAX_CH_NUM) return 0;  /* 结束必须 >= 起始且不越界 */
            cnt = (uint16_t)(end - start + 1);      /* ch_num = X3 - X2 + 1（个数） */
        }
    }
    /*
     * 名字允许带后缀（"SW_MG_1_1 01"、"SW_MG_1_1-校准版" 等）：
     * 核心字段解析成功后，后面的内容一律忽略，因此这里不再要求 i == name_len。
     * 只有核心字段本身错误才在上面提前 return 0。
     */

    /* ---------- WY 按通道数细分 ---------- */
    if (t == TPYE_WY2) {
        switch (cnt) {
            case 4:  t = TPYE_WY4; break;
            case 6:  t = TPYE_WY6; break;
            case 8:  t = TPYE_WY8; break;
            default: t = TPYE_WY2; break;
        }
    }

    /* 激光不做任何 ch_num 修正，等 payload 决定 */

    *type     = t;
    *host_num = host;
    /* ★ 契约 §2.1：输出 **0 基下标**（= 通道号 - 1），下游全部按 0 基索引使用 */
    *start_ch = (uint8_t)(start - 1);
    *ch_num   = (uint8_t)cnt;
    return 1;
}

/**
 * 解析名字里的 SW_<TYPE>_<host>_<start>[_<end>]
 *
 * **子串搜索**：名字允许在 "SW_" 之前带任意前缀（如 "A-SW_MG_1_1"、
 * "现场1-SW_MG_1_1"），函数依次尝试每一个 "SW_" 起点，返回第一个能
 * 解析出合法核心字段的结果；核心字段之后允许带任意后缀。
 * 只有整串都找不到可解析的 "SW_" 核心时，才判定为格式错误返回 0。
 *
 * 例：SW_MG_1_1            → MG,  host=1,  start_ch=0（通道 1）, ch_num=1
 *      A-SW_MG_1_1         → MG,  host=1,  start_ch=0（通道 1）, ch_num=1
 *      现场1-SW_WY_25_9_12  → WY4, host=25, start_ch=8（通道 9..12）, ch_num=4
 *
 * ★ 返回的 start_ch 是 **0 基下标**（通道号 - 1），见 parse_sensor_name_at 注释。
 *
 * 成功返回 1，失败 0。
 */
static int parse_sensor_name(const uint8_t *name, uint8_t name_len,
                             Sensor_Tpye *type, uint8_t *host_num,
                             uint8_t *start_ch, uint8_t *ch_num)
{
    if (name_len < 7 || name_len > MAX_NAME_LEN) return 0;

    for (uint8_t s = 0; (uint16_t)s + SW_NAME_PREFIX_LEN <= name_len; s++) {
        if (name[s] != 'S' || name[s+1] != 'W' || name[s+2] != '_') continue;
        if (parse_sensor_name_at(&name[s], (uint8_t)(name_len - s),
                                 type, host_num, start_ch, ch_num))
            return 1;
    }
    return 0;
}

/**
 * 检测激光实际占用通道数。
 *
 * 激光 payload 固定 9 字节（★ 契约_传感器数据解析与显示标度 §1.4 / §3.2）：
 *   [0..1]  激光值（**大端**：高字节在前）
 *   [2..3]  倾角 1（大端，有符号）
 *   [4..5]  倾角 2（大端，有符号）
 *   [6..7]  倾角 3（大端，有符号）
 *   [8]     电压
 *
 * 判定：
 *   三个倾角全部 == 0xFDFD → 返回 1（仅激光）
 *   否则                   → 返回 4（1 激光 + 3 倾角）
 *   payload 太短 / 为空     → 返回 0（无法判断，需等下一包）
 *
 * 注：0xFDFD 的字节序列是 `FD FD`，**大小端对称**，故本判据不受字节序修正影响。
 */
static uint8_t detect_jg_channels(const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL || payload_len < 9)
        return 0;

    for (int i = 0; i < 3; i++) {
        uint16_t v = (uint16_t)((payload[2 + i*2] << 8) |        /* ★ 大端 */
                                payload[3 + i*2]);
        if (v != 0xFDFD)
            return 4;
    }
    return 1;
}
/* 已占用的通道数（valid == 1 的通道个数）
 * ★ 非 static：Flash.c 的 binding_store_load() 恢复完通道后要打印占用数（见 observer.h） */
uint8_t count_used_channels(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_CH_NUM; i++)
        if (CH_com_buf[i].valid) n++;
    return n;
}

/* ==================================================================
 * 功能清单新增接口（契约 §1.4 / §3 / §5）
 * ================================================================== */

/**
 * @brief 电压异常判定 + 计数（「信息汇总 → 蓝牙电压异常警报:个数」）
 *
 * 判定依据（阈值取法说明）：
 *   `voltage` 是广播 payload 里的 **1 字节原始值**（0..255）。本工程两端都没有
 *   任何 type→量程/标度的映射表（0x03 应答只透传 type 字节，不发单位），
 *   凭猜测设"低于 x V 算异常"会引入误报；但 **voltage == 0 是确定性异常**：
 *   设备正在持续广播说明它一定有供电，收到 0 只有三种可能：
 *     (a) 设备固件没带电压字段（payload 被裁剪），
 *     (b) 电压采样/上报异常，
 *     (c) 供电掉电（掉电设备通常不再广播，所以这是最轻的一种）。
 *   三者都属于需要告警的异常，因此判据取 `voltage == 0`，不设幅值上限，
 *   保证 0 误报。将来若两端确认了量纲，只需改这一个判据。
 *
 * 去重：
 *   同一台设备的多个通道共用同一个电压字节，如果逐通道累加，一台 4 通道设备
 *   会一次算 4 个异常。故用调用方传入的 counted 标志保证
 *   **同一个广播包（= 同一次 fill_channel_data()）只计一次**。
 *
 * @param voltage 本包电压字节
 * @param counted 本次 fill_channel_data() 的去重标志（初值 0）
 */
static void volt_err_check(uint8_t voltage, uint8_t *counted)
{
    if (voltage != 0)      return;
    if (*counted != 0)     return;      /* 本包已计过 */
    *counted = 1;
    if (g_volt_err_count < 0xFF) g_volt_err_count++;
}

uint8_t observer_volt_err_count(void)
{
    return g_volt_err_count;
}

/**
 * @brief 记录一个"最近扫描到的蓝牙名称"（「绑定设备」子页显示用）
 *
 * 规则：同 MAC 视为同一条（把它提到队首并刷新名字）；否则作为新条目插到队首，
 *       队列整体后移，容量满时挤掉最旧的一条（环形语义，队首最新）。
 * 调用方：SACN_DATA() 的绑定模式分支（有名字包时）。
 */
static void scan_name_cache_add(const uint8_t *mac, const char *name, uint8_t name_len)
{
    uint8_t i, dst;

    if (mac == NULL || name == NULL || name_len == 0) return;
    if (name_len > (MAX_NAME_LEN - 1)) name_len = (MAX_NAME_LEN - 1);

    dst = SCAN_NAME_CACHE_NUM;                 /* 默认：满表 → 覆盖最旧 */
    for (i = 0; i < SCAN_NAME_CACHE_NUM; i++)
    {
        if (MEM_SAME(g_scan_name_cache[i].mac, mac, 6))
        {
            dst = i;                           /* 已存在 → 提到队首 */
            break;
        }
        if (g_scan_name_cache[i].name[0] == '\0' && dst == SCAN_NAME_CACHE_NUM)
        {
            dst = i;                           /* 第一个空位 */
        }
    }
    if (dst == SCAN_NAME_CACHE_NUM) dst = (uint8_t)(SCAN_NAME_CACHE_NUM - 1);

    /* [0, dst) 整体后移一格，空出队首 */
    for (i = dst; i > 0; i--)
    {
        g_scan_name_cache[i] = g_scan_name_cache[i - 1];
    }

    tmos_memset(&g_scan_name_cache[0], 0, sizeof(scan_name_entry_t));
    tmos_memcpy(g_scan_name_cache[0].mac, mac, 6);
    tmos_memcpy(g_scan_name_cache[0].name, name, name_len);
    g_scan_name_cache[0].name[name_len] = '\0';
}

uint8_t scan_name_cache_count(void)
{
    uint8_t i;
    for (i = 0; i < SCAN_NAME_CACHE_NUM; i++)
    {
        if (g_scan_name_cache[i].name[0] == '\0') break;
    }
    return i;
}

const char *scan_name_cache_name(uint8_t idx)
{
    if (idx >= SCAN_NAME_CACHE_NUM) return NULL;
    if (g_scan_name_cache[idx].name[0] == '\0') return NULL;
    return (const char *)g_scan_name_cache[idx].name;
}

/* ---- 一轮采集完成（0x06 主动上报，契约 §5） ---- */

void observer_round_tick(void)
{
    g_round_sec++;                             /* 1 秒事件累加，不新开定时器 */
}

uint8_t observer_round_has_new_data(void)
{
    return (g_round_mask != 0) ? 1 : 0;
}

uint32_t observer_round_seconds_since_report(void)
{
    return g_round_sec - g_round_last_sec;
}

void observer_round_clear(void)
{
    g_round_mask     = 0;
    g_round_last_sec = g_round_sec;             /* 记录"刚刚上报过" */
}

/**
 * @brief 本轮位图是否已覆盖**全部已占用通道**
 * @return 1 = 收齐（且至少有一个已占用通道）；0 = 未收齐 / 当前没有绑定设备
 */
uint8_t observer_round_is_complete(void)
{
    uint8_t i, used = 0;

    for (i = 0; i < MAX_CH_NUM; i++)
    {
        if (!CH_com_buf[i].valid) continue;
        used++;
        if ((g_round_mask & (1UL << i)) == 0) return 0;   /* 该通道本轮还没数据 */
    }
    return (used > 0) ? 1 : 0;
}

/* [start, start+ch_num) 是否可用：范围合法 & 各通道空闲（或本就属于同 MAC） */
static int channels_available(uint8_t start, uint8_t ch_num, const uint8_t *mac)
{
    if (ch_num == 0) return 0;
    if ((uint16_t)start + ch_num > MAX_CH_NUM) return 0;

    for (uint8_t i = 0; i < ch_num; i++) {
        device_t *d = &CH_com_buf[start + i];
        if (d->valid && MEM_DIFF(d->mac, mac, 6))   /* ★ 不相等=FALSE */
            return 0;
    }
    return 1;
}

static void occupy_channels(const uint8_t *mac, const char *name, uint8_t name_len,
                            Sensor_Tpye type, uint8_t start, uint8_t ch_num)
{
    for (uint8_t i = 0; i < ch_num; i++) {
        device_t *d = &CH_com_buf[start + i];
        tmos_memset(d, 0, sizeof(device_t));
        tmos_memcpy(d->mac, mac, 6);
        /* name[] 必须留一个字节给 '\0'，否则 tmos_strlen 会越界读到 mac 字段 */
        if (name_len > (MAX_NAME_LEN - 1)) name_len = (MAX_NAME_LEN - 1);
        tmos_memcpy(d->name, name, name_len);
        d->Type  = type;
        d->valid = 1;
    }
}

int find_binding_by_mac(const uint8_t *mac)
{
    for (uint8_t i = 0; i < g_binding_count; i++) {
        if (MEM_SAME(g_binding_list[i].mac, mac, 6))   /* ★ 相同=TRUE */
            return (int)i;
    }
    return -1;
}

int find_binding_by_name(const char *name, uint8_t name_len)
{
    for (uint8_t i = 0; i < g_binding_count; i++) {
        if (tmos_strlen((char*)g_binding_list[i].name) == name_len &&
            MEM_SAME(g_binding_list[i].name, name, name_len))   /* ★ */
            return (int)i;
    }
    return -1;
}

/**
 * 绑定/更新。返回索引，失败 -1。
 * 规则（优先级从高到低）：
 *   1) 同 MAC，信息完全一致  → 幂等返回
 *   2) 同 MAC，信息不一致    → 告警（仅一条日志）+ 保留原绑定，返回原索引
 *   3) 名字被别的 MAC 占用   → 告警 + 拒绝
 *   4) 剩余通道不够          → 拒绝
 *   5) 目标通道区间冲突      → 拒绝
 *   6) 绑定表满              → 拒绝
 *   7) 通过                  → 写表 + 占通道
 */

static int add_binding(const uint8_t *mac,
                       const char *name, uint8_t name_len,
                       Sensor_Tpye type, uint8_t host_num,
                       uint8_t start_ch, uint8_t ch_num,
                       int known_idx)
{
    int idx;

    /* name[] 必须留一个字节给 '\0'，否则 tmos_strlen 会越界读到 mac 字段 */
    if (name_len > (MAX_NAME_LEN - 1)) name_len = (MAX_NAME_LEN - 1);

    /*
     * ★ 任务 A-1：调用方（SACN_DATA）在入口已经查过一次绑定表，结果通过
     *   known_idx 传进来，这里**不再重复线性查表**：
     *     known_idx >= 0 → 直接用该索引（先做一次 MAC 校验，防索引失效；
     *                      校验不过才回退成一次 find_binding_by_mac 兜底）；
     *     known_idx == -1 → 调用方已确认"未绑定"，直接进入下面的新增流程，
     *                      同样不做第二次查表。
     */
    if (known_idx >= 0)
    {
        if (known_idx < (int)g_binding_count &&
            MEM_SAME(g_binding_list[known_idx].mac, mac, 6))
        {
            idx = known_idx;                  /* 复用入口那次查询结果 */
        }
        else
        {
            idx = find_binding_by_mac(mac);   /* 兜底：传入索引不可信时才查表 */
        }
    }
    else
    {
        idx = -1;                             /* 已确认未绑定，不再查表 */
    }

    /* 1) / 2) 同 MAC 已绑定 */
    if (idx >= 0) {
        scan_binding *b = &g_binding_list[idx];

        if (tmos_strlen((char*)b->name) == name_len &&
            MEM_SAME(b->name, name, name_len) &&        /* ★ */
            b->Type         == type     &&
            b->host_num     == host_num &&
            b->frist_ch_num == start_ch &&
            b->ch_num       == ch_num)
        {
            return idx;                                 /* 幂等 */
        }

        /*
         * 同 MAC 但本次信息（名字/类型/主机号/起始通道/通道数）与绑定记录不一致：
         *   不再拒绝，也**不改动**原绑定记录 —— 类型、主机号、通道分配、
         *   以及后续的数据填充全部沿用原记录（内存布局完全不变）；
         *   只打印一条告警，并返回原来那条绑定的索引，
         *   让 add_device() 继续按原绑定填充数据（数据照常接收）。
         * 名字变更的正式上报由 10 分钟巡检任务（name_change_check_round）完成。
         */
        /* FIX(绑定页名字不更新): 同 MAC 广播名变化时更新显示名 b->name,
           让绑定页/汇总页显示当前名字; 类型/主机号/通道分配保持不变
           (避免通道重分配, 数据连续性不受影响)。 */
        if (tmos_strlen((char*)b->name) != name_len ||
            !MEM_SAME(b->name, name, name_len))
        {
            tmos_memset(b->name, 0, sizeof(b->name));
            tmos_memcpy(b->name, name, name_len);
            b->name[name_len] = (char)0;
        }

        BLE_LOG("[WARN] MAC bound with different name, keep old binding: "
              "MAC %02X:%02X:%02X:%02X:%02X:%02X old=%s(%d,%d,%d) new=%.*s(%d,%d,%d)\r\n",
              mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
              b->name, b->host_num, b->frist_ch_num, b->ch_num,
              name_len, name, host_num, start_ch, ch_num);
        return idx;                                     /* ★ 返回原绑定索引，不再 -1 */
    }

    /* 3) 名字被别的 MAC 占用 */
    int nidx = find_binding_by_name(name, name_len);
    if (nidx >= 0) {
        scan_binding *b = &g_binding_list[nidx];

        BLE_LOG("[WARN] name already used by another MAC → REJECT.\r\n");
        BLE_LOG("       name: %.*s\r\n", name_len, name);
        BLE_LOG("       owner MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
              b->mac[0],b->mac[1],b->mac[2],b->mac[3],b->mac[4],b->mac[5]);
        BLE_LOG("       new   MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
              mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        g_bind_reject_cnt++;                 /* ★ 契约 §2.3：拒绑计数（可见性） */
        return -1;
    }

    /* 4) 剩余通道是否够 */
    uint8_t used = count_used_channels();
    if ((uint16_t)used + ch_num > MAX_CH_NUM) {
        BLE_LOG("[WARN] channels full: used=%d need=%d max=%d → REJECT.\r\n",
              used, ch_num, MAX_CH_NUM);
        g_bind_reject_cnt++;                 /* ★ 契约 §2.3：拒绑计数（可见性） */
        return -1;
    }

    /* 5) 目标通道区间冲突 */
    if (!channels_available(start_ch, ch_num, mac))
    {
        BLE_LOG("[WARN] channel range conflict: start=%d num=%d Reject.\r\n",
              start_ch, ch_num);
        g_bind_reject_cnt++;                 /* ★ 契约 §2.3：拒绑计数（可见性） */
        return -1;
    }

    /* 6) 绑定表满 */
    if (g_binding_count >= MAX_BINDING_NUM) {
        BLE_LOG("[ERR] binding list full\r\n");
        g_bind_reject_cnt++;                 /* ★ 契约 §2.3：拒绑计数（可见性） */
        return -1;
    }

    /* 7) 写表 + 占通道 */
    scan_binding *b = &g_binding_list[g_binding_count];
    tmos_memset(b, 0, sizeof(scan_binding));
    tmos_memcpy(b->name, name, name_len);
    tmos_memcpy(b->mac,  mac,  6);
    b->Type         = type;
    b->host_num     = host_num;
    b->frist_ch_num = start_ch;
    b->ch_num       = ch_num;

    occupy_channels(mac, name, name_len, type, start_ch, ch_num);

    /* ★ 契约 §2.6：新增绑定 ⇒ 该设备（绑定索引）的 0x03 轮次状态清零：
     *   本轮还没收到过它的数据（fresh=0）、没有连续丢失（miss=0），
     *   与 g_binding_list[] 的索引严格对齐（绑定表只追加，索引不会搬移）。 */
    g_dev_fresh[g_binding_count] = 0;
    g_dev_miss[g_binding_count]  = 0;

    /* 名字快照基线：新绑定记录的快照 = 绑定时记录的名字，
     * 避免该设备在窗口内一直未广播时被误判为"名字变更"。 */
    tmos_memset(&g_name_snap[g_binding_count], 0, sizeof(name_snapshot_t));
    tmos_memcpy(g_name_snap[g_binding_count].mac, mac, 6);
    tmos_memcpy(g_name_snap[g_binding_count].name, name, name_len);
    g_name_snap[g_binding_count].name[name_len] = '\0';
    g_name_snap[g_binding_count].valid = 1;

    /* ★ 契约 §2.3：新增绑定成功后**只置落盘标志**，不在这里擦写 Flash ——
     *   本函数由 BLE 广播回调 SACN_DATA() 调用，Flash 擦除几十毫秒会阻塞协议栈。
     *   真正的 binding_store_save() 在 Usart3_task.c 的 1 秒事件里执行。
     *   幂等命中（上面的 idx >= 0 分支）不改动任何东西，因此不置位。 */
    g_store_dirty = 1;

    /* ★ 契约 §2.1：日志改打**通道号**（1 基），不再打 0 基下标，避免误导。
     *   例：SW_MG_1_1 → start_ch=0 ⇒ 打印 ch[1..1]。
     *   ★ 契约 §2.3：本函数只在 BLE 回调路径上执行 ⇒ 走 BLE_LOG（默认静音）。 */
    BLE_LOG("[BIND] OK: %.*s  MAC %02X:%02X:%02X:%02X:%02X:%02X  "
          "ch[%d..%d]  used=%d/%d\r\n",
          name_len, name, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
          start_ch + 1, start_ch + ch_num,
          count_used_channels(), MAX_CH_NUM);

    g_bind_ok_cnt++;                        /* ★ 契约 §2.3：成功绑定计数（1 秒摘要可见） */

    return (int)g_binding_count++;
}


/**
 * 按绑定记录把 payload 拆分写入对应通道。
 *
 * ★ 任务 A-1：入参从"一堆散字段 + 名字"改成**调用方已经定位好的绑定记录指针**，
 *   函数内部不再做任何绑定表查询；MAC 由调用方传入用于通道归属校验。
 *
 * 通用格式：
 *   2 字节 * ch_num + 1 字节电压 + 可选 1 字节通道个数
 *
 * 激光格式（TPYE_JG）：
 *   [0..1]  激光值   → 起始通道
 *   [2..3]  倾角 1   → 起始通道 + 1
 *   [4..5]  倾角 2   → 起始通道 + 2
 *   [6..7]  倾角 3   → 起始通道 + 3
 *   [8]     电压     → 所有通道共用
 *   倾角通道的 Type 写入 TPYE_QJ，激光通道保持 TPYE_JG。
 *
 * 打印：逐通道打印走 CH_LOG（任务 A-3，默认编译掉）；
 *       通道越界 / payload 过短 / ch_num 不匹配等结构性异常仍走 PRINT。
 *
 * ★ 契约 §2.6：返回值 = 本包是否**真正写入过至少一个通道**
 *   （通道属于该 MAC 且校验通过才写）。调用方据此置
 *   `g_dev_fresh[bidx] = 1; g_dev_miss[bidx] = 0;`
 *   —— "收到数据即视为该设备本轮已到、并把丢失计数归零"。
 *   payload 太短 / 通道越界 / 通道不属于该 MAC 都不算"收到数据"。
 */
static uint8_t fill_channel_data(const uint8_t *mac,
                              const scan_binding *b,
                              const uint8_t *payload, uint16_t payload_len,
                              int rssi)
{
    uint8_t     c;
    uint8_t     wrote    = 0;                  /* ★ 本包是否写入过通道（见函数头注释） */
    Sensor_Tpye type     = b->Type;            /* ★ 直接取用已定位的绑定记录 */
    uint8_t     start_ch = b->frist_ch_num;
    uint8_t     ch_num   = b->ch_num;
    uint8_t     volt_bad = 0;                  /* 本包电压异常去重标志（同一包只计一次） */

    if (ch_num == 0) return 0;
    if ((uint16_t)start_ch + ch_num > MAX_CH_NUM) {
        BLE_LOG("[WARN] ch out of range\r\n");
        return 0;
    }

    /* ================= 激光单独处理 ================= */
    if (type == TPYE_JG) {
        /* ★ 契约_传感器数据解析与显示标度 §1.4/§3.2：JG 载荷恒 9 字节
         *   [0..1] 激光(mm) | [2..5] 倾1..倾2 | [6..7] 倾3 | [8] 电压
         *   激光与三个倾角都是**大端**（改前误按小端解析 ⇒ 值高低字节颠倒）。 */
        if (payload == NULL || payload_len < 9) {
            BLE_LOG("[WARN] JG payload too short: %d, need >= 9\r\n", payload_len);
            return 0;
        }

        uint8_t  voltage   = payload[8];
        uint16_t laser_val = (uint16_t)((payload[0] << 8) | payload[1]);      /* ★ 大端 */

        /* ---------- 通道 0：激光本体 ---------- */
        {
            device_t *d = &CH_com_buf[start_ch];
            if (!d->valid || MEM_DIFF(d->mac, mac, 6)) {
                CH_LOG("[WARN] JG ch %d not owned by MAC, skip\r\n", start_ch);
            } else {
                d->CH_data      = laser_val;
                d->voltage      = voltage;
                d->rssi         = (uint8_t)rssi;
                d->data_re_flag = 1;
                d->Type         = TPYE_JG;
                g_round_mask   |= (uint32_t)(1UL << start_ch);   /* 「本轮已更新通道」置位 */
                volt_err_check(voltage, &volt_bad);              /* 电压异常计数（本包一次） */
                wrote           = 1;                             /* ★ 真正写入 */
                CH_LOG("[DATA] JG ch=%d laser=%u vol=%u rssi=%d\r\n",
                       start_ch, laser_val, voltage, rssi);
            }
        }

        /* ---------- 通道 1..3：倾角（仅当 ch_num==4 时） ---------- */
        for (c = 0; c < (ch_num - 1) && c < 3; c++) {
            uint8_t  ch  = (uint8_t)(start_ch + 1 + c);
            uint16_t val = (uint16_t)((payload[2 + c*2] << 8) |         /* ★ 大端（有符号量，显示期按 int16 处理） */
                                      payload[3 + c*2]);
            device_t *d  = &CH_com_buf[ch];

            if (!d->valid || MEM_DIFF(d->mac, mac, 6)) {
                CH_LOG("[WARN] QJ ch %d not owned by MAC, skip\r\n", ch);
                continue;
            }
            d->CH_data      = val;
            d->voltage      = voltage;
            d->rssi         = (uint8_t)rssi;
            d->data_re_flag = 1;
            d->Type         = TPYE_QJ;               /* ★ 倾角类型 */
            g_round_mask   |= (uint32_t)(1UL << ch); /* 「本轮已更新通道」置位 */
            volt_err_check(voltage, &volt_bad);      /* 电压异常计数（本包一次） */
            wrote           = 1;                     /* ★ 真正写入 */
            CH_LOG("[DATA] QJ ch=%d val=%u vol=%u rssi=%d\r\n",
                   ch, val, voltage, rssi);
        }
        return wrote;
    }

    /* ================= 通用传感器 =================
     * ★ 契约_传感器数据解析与显示标度 §1.1/§1.3/§3.1（2026-09-22 修正）：
     *   - 通道数据 **大端**：value = (payload[2c] << 8) | payload[2c+1]
     *     （改前按小端解析 ⇒ 所有值高低字节颠倒：`02 01` 读成 0x0102）；
     *   - **电压 = 数据区之后的最后一个字节** = payload[payload_len-1]，
     *     与通道数无关。WY 的载荷**恒 17 字节**（16 数据 + 1 电压），
     *     所以 WY2/WY4/WY6 的电压**不在** ch_num*2 处（改前会读到数据字节）；
     *   - 数据区 = payload[0 .. payload_len-2]，**取前 ch_num 个 u16** 依次对应通道 0..ch_num-1，
     *     多余的尾部字节忽略（WY 通道数变化时数据区总长不变）；
     *   - 已删除"包内通道数"字段（规格里没有它；对 WY 会把数据字节误读成通道数而假告警）。 */
    if (payload == NULL || payload_len < (uint16_t)(ch_num * 2 + 1)) {
        BLE_LOG("[WARN] payload too short: %d, need >= %d\r\n",
              payload_len, ch_num * 2 + 1);
        return 0;
    }

    uint8_t voltage = payload[payload_len - 1];      /* ★ 最后一个字节 */

    for (c = 0; c < ch_num; c++) {
        uint16_t value   = (uint16_t)((payload[c*2] << 8) |          /* ★ 大端 */
                                      payload[c*2+1]);
        uint8_t  channel = (uint8_t)(start_ch + c);

        device_t *d = &CH_com_buf[channel];

        if (!d->valid || MEM_DIFF(d->mac, mac, 6)) {
            CH_LOG("[WARN] ch %d not owned by MAC, skip\r\n", channel);
            continue;
        }
        d->CH_data      = value;
        d->voltage      = voltage;
        d->rssi         = (uint8_t)rssi;
        d->data_re_flag = 1;
        g_round_mask   |= (uint32_t)(1UL << channel);   /* 「本轮已更新通道」置位 */
        volt_err_check(voltage, &volt_bad);             /* 电压异常计数（本包一次） */
        wrote           = 1;                            /* ★ 真正写入 */
        CH_LOG("[DATA] ch=%d val=%u vol=%u rssi=%d\r\n",
               channel, value, voltage, rssi);
    }
    return wrote;
}
/**
 * ★ 契约 §2.6 第 2 步：数据**真正写入通道后**按**绑定索引**置位。
 *
 *     g_dev_fresh[bidx] = 1   本轮该设备已收到新数据
 *     g_dev_miss[bidx]  = 0   用户："设备重新出现就把错误次数归零"
 *
 * 本函数只做"0→1 / 清零"这类单字节写，**不做任何发送、不查表、不打印**，
 * 因此可以安全地在 BLE 广播回调（SACN_DATA）上下文里调用。
 */
static void dev_note_data(uint8_t bidx)
{
    if (bidx >= MAX_BINDING_NUM) return;
    g_dev_fresh[bidx] = 1;
    g_dev_miss[bidx]  = 0;
}

/* ==================================================================
 * 0x03「收齐本轮再应答」状态机（契约 §2.6）
 *   - observer_cmd03_request()：收到合法 0x03 请求时调用（主循环上下文）
 *   - observer_cmd03_sec_tick() / _ready() / _answered()：
 *     全部只在 Usart3_task.c 的 START_TIMER_EVT（1 秒事件）里调用；
 *     应答帧也在那里发送（**绝不在 BLE 回调里阻塞发送**）。
 * ================================================================== */

/* 受理一次 0x03 请求。返回 1 = 已受理进入等待；0 = 已有请求在途，本次忽略。 */
uint8_t observer_cmd03_request(void)
{
    if (g_rsp_pending) {
        /* ★ 契约 §2.6 第 1 条：在途则忽略（不重复入队、不回包），等当前轮结束。
         *   STM32 侧也有"在途不再发新请求"的守卫（契约 §3.1），双保险。 */
        return 0;
    }
    tmos_memset(g_dev_fresh, 0, sizeof(g_dev_fresh));   /* 清本轮"已收到"标记 */
    g_rsp_pending   = 1;
    g_rsp_start_sec = g_rsp_sec;                        /* 记起始秒（超时判定用） */
    return 1;
}

/* 1 秒事件：秒计数 +1（应答超时判定的唯一时基） */
void observer_cmd03_sec_tick(void)
{
    g_rsp_sec++;
}

/* 1 秒事件：现在是否应当应答？
 *   收齐：对每个 i < g_binding_count，若 g_dev_miss[i] < DEV_MISS_ROUNDS_MAX
 *         则要求 g_dev_fresh[i] == 1；全部满足 ⇒ 应答。
 *         （miss >= 3 的判损设备**不参与等待**，其通道沿用上次数据 —— 契约 §2.6 第 4 条。
 *          没有任何绑定时该条件为空集，恒真 ⇒ 下一次 1 秒 tick 立即应答，不会空等 30 秒。）
 *   超时：g_rsp_pending 且 (now - start) >= RESP_TIMEOUT_SEC ⇒ 用旧数据应答。 */
uint8_t observer_cmd03_ready(void)
{
    uint8_t i;

    if (!g_rsp_pending) return 0;

    /* ---- 超时（30 秒）---- */
    if ((uint32_t)(g_rsp_sec - g_rsp_start_sec) >= RESP_TIMEOUT_SEC)
        return 1;

    /* ---- 早应答窗口: 等不到全部设备就用已有数据应答, 缩短息屏获取时间 ---- */
    if ((uint32_t)(g_rsp_sec - g_rsp_start_sec) >= RESP_EARLY_SEC)
        return 1;

    /* ---- 收齐本轮 ---- */
    for (i = 0; i < g_binding_count && i < MAX_BINDING_NUM; i++) {
        if (g_dev_miss[i] >= DEV_MISS_ROUNDS_MAX) continue;   /* 判损设备不等它 */
        if (!g_dev_fresh[i]) return 0;                        /* 还有设备没到 */
    }
    return 1;
}

/* 应答已发出：用 fresh 更新 miss（fresh→0，否则 +1 且封顶 3），清 pending。
 * 封顶到 3 的意义：判损设备永远停在上限，既不继续涨、也不再阻塞等待。 */
void observer_cmd03_answered(void)
{
    uint8_t i;

    for (i = 0; i < g_binding_count && i < MAX_BINDING_NUM; i++) {
        if (g_dev_fresh[i]) {
            g_dev_miss[i] = 0;
        } else if (g_dev_miss[i] < DEV_MISS_ROUNDS_MAX) {
            g_dev_miss[i]++;
        }
    }
    g_rsp_pending = 0;
}

/**
 * ★ 任务 A-1/A-2：**已绑定 MAC 的数据填充**（只更新模式 SCAN_MODE_DATA 使用）。
 *
 * 与改前 add_device(bind_now=0) 的差别：
 *   - 绑定记录索引由调用方（SACN_DATA）在入口查好后传入，这里**不再查表**；
 *   - 完全删除"未绑定 MAC → PRINT([INFO] MAC not bound, ignore)"这条逐包打印
 *     （改由 SACN_DATA 在入口直接 return，未绑定设备的广播包 0 输出 0 解析）。
 *
 * @param bidx 入口已定位的绑定记录索引（>= 0）
 */
static int add_device_bound(int bidx, const uint8_t *addr,
                            const uint8_t *payload, uint16_t payload_len,
                            int Rssi)
{
    if (bidx < 0 || bidx >= (int)g_binding_count) return -1;
    if (payload == NULL || payload_len == 0) return bidx;

    if (fill_channel_data(addr, &g_binding_list[bidx], payload, payload_len, Rssi))
        dev_note_data((uint8_t)bidx);            /* §2.6：标记该设备本轮已到 */
    return bidx;
}

/**
 * ★ 绑定模式（SCAN_MODE_BIND）统一入口。
 *
 *   - 必须有名字（来自名字包）；
 *   - 解析名字 → 若是激光，用 payload 决定 ch_num
 *     （三倾角全 0xFDFD → 1，否则 → 4）；
 *   - 调用 add_binding（内部完成 MAC/名字/通道/表满 全部校验），
 *     known_idx 即入口那次查表的结果，add_binding 内部不再重复查表；
 *   - 本包若带 payload，则填充数据（同样不再查表）。
 *
 * @param known_idx 入口 find_binding_by_mac() 的结果：>=0 已绑定，-1 未绑定
 */
static int add_device_bind(uint8_t *addr,
                           const char *name, uint8_t name_len,
                           int Rssi, int known_idx,
                           const uint8_t *payload, uint16_t payload_len)
{
    int bidx;

    /* ---------- 绑定模式：必须有名字 ---------- */
    if (name == NULL || name_len == 0)
        return -1;

    Sensor_Tpye type;
    uint8_t     host_num, start_ch, ch_num;

    if (!parse_sensor_name(name, name_len, &type, &host_num, &start_ch, &ch_num)) {
        BLE_LOG("[WARN] invalid sensor name: %.*s\r\n", name_len, name);
        return -1;
    }

    /* ---------- 激光特殊：ch_num 由 payload 决定 ---------- */
    if (type == TPYE_JG) {
        uint8_t jg_ch = detect_jg_channels(payload, payload_len);
        if (jg_ch == 0) {
            BLE_LOG("[INFO] JG waiting for payload to decide ch_num\r\n");
            return -1;                              /* 数据未到，等下一包 */
        }
        if (jg_ch != ch_num) {
            BLE_LOG("[INFO] JG ch_num from payload: %d (name said %d)\r\n",
                  jg_ch, ch_num);
        }
        ch_num = jg_ch;                             /* ★ payload 权威 */
    }

    bidx = add_binding(addr, name, name_len, type, host_num, start_ch, ch_num,
                       known_idx);
    if (bidx < 0) return -1;

    /* 本包同时带 payload 就顺手填一次（索引已知，不再查表） */
    if (payload && payload_len > 0) {
        if (fill_channel_data(addr, &g_binding_list[bidx], payload, payload_len, Rssi))
            dev_note_data((uint8_t)bidx);        /* §2.6：标记该设备本轮已到 */
    }
    return bidx;
}

/* ==================================================================
 * 解绑 / 开机恢复（任务 B、D 在 observer 侧的 RAM 操作）
 * ================================================================== */

/**
 * ★ 任务 B：解绑全部 —— 清空 RAM 中**所有**绑定相关状态。
 *
 *   清 g_binding_list[] / g_binding_count / CH_com_buf[] / g_name_snap[] /
 *      g_name_err_list[] / g_name_err_count（含环形写指针 g_name_err_next）
 *      + 0x03 轮次状态 g_dev_fresh[] / g_dev_miss[]（契约 §2.6 第 5 条）
 *      + 落盘标志 g_store_dirty（契约 §2.3）。
 *
 *   注意：
 *     - 本函数只动 RAM，**不**碰 Flash；Flash 里已保存的记录由调用方
 *       （Usart3_task.c 的 case 0x04）用 binding_store_clear() 一并清除；
 *     - 名字快照与错误记录必须一起清（见旧报告遗留项 #13：快照表按"下标"与
 *       绑定表对齐，绑定表被清空后若不同步清快照，重启绑定会出现槽位错位）；
 *     - 清空后 CH_com_buf[].valid 全为 0 ⇒ 不再采集任何设备的通道数据。
 *
 * @return 1 = 清空前存在绑定或通道占用；0 = 本来就没有绑定
 */
uint8_t observer_clear_all_bindings(void)
{
    uint8_t had = 0;

    if (g_binding_count > 0)                 had = 1;
    if (count_used_channels() > 0)           had = 1;

    tmos_memset(g_binding_list,  0, sizeof(g_binding_list));
    g_binding_count = 0;
    tmos_memset(CH_com_buf,      0, sizeof(CH_com_buf));
    tmos_memset(g_name_snap,     0, sizeof(g_name_snap));
    tmos_memset(g_name_err_list, 0, sizeof(g_name_err_list));
    g_name_err_count = 0;
    g_name_err_next  = 0;
    /* 功能清单新增状态：电压异常计数、「一轮采集完成」位图、
     * 最近扫描名称缓存都随绑定一起清（解绑/恢复出厂后不应保留旧统计）。 */
    g_volt_err_count = 0;
    g_round_mask     = 0;
    tmos_memset(g_scan_name_cache, 0, sizeof(g_scan_name_cache));
    /* ★ 契约 §2.6 第 5 条：解绑(0x04)/恢复出厂(0x07)/开机恢复都必须把
     *   "本轮已收到"与"连续丢失轮数"清零（下标已与新的空绑定表对齐）。
     *   g_rsp_pending 故意**不清**：若有 0x03 正在等待，下一个 1 秒 tick 因
     *   绑定数为 0 会立即"收齐"应答（回全 0 通道），不会拖到 30 秒超时。 */
    tmos_memset(g_dev_fresh,     0, sizeof(g_dev_fresh));
    tmos_memset(g_dev_miss,      0, sizeof(g_dev_miss));
    /* ★ 契约 §2.3：绑定已被清空（Flash 也由调用方 binding_store_clear() 擦掉），
     *   撤销可能挂起的落盘请求，避免解绑后又被存回去。 */
    g_store_dirty = 0;

    return had;
}

/**
 * ★ 任务 D：开机从 Flash 恢复时使用 —— 按一条**已通过 Flash 校验**的记录
 *   追加绑定并占用通道。等价于 add_binding() 的"第 7 步"，但：
 *     - 不查名字占用（Flash 里的记录本就是互不冲突的绑定结果）；
 *     - 每条都要重新做范围/冲突校验，任何不合法的记录被拒绝而不是带病运行；
 *     - 同步建立名字快照基线（与 add_binding 一致）。
 *
 * @return 0 成功；-1 拒绝（字段非法 / 通道冲突 / 绑定表满）
 */
int observer_restore_binding(const uint8_t *mac, const char *name, uint8_t name_len,
                             Sensor_Tpye type, uint8_t host_num,
                             uint8_t start_ch, uint8_t ch_num)
{
    scan_binding *b;

    if (mac == NULL || name == NULL)              return -1;
    if (g_binding_count >= MAX_BINDING_NUM)       return -1;   /* 绑定表满 */
    if (ch_num == 0)                              return -1;   /* 通道数为 0 */
    if ((uint16_t)start_ch + ch_num > MAX_CH_NUM) return -1;   /* 通道越界 */
    if (type == TPYE_NONE || type >= TPYE_END)    return -1;   /* 类型非法 */
    if (!channels_available(start_ch, ch_num, mac)) return -1; /* 区间冲突 */

    if (name_len > (MAX_NAME_LEN - 1)) name_len = (MAX_NAME_LEN - 1);

    b = &g_binding_list[g_binding_count];
    tmos_memset(b, 0, sizeof(scan_binding));
    tmos_memcpy(b->mac,  mac,  6);
    tmos_memcpy(b->name, name, name_len);
    b->Type         = type;
    b->host_num     = host_num;
    b->frist_ch_num = start_ch;
    b->ch_num       = ch_num;

    occupy_channels(mac, name, name_len, type, start_ch, ch_num);

    /* ★ 契约 §2.6 第 5 条：开机恢复的每一条也把 0x03 轮次状态清零，
     *   与 g_binding_list[] 的索引严格对齐（新槽位 = fresh 0 / miss 0）。 */
    g_dev_fresh[g_binding_count] = 0;
    g_dev_miss[g_binding_count]  = 0;

    /* 名字快照基线：与 add_binding() 第 7 步保持一致 */
    tmos_memset(&g_name_snap[g_binding_count], 0, sizeof(name_snapshot_t));
    tmos_memcpy(g_name_snap[g_binding_count].mac, mac, 6);
    tmos_memcpy(g_name_snap[g_binding_count].name, name, name_len);
    g_name_snap[g_binding_count].name[name_len] = '\0';
    g_name_snap[g_binding_count].valid = 1;

    PRINT("[LOAD] binding[%d] %.*s MAC %02X:%02X:%02X:%02X:%02X:%02X "
          "ch[%d..%d]\r\n",
          g_binding_count, name_len, name,
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
          start_ch, start_ch + ch_num - 1);

    g_binding_count++;
    return 0;
}

/* ==================================================================
 * 绑定名变更监测（10 分钟巡检）
 *
 * 背景：需求是"绑定之后每 10 分钟检查一次：正在扫描时，广播包里同一 MAC
 *       （已被绑定）的名字若被改变，就告警并存储错误记录"。
 * 实现方式：某一瞬间的广播拿不到 → 采用"窗口内采集 + 到点汇总"：
 *   1) 扫描回调（SACN_DATA）里对**已绑定 MAC** 持续记录最近一次广播的名字；
 *   2) 每 600 秒（1s 事件累计）汇总一次：遍历绑定表，比较"绑定时记录的名字"
 *      与"窗口内最近捕获的名字"，不同则写一条 RAM 错误记录 + 串口打印；
 *   3) 汇总后清空快照，开始下一个 10 分钟窗口（周期性，不会只跑一次）。
 * ================================================================== */

/**
 * 记录"已绑定 MAC"最近一次广播里的名字（名字快照）。
 *
 * ★ 任务 A-2：本函数**只在已绑定时被调用**（调用方 SACN_DATA 用入口那次
 *   find_binding_by_mac() 的结果判断），索引直接传入，因此内部**不再查表**。
 *   未绑定的 MAC 根本不会走到这里（数据模式下入口就已 return）。
 *
 * 只做捕获：不修改任何绑定记录，也不影响数据填充。
 * 绑定模式（SCAN_MODE_BIND）与只更新模式（SCAN_MODE_DATA）的广播包都会触发。
 */
static void name_capture_by_idx(int idx, const uint8_t *mac,
                                const char *name, uint8_t name_len)
{
    name_snapshot_t *s;

    if (idx < 0 || idx >= MAX_BINDING_NUM) return;
    if (mac == NULL || name == NULL || name_len == 0) return;

    if (name_len > (MAX_NAME_LEN - 1)) name_len = (MAX_NAME_LEN - 1);

    s = &g_name_snap[idx];
    tmos_memcpy(s->mac, mac, 6);
    tmos_memcpy(s->name, name, name_len);
    s->name[name_len] = '\0';                   /* 留 '\0' */
    s->valid = 1;
}

/**
 * 写入/更新一条"名字变更"错误记录（RAM 定长表）。
 *
 * 同一 MAC + 同一对（旧名,新名）视为同一条记录：只累加检测次数并刷新时间；
 * 否则作为新记录写入，表满时用环形索引覆盖最旧的一条。
 * 返回该记录在表中的下标。
 */
static uint8_t name_err_record(const uint8_t *mac,
                               const uint8_t *old_name, uint8_t old_len,
                               const uint8_t *new_name, uint8_t new_len)
{
    uint8_t i;
    name_change_err_t *e;

    if (old_len > (MAX_NAME_LEN - 1)) old_len = (MAX_NAME_LEN - 1);
    if (new_len > (MAX_NAME_LEN - 1)) new_len = (MAX_NAME_LEN - 1);

    /* 已有同一条记录 → 只更新计数与时间 */
    for (i = 0; i < g_name_err_count; i++) {
        e = &g_name_err_list[i];
        if (MEM_SAME(e->mac, mac, 6) &&
            tmos_strlen((char*)e->old_name) == old_len &&
            tmos_strlen((char*)e->new_name) == new_len &&
            MEM_SAME(e->old_name, old_name, old_len) &&
            MEM_SAME(e->new_name, new_name, new_len))
        {
            e->hit_cnt++;
            e->last_sec = g_name_chk_sec;
            return i;
        }
    }

    /* 新记录：环形写入（满了覆盖最旧） */
    i = g_name_err_next;
    e = &g_name_err_list[i];
    tmos_memset(e, 0, sizeof(name_change_err_t));
    tmos_memcpy(e->mac, mac, 6);
    tmos_memcpy(e->old_name, old_name, old_len);
    e->old_name[old_len] = '\0';
    tmos_memcpy(e->new_name, new_name, new_len);
    e->new_name[new_len] = '\0';
    e->hit_cnt   = 1;
    e->first_sec = g_name_chk_sec;
    e->last_sec  = g_name_chk_sec;

    g_name_err_next = (uint8_t)((i + 1) % NAME_CHK_ERR_NUM);
    if (g_name_err_count < NAME_CHK_ERR_NUM) g_name_err_count++;

    return i;
}

/**
 * 打印 RAM 错误记录表全部内容。
 */
void name_change_dump(void)
{
    uint8_t i;

    PRINT("[NAME-CHK] ---- name-change error table (%d/%d, ring overwrite oldest) ----\r\n",
          g_name_err_count, NAME_CHK_ERR_NUM);

    for (i = 0; i < g_name_err_count; i++) {
        name_change_err_t *e = &g_name_err_list[i];

        PRINT("[NAME-CHK] rec[%d] MAC %02X:%02X:%02X:%02X:%02X:%02X "
              "old=%s new=%s hit=%u first=%us last=%us\r\n",
              i,
              e->mac[0],e->mac[1],e->mac[2],e->mac[3],e->mac[4],e->mac[5],
              e->old_name, e->new_name,
              (unsigned int)e->hit_cnt,
              (unsigned int)e->first_sec,
              (unsigned int)e->last_sec);
    }
}

/**
 * 走一轮"绑定名变更"检查。
 *
 * 前提：此刻正在扫描。代码里可用的判据有两个：
 *   - SW_SCAN_STATE == 0 表示"不扫描"（该变量注释：0:不扫描 ;2:扫描名字 ;3:扫描数据;4:全扫描）；
 *   - g_scan_enable == 0 表示广播解析总开关被关闭。
 *     ★ P2 修复：原来这里用的是 menu_rank != 2（"只有组网页才解析广播"），
 *       广播解析已与当前显示页面解耦，该判据不再成立，改为 g_scan_enable。
 * 任一不满足即跳过本轮并打印说明；此时**保留快照不清空**，
 * 让下一个 10 分钟窗口继续累计（避免因短暂停止扫描而漏掉一次变更）。
 */
static void name_change_check_round(void)
{
    uint8_t i;
    uint8_t captured = 0;
    uint8_t changed  = 0;

    g_name_chk_round++;

    if (SW_SCAN_STATE == 0) {
        PRINT("[NAME-CHK] round %u SKIP: not scanning (SW_SCAN_STATE=0)\r\n",
              (unsigned int)g_name_chk_round);
        return;
    }
    if (!g_scan_enable) {
        PRINT("[NAME-CHK] round %u SKIP: scan disabled (g_scan_enable=0)\r\n",
              (unsigned int)g_name_chk_round);
        return;
    }

    for (i = 0; i < g_binding_count && i < MAX_BINDING_NUM; i++) {
        scan_binding    *b = &g_binding_list[i];
        name_snapshot_t *s = &g_name_snap[i];
        uint8_t          old_len, new_len, ridx;

        if (!s->valid) continue;                    /* 窗口内没收到该 MAC 的广播 */
        if (MEM_DIFF(s->mac, b->mac, 6)) continue;  /* 防御：槽位错位则不比较 */
        captured++;

        old_len = (uint8_t)tmos_strlen((char*)b->name);   /* 绑定时记录的名字 */
        new_len = (uint8_t)tmos_strlen((char*)s->name);   /* 窗口内最近捕获的名字 */

        if (old_len == new_len && MEM_SAME(b->name, s->name, old_len))
            continue;                               /* 相同 → 不产生记录 */

        /* 不同 → 一条错误记录 + 打印（含 MAC、旧名、新名、时间、轮次） */
        ridx = name_err_record(b->mac, b->name, old_len, s->name, new_len);
        changed++;

        PRINT("[NAME-CHK][ERR] name changed! MAC %02X:%02X:%02X:%02X:%02X:%02X "
              "old=%s new=%s sec=%u round=%u rec[%d]\r\n",
              b->mac[0],b->mac[1],b->mac[2],b->mac[3],b->mac[4],b->mac[5],
              b->name, s->name,
              (unsigned int)g_name_chk_sec,
              (unsigned int)g_name_chk_round, ridx);
    }

    PRINT("[NAME-CHK] round %u done: bound=%d captured=%d changed=%d err_rec=%d/%d\r\n",
          (unsigned int)g_name_chk_round, g_binding_count, captured, changed,
          g_name_err_count, NAME_CHK_ERR_NUM);

    if (changed) name_change_dump();

    /* 清空快照，开始下一个 10 分钟窗口 */
    tmos_memset(g_name_snap, 0, sizeof(g_name_snap));
}

/**
 * 名字变更监测的 1 秒心跳（由 Usart3_task.c 的 1s 事件调用）。
 *
 * 累加到 NAME_CHK_PERIOD_SEC（600 秒 = 10 分钟）执行一轮检查；
 * 到点后无条件把计数清零，因此是**周期性**任务（每 10 分钟一轮），不会只跑一次。
 */
void name_change_monitor_tick(void)
{
    g_name_chk_sec++;
    g_name_chk_pending++;

    if (g_name_chk_pending < NAME_CHK_PERIOD_SEC) return;

    g_name_chk_pending = 0;         /* 自动重新计时：下一轮 10 分钟后 */
    name_change_check_round();
}

/* ==================================================================
 * ★ 契约 §2.2 消费者侧：**单条广播包的完整解析逻辑**
 *
 * 本函数是"改动前 SACN_DATA() 的全部原有逻辑"的**原样搬迁**（行为逐位一致）：
 *   1) 入口只查一次绑定表（bidx 全程复用）；
 *   2) 未绑定 + 只更新模式 → 立即 return（0 次后续查表 / 0 字节输出）；
 *   3) 已绑定 → 解析名字并更新名字快照（10 分钟名字变更巡检用）；
 *   4) 绑定模式（SCAN_MODE_BIND）→ scan_name_cache_add + payload 解析 +
 *      add_device_bind（发现新设备 / 绑定 / 拒绑规则全部不变）；
 *   5) 只更新模式（SCAN_MODE_DATA）→ payload 解析 + add_device_bound（填通道）。
 *
 * 运行上下文：**主循环**（usartTaskId 的 TMOS 任务），由 observer_adv_drain()
 * 按 FIFO 顺序逐条调用 —— 不在 BLE 回调里。回调只负责入队（见 SACN_DATA）。
 * ================================================================== */
static void sacn_process_packet(uint8_t *addr, uint8_t EVENT, uint8_t *data, uint16_t data_len, int Rssi)
{
    char     name[MAX_NAME_LEN];
    uint8_t  name_len    = 0;
    uint8_t  payload[MAX_PAYLOAD_LEN];
    uint8_t  payload_len = sizeof(payload);
    uint8_t  has_name;
    uint8_t  has_payload;
    int      bidx;

    /* ★ 两包各自解析，互不依赖；解析模式只看 g_scan_mode，与页面无关 */

    /* ★ 契约 §2.2：g_scan_enable 总开关的判定在**生产者** SACN_DATA() 里
     *   （关掉扫描时广播包根本不会入队），这里不再重复判定。 */

    /*
     * ★ 任务 A-1（本次核心优化）：**入口只查一次绑定表**，结果 bidx 全程复用。
     *
     *   改前（每个广播包，无论绑定与否）：
     *       parse_device_name_from_adv → name_capture（查表 1 次）
     *       → parse_payload_from_adv → add_device（查表 1 次）
     *       → 未绑定再 PRINT("[INFO] MAC not bound, ignore")  ← 阻塞串口
     *   共 2 次线性查表 + 1 条阻塞打印。
     *
     *   改后：
     *       未绑定 且 SCAN_MODE_DATA → **立即 return**：不解析名字、不解析 payload、
     *                                  不打印、不写任何通道（0 次后续查表 / 0 字节输出）；
     *       未绑定 且 SCAN_MODE_BIND → 继续往下（绑定模式必须解析名字去发现新设备）；
     *       已绑定                  → bidx 一路复用到"名字快照"和"数据填充"，
     *                                 两处都不再查表（add_binding 也接收该索引）。
     */
    bidx = find_binding_by_mac(addr);
    if (bidx < 0 && g_scan_mode == SCAN_MODE_DATA) return;

    /*
     * ★ 名字解析 + 捕获：
     *   - 已绑定（bidx >= 0）：解析出来只为更新"名字快照"，供 10 分钟巡检比对，
     *     行为与改前完全一致（改前由 name_capture 内部查表判未绑定后 return）；
     *   - 绑定模式且未绑定：解析名字是发现新设备的必要条件，必须解析；
     *   - 只更新模式且未绑定：上面已经 return，**这里根本不会执行**。
     *   注：名字段(0x08/0x09)与厂商数据段(0xFF)通常在同一广播包里，
     *       本包解析不到名字就不捕获，等下一包。
     */
    name_len = 0;
    has_name = parse_device_name_from_adv(data, (uint8_t)data_len,
                                          name, &name_len);
    if (has_name && bidx >= 0)
        name_capture_by_idx(bidx, addr, name, name_len);

    if (g_scan_mode == SCAN_MODE_BIND)
    {        /* 绑定 + 更新 */
        if (!has_name) return;               /* 等名字包 */

        /* ★ 功能清单：「设备绑定 → 绑定设备」子页要显示"最近扫描到的蓝牙名称"。
         *   只在绑定模式（即停留在该二级子页）里记录，新名字置顶、同 MAC 去重。 */
        scan_name_cache_add(addr, name, name_len);

        /*
         * 名字段(0x08/0x09)和厂商数据段(0xFF)通常在同一个广播包里，
         * 所以这里必须真正解析一次 payload：
         *   - 不解析就传未初始化栈缓冲，会按栈垃圾决定激光(JG)通道数是 1 还是 4；
         *   - 解析不到就按"本包无数据"处理：只完成绑定，等下一包再填数据。
         */
        payload_len = sizeof(payload);
        has_payload = parse_payload_from_adv(data, (uint8_t)data_len,
                                             payload, &payload_len);

        /* 逐包打印走 CH_LOG（任务 A-3：默认编译掉，绑定模式 30 秒窗口内也不刷屏） */
        CH_LOG("EVENT=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X Rssi=%d\r\n",
               EVENT, addr[0],addr[1],addr[2],addr[3],addr[4],addr[5], Rssi);
        CH_LOG("Name = %s  payload_len = %d\r\n",
               name, has_payload ? payload_len : 0);

        add_device_bind(addr, name, name_len, Rssi, bidx,
                        has_payload ? payload : NULL,
                        has_payload ? payload_len : 0);
    }
    else
    { /* SCAN_MODE_DATA：只更新（走到这里必然 bidx >= 0 且已绑定） */
        has_payload = parse_payload_from_adv(data, (uint8_t)data_len,
                                             payload, &payload_len);
        if (!has_payload) return;            /* 等数据包 */
        CH_LOG("Payload len = %d\r\n", payload_len);
        add_device_bound(bidx, addr, payload, payload_len, Rssi);
    }

}

/**
 * ★ 契约 §2.2 消费者：按 FIFO 顺序排空环形队列，每次最多处理 max_items 条。
 *
 * 调用点：Usart3_task.c 的 START_IO_EVT（10ms 事件）→ observer_adv_drain(16);
 *   ⇒ 容量 16 条/10ms = **1600 包/秒**；最坏 1 包/ms 的广播密度留 60% 余量；
 *     单拍最坏 ≈ 16 × 50us = 0.8ms，占 10ms 的 8%（不新增更高频的 TMOS 事件）。
 * 顺序性：严格 FIFO，不重排、不合并（同 MAC 的多次广播按到达顺序覆盖通道数据，
 *         最终结果与改动前"逐包同步处理"一致，只是最多晚一拍 ≈10ms 落地）。
 * 积压统计：进入函数时先采一次"当前积压"更新 g_adv_q_max（历史最大），
 *           供现场判断"是否长期排不完"；该读取也在主循环上下文。
 */
void observer_adv_drain(uint16_t max_items)
{
    adv_q_entry_t *e;
    uint16_t       n = 0;
    uint8_t        backlog;

    if (!g_scan_enable)
    {
        g_adv_q_head = g_adv_q_tail;
        return;
    }

    backlog = (uint8_t)((g_adv_q_tail - g_adv_q_head) & (uint8_t)(ADV_Q_DEPTH - 1));
    if (backlog > g_adv_q_max) g_adv_q_max = backlog;

    while (n < max_items && g_adv_q_head != g_adv_q_tail)
    {
        e = &g_adv_q[g_adv_q_head];
        /* ★ 先处理、后推进 head：生产者绝不会覆盖正在处理的条目 */
        sacn_process_packet(e->addr, e->event_type, e->data,
                            (uint16_t)e->len, (int)e->rssi);
        g_adv_q_head = (uint8_t)((g_adv_q_head + 1) & (uint8_t)(ADV_Q_DEPTH - 1));
        n++;
    }
}

/**
 * ★ 契约 §2.3 + §2.1 补充：1 秒事件里的**可见性摘要**与**扫描看门狗**。
 *
 * 只在主循环上下文（Usart3_task.c 的 START_TIMER_EVT）调用，不新增任何定时器、
 * 不在中断里做任何事：
 *   1) 消费 g_adv_seen，维护"距上次收到广播包的秒数" g_adv_quiet_sec；
 *   2) 连续 >= ADV_QUIET_RESTART_SEC(10) 秒没有任何广播包且扫描开着 ⇒ 重新
 *      StartDiscovery —— 兜底"TGAP_DISC_SCAN=0 的语义未被真机实测"这一风险
 *      （万一该栈把 0 当成"不扫描"，这一层把功能救回来）。
 *      按 SDK 返回码三分类计数（语义已按 CH58xBLE_LIB.h 定死）：
 *        SUCCESS                          → g_scan_restart_cnt：之前确实是停的，被救回来了
 *        bleAlreadyInRequestedMode(0x11)  → g_scan_busy_cnt  ：扫描本来就在跑（正常）
 *        其它（如 bleIncorrectMode 0x12）  → g_scan_err_cnt   ：真异常
 *      ★ busy 会随"现场没有广播源"每 10 秒 +1，属**预期**增长，不是故障。
 *   3) 任一计数变化 ⇒ 打印一行摘要（主循环上下文；关掉 BLE_LOG 明细也不瞎）：
 *      [BLE] adv: q_max=%u drop=%u scan rst=%u busy=%u err=%u | bind ok=%u reject=%u
 */
void observer_ble_stat_tick(void)
{
    bStatus_t st;

    /* ---- 1) 静默计时 ---- */
    if (g_adv_seen)
    {
        g_adv_seen      = 0;
        g_adv_quiet_sec = 0;                 /* 收到了广播包 ⇒ 静默归零 */
    }
    else if (g_scan_enable && g_adv_quiet_sec < 0xFFFF)
    {
        g_adv_quiet_sec++;
    }

    /* ---- 2) 扫描看门狗（纯 1 秒事件内的一个判断） ---- */
    if (g_scan_enable && g_adv_quiet_sec >= ADV_QUIET_RESTART_SEC)
    {
        g_adv_quiet_sec = 0;                 /* 重启后重新计时，避免每秒都重启 */
        st = GAPRole_ObserverStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST);
        if (st == SUCCESS)                        g_scan_restart_cnt++;
        else if (st == bleAlreadyInRequestedMode) g_scan_busy_cnt++;
        else                                      g_scan_err_cnt++;
    }

    /* ---- 3) 计数摘要：任一计数变化才打印一次（避免每秒刷屏） ---- */
    if (g_bind_ok_cnt != s_last_ok || g_bind_reject_cnt != s_last_rej ||
        g_adv_q_drop  != s_last_drop || g_scan_restart_cnt != s_last_rst ||
        g_scan_busy_cnt != s_last_busy || g_scan_err_cnt != s_last_err ||
        g_adv_q_max   != s_last_qmax)
    {
        s_last_ok   = g_bind_ok_cnt;
        s_last_rej  = g_bind_reject_cnt;
        s_last_drop = g_adv_q_drop;
        s_last_rst  = g_scan_restart_cnt;
        s_last_busy = g_scan_busy_cnt;
        s_last_err  = g_scan_err_cnt;
        s_last_qmax = g_adv_q_max;

        PRINT("[BLE] adv: q_max=%u drop=%u scan rst=%u busy=%u err=%u | bind ok=%u reject=%u\r\n",
              (unsigned int)g_adv_q_max, (unsigned int)g_adv_q_drop,
              (unsigned int)g_scan_restart_cnt, (unsigned int)g_scan_busy_cnt,
              (unsigned int)g_scan_err_cnt,
              (unsigned int)g_bind_ok_cnt, (unsigned int)g_bind_reject_cnt);
    }
}

/* ==================================================================
 * ★ 第 9 轮新增：蓝牙扫描诊断（让"扫描到底有没有在跑"直接可见）
 * ------------------------------------------------------------------
 * 背景：原来的 [BLE] adv 摘要**只在计数变化时**才打印；而"空中有广播包在到"
 *       恰恰会把静默计时归零 ⇒ 计数不变 ⇒ 什么都不打印。于是"扫描正常"和
 *       "扫描死了"在日志上看起来**一模一样**，根本无法区分。
 * 本函数每 BLE_DIAG_PERIOD_SEC 秒**无条件**打印一行完整扫描状态，并把
 * 「最近扫描到的蓝牙名称」缓存一并打出来 —— 一眼就能判断：
 *   · rx 在涨        -> 空中确实有广播包，射频/天线正常
 *   · rx=0 且 rst 涨 -> 一个包都收不到（扫描停了 / 天线 / 频段问题）
 *   · cache 里有名字  -> 名字包能解出来，顺便看是不是 SW_ 格式
 *   · reject 在涨    -> 名字解到了但被拒（格式或通道号越界）
 * ================================================================== */
/* ★ 本轮改动：默认 **关闭** 扫描诊断打印。
   为什么：BLE_DIAG_LOG 打开时每 5 秒会阻塞打印约 40ms（见下面的使用点），
   而蓝牙扫描/绑定、屏幕重绘、串口收发**共用同一个 Main_Circulation() 协程**，
   没有优先级隔离 —— 这段打印会直接推迟 BLE 协议栈的广播/连接事件，
   是"开蓝牙时动作变慢"的一个明确来源（详见
   《串口协议与蓝牙影响分析.md》的 C 节：整屏重绘 8~16ms、Flash 擦 20~70ms、
   诊断打印 ~40ms/5s，其中只有这一条是**纯调试开销、可以零代价去掉**）。
   需要现场诊断时把 1 改回来重新编译即可（打印内容与原来完全一致）。 */
#define BLE_DIAG_LOG          0     /* 0 = 关闭扫描诊断打印；1 = 打开（调试用） */
#define BLE_DIAG_PERIOD_SEC   5     /* 每隔多少秒打印一行 */

volatile uint32_t g_adv_rx_cnt = 0; /* 收到的广播包总数（纯诊断，不参与逻辑） */

/* Low-rate main-loop diagnostics; no reset, flash or serial command side effects.
 * rx counts accepted broadcasts, not completed sensor measurements. */
void observer_power_diag_tick(void)
{
    static uint8_t seconds = 0;
    if (++seconds < 10u) return;
    seconds = 0;
    PRINT("[BLE-PWR] known=%u state=%u en=%u ready=%u frames=%lu rx=%lu\r\n",
          (unsigned int)g_collection_state_seen,
          (unsigned int)g_collection_state, (unsigned int)g_scan_enable,
          (unsigned int)g_observer_ready,
          (unsigned long)g_collection_state_frames, (unsigned long)g_adv_rx_cnt);
}

void observer_ble_diag_tick(void)
{
#if BLE_DIAG_LOG
    static uint16_t s_sec = 0;
    uint8_t i, n;

    if (++s_sec < BLE_DIAG_PERIOD_SEC) return;
    s_sec = 0;

    PRINT("[BLE-DIAG] en=%u scanmode=%u SWSTATE=%u | rx=%lu quiet=%u qmax=%u drop=%u rst=%u busy=%u err=%u | bindok=%u reject=%u cache=%u\r\n",
          (unsigned int)g_scan_enable,
          (unsigned int)g_scan_mode,
          (unsigned int)SW_SCAN_STATE,
          (unsigned long)g_adv_rx_cnt,
          (unsigned int)g_adv_quiet_sec,
          (unsigned int)g_adv_q_max,
          (unsigned int)g_adv_q_drop,
          (unsigned int)g_scan_restart_cnt,
          (unsigned int)g_scan_busy_cnt,
          (unsigned int)g_scan_err_cnt,
          (unsigned int)g_bind_ok_cnt,
          (unsigned int)g_bind_reject_cnt,
          (unsigned int)scan_name_cache_count());

    n = scan_name_cache_count();
    if (n > SCAN_NAME_CACHE_NUM) n = SCAN_NAME_CACHE_NUM;
    for (i = 0; i < n; i++)
    {
        PRINT("    [NAME] %02X:%02X:%02X:%02X:%02X:%02X  '%s'\r\n",
              g_scan_name_cache[i].mac[0], g_scan_name_cache[i].mac[1],
              g_scan_name_cache[i].mac[2], g_scan_name_cache[i].mac[3],
              g_scan_name_cache[i].mac[4], g_scan_name_cache[i].mac[5],
              (const char *)g_scan_name_cache[i].name);
    }
#endif
}


/**
 * ★ 契约 §2.2 生产者：BLE 广播回调（SACN_DATA）**只做入队**，回调内零阻塞。
 *
 * 成本：一次 <=64B 的 tmos_memcpy + 几个字节的字段写入，**O(1)**；
 *       没有查绑定表、没有解析名字/payload、没有填通道、没有绑定、
 *       没有写 Flash、没有任何 PRINT。⇒ 协议栈回调上下文里的最坏耗时
 *       从"线性查表 + 有界解析 + 可能的阻塞打印(3.5~8ms)"降为几十微秒内的定长拷贝。
 *
 * 队列满 ⇒ g_adv_q_drop++ 并**丢弃最新**一条（保留队列中更早的数据，
 * 维持"先到先处理"的 FIFO 语义；丢最新也使 drop 计数直接反映过载程度）。
 */
__HIGH_CODE
void SACN_DATA(uint8_t *addr, uint8_t EVENT, uint8_t *data, uint16_t data_len, int Rssi)
{
    adv_q_entry_t *e;
    uint8_t        next;
    uint16_t       len;

    /* ★ P2 修复（保留）：预留总开关，关闭时连入队都不做 */
    if (!g_scan_enable) return;

    g_adv_rx_cnt++;   /* ★ 第 9 轮：广播包计数（诊断） */

    next = (uint8_t)((g_adv_q_tail + 1) & (uint8_t)(ADV_Q_DEPTH - 1));
    if (next == g_adv_q_head)                /* 留一个空槽判满 ⇒ 队列满 */
    {
        if (g_adv_q_drop < 0xFFFF) g_adv_q_drop++;
        return;                              /* 丢最新，直接返回 */
    }

    e = &g_adv_q[g_adv_q_tail];
    tmos_memcpy(e->addr, addr, 6);           /* 广播地址恒为 6 字节 */

    len = data_len;
    if (data == NULL)         len = 0;       /* 防御：无数据段则长度记 0 */
    if (len > ADV_Q_DATA_MAX) len = ADV_Q_DATA_MAX;   /* 截断（见改动清单"风险与取舍"） */
    if (len) tmos_memcpy(e->data, data, len);

    e->len        = (uint8_t)len;
    e->event_type = EVENT;
    e->rssi       = (int8_t)Rssi;

    g_adv_q_tail = next;                     /* 生产者只写 tail：最后一步"发布" */
    g_adv_seen   = 1;                        /* 喂给 1 秒事件：扫描看门狗"有广播包" */
}
