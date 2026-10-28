/********************************** (C) COPYRIGHT *******************************
 * File Name          : observer.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/11/12
 * Description        : �۲�Ӧ��������������ϵͳ��ʼ��
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef OBSERVER_H
#define OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif
#include "yuying_TFT.h"

// Simple BLE Observer Task Events
#define START_DEVICE_EVT       0x0001
#define START_DISCOVERY_EVT    0x0002
#define START_SCAN_EVT         0x0004

//
#define MAX_DEVICES 2
#define MAX_NAME_LEN 30
#define MAX_MFG_DATA_LEN 50

#define SW_NAME_PREFIX      "SW_"
#define SW_NAME_PREFIX_LEN  3

//#define MAX_NAME_LEN        24      /* 名字缓冲区最大长度（含 '\0'） */
#define MAX_PAYLOAD_LEN     64
/* ★ 容量对齐（2026-09-21 契约 §2.2）：协议只承载 20 通道（0x03/0x06 应答固定
 *   20 × 4 = 80 字节，Usart3_task.c 的 SENSOR_RSP_CH_NUM == 20），设备名里的
 *   通道号也是 1..20（STM32 侧 ble_data.c：devices[X2 - 1]）。
 *   改前是 30 ⇒ channels_available()/count_used_channels() 会分配并统计下标 20..29，
 *   但这些通道**永远不会出现在 0x03/0x06 应答里**（只上报 0..19），
 *   表现为"绑定成功却读不到数据"（契约 §1.2）。统一为 20，顺带省 RAM。 */
#define MAX_CH_NUM          20      /* 总通道数，对应 CH_com_buf[20]（下标 0..19 = 通道 1..20） */
#define MAX_BINDING_NUM     32      /* 最多绑定的设备数（MAC 条数） */

/* ---------------- 绑定名变更监测（10 分钟巡检） ---------------- */
#define NAME_CHK_PERIOD_SEC 600     /* 巡检周期：600 秒 = 10 分钟（由 1s 事件累计触发） */
#define NAME_CHK_ERR_NUM    16      /* 名字变更错误记录表容量（RAM，满则环形覆盖最旧） */

/* ---------------- 扫描模式（P1/P2 修复 + 功能清单页面驱动） ----------------
 * 广播扫描/解析模式与"当前显示哪个页面"的关系（契约 §4，2026-09-21 冻结）：
 *   - 由 Usart3_task.c 的 1 秒事件按**当前页面**切换：
 *       停留在「设备绑定 → 绑定设备」二级子页 ⇒ SCAN_MODE_BIND
 *       （menu_rank == 3 && rank2_addr == 2 && UI_main.re_flag == 2）
 *       其余任何页面（含主页、菜单列表、其它子页） ⇒ SCAN_MODE_DATA
 *   - **旧版"开机 30 秒窗口"已删除**（原来开机 30 秒内绑定、之后只更新）；
 *   - observer.c 的 SACN_DATA() 仍只依据 g_scan_mode 选择分支，不看 menu_rank。
 * Data_list1.menu_rank / rank2_addr / rank3_addr 仍为"只能由主机帧设置 + UI 读取"，
 * 1 秒事件只读不回写。 */
#define SCAN_MODE_DATA      0       /* 只更新：按绑定表填数据 */
#define SCAN_MODE_BIND      1       /* 绑定：解析名字 + add_binding */
extern volatile uint8_t g_scan_mode;    /* 由 1 秒事件按当前页面切换（见上） */
extern volatile uint8_t g_scan_enable;  /* Disabled after reset until explicit committed ON. */
/* Main-loop/TMOS only: state 1 resumes collection, other values stop it. */
void observer_set_collection_enabled(uint8_t state);

/* ---------------- 广播回调异步化（契约 §2.2，2026-09-22 冻结） ----------------
 * SACN_DATA()（BLE 广播回调）**只入队**；解析名字/payload、绑定、填通道、写名字快照
 * 全部搬到主循环里由 observer_adv_drain() 排空时执行（消费者）。
 *   - 队列：ADV_Q_DEPTH(64) 条 × 73B = 4672B ≈ 4.56KB bss，条目类型 adv_q_entry_t
 *     定义在 observer.c（内部类型，不对外暴露）；
 *   - 排空节拍：Usart3_task.c 的 START_IO_EVT（10ms）里 observer_adv_drain(16);
 *     ⇒ 1600 包/秒容量，落地延迟 <= 10ms；
 *   - 头/尾索引用 volatile uint8_t + 留一个空槽判满（单生产者只写 tail、
 *     单消费者只写 head），不关中断也能自洽。 */
void observer_adv_drain(uint16_t max_items);

/* 1 秒事件（主循环上下文）调用：计数摘要打印 + 扫描看门狗。
 *   - 摘要行：[BLE] adv: q_max=%u drop=%u scan rst=%u busy=%u err=%u | bind ok=%u reject=%u
 *     只在"任一计数变化"时打印一次；
 *   - 看门狗：连续 ADV_QUIET_RESTART_SEC(10) 秒没有任何广播包且 g_scan_enable=1 时
 *     重新 GAPRole_ObserverStartDiscovery()（兜底 TGAP_DISC_SCAN=0 的语义风险）。 */
void observer_ble_stat_tick(void);
void observer_power_diag_tick(void); /* 10-second receive/gate snapshot, main loop only. */
void observer_ble_diag_tick(void);   /* 第 9 轮：蓝牙扫描诊断打印 */

/* 回调路径日志总闸（契约 §2.3）：默认 0 = 静音。
 * 回调路径上只允许 BLE_LOG(...)；非回调路径（[LOAD]/0x03/0x04/0x05/Flash）保持 PRINT。 */
extern volatile uint8_t g_ble_log_enable;

/* 可见性计数：只在 1 秒事件（主循环上下文）打印，绝不在 BLE 回调里打印 */
extern uint16_t g_adv_q_drop;        /* 队列满丢弃次数（丢最新） */
extern uint8_t  g_adv_q_max;         /* 历史最大积压（条） */
extern uint16_t g_bind_ok_cnt;       /* 成功新增绑定次数 */
extern uint16_t g_bind_reject_cnt;   /* 拒绑次数（名字被占/通道不足/区间冲突/表满） */
extern uint16_t g_scan_restart_cnt;  /* 看门狗返回 SUCCESS：之前确实是停的，被重启 */
extern uint16_t g_scan_busy_cnt;     /* 看门狗返回 0x11：扫描本就在跑（正常，随静默 +1） */
extern uint16_t g_scan_err_cnt;      /* 看门狗返回其它非 SUCCESS 码（异常） */

//实时扫描的缓存
typedef struct {
    uint8_t mac[6];                     //ַ
    uint8_t name[MAX_NAME_LEN];         //
    uint8_t name_len;                   //
    uint8_t mfg_data[MAX_MFG_DATA_LEN]; //
    uint8_t mfg_len;                    //
    int rssi;
} device_info_t;


//已经绑定的每一个通道的数据记录
/* 每通道数据记录 */
typedef struct
{
    uint8_t      mac[6];             /* MAC */
    uint8_t      name[MAX_NAME_LEN]; /* 名字（'\0' 结尾） */
    Sensor_Tpye  Type;               /* 传感器类型 */
    uint16_t     CH_data;            /* 通道数据 */
    uint8_t      voltage;            /* 电压（同设备各通道共用） */
    uint8_t      rssi;               /* 信号强度 */

    uint8_t      valid;              /* 该通道是否已被分配 */
    uint8_t      data_re_flag;       /* 收到新数据标志 */
    uint8_t      reserved;
} device_t;

/* 绑定记录（每个 MAC 一条） */
typedef struct
{
    uint8_t      name[MAX_NAME_LEN]; /* 名字 */
    uint8_t      mac[6];             /* MAC */
    Sensor_Tpye  Type;               /* 传感器类型 */
    uint8_t      host_num;           /* 对应主机号 */
    uint8_t      frist_ch_num;       /* 起始通道 */
    uint8_t      ch_num;             /* 通道个数 */
} scan_binding;

/*
 * 名字快照：10 分钟窗口内，某个"已绑定 MAC"最近一次广播里捕获到的名字。
 * 下标与 g_binding_list[] 一一对应（绑定表只追加、不删除，索引稳定）。
 * 仅用于比对，不参与任何数据填充。
 */
typedef struct
{
    uint8_t      mac[6];             /* 对应 MAC（比对前做一次校验，防槽位错位） */
    uint8_t      name[MAX_NAME_LEN]; /* 最近一次捕获到的名字，'\0' 结尾 */
    uint8_t      valid;              /* 本窗口内是否捕获到该 MAC 的名字 */
} name_snapshot_t;

/*
 * 绑定名变更错误记录（RAM 定长表，不写 EEPROM/Flash）。
 * 记录：MAC、绑定时记录的名字（旧名）、窗口内捕获到的名字（新名）、
 *       检测次数、首次/最近一次检测时间（秒计数）。
 */
typedef struct
{
    uint8_t      mac[6];                 /* 发生名字变更的 MAC */
    uint8_t      old_name[MAX_NAME_LEN]; /* 绑定时记录的名字，'\0' 结尾 */
    uint8_t      new_name[MAX_NAME_LEN]; /* 窗口内捕获到的新名字，'\0' 结尾 */
    uint32_t     hit_cnt;                /* 被重复检测到的次数 */
    uint32_t     first_sec;              /* 首次检测时间（秒计数） */
    uint32_t     last_sec;               /* 最近一次检测时间（秒计数） */
} name_change_err_t;



typedef struct
{
    uint8_t      mac[6];             // MAC
    uint8_t      name[MAX_NAME_LEN]; // 名字
    Sensor_Tpye  Type;               // 传感器类型ַ
    uint16_t     CH_data ;           // 通道数据
    uint8_t      rssi;               // 信号强度
    uint8_t      valid ;             // 是否有效
//    uint8_t      name_received ;     //
//    uint8_t      mfg_received  ;      //
    uint8_t      data_re_flag  ;     //
//    uint32_t     last_seen;          //
    uint8_t      reserved ;
}JG_device_t;

extern void Observer_Init(void);
extern uint16_t Observer_ProcessEvent(uint8_t task_id, uint16_t events);

__HIGH_CODE
void SACN_DATA(uint8_t *addr, uint8_t EVENT, uint8_t *data,
               uint16_t data_len, int Rssi);

int find_binding_by_mac(const uint8_t *mac);
int find_binding_by_name(const char *name, uint8_t name_len);
/* 已占用通道数（CH_com_buf[].valid 计数）；Flash.c 开机恢复后打印用 */
uint8_t count_used_channels(void);

/* ---- 绑定表 / 通道表对外可见（解绑、落盘、开机恢复都要用） ---- */
/* 定义在 observer.c；Usart3_task.c（0x03 应答）与 Flash.c（0x04/0x05、开机加载）都会引用 */
extern scan_binding g_binding_list[MAX_BINDING_NUM];   /* RAM 绑定表 */
extern uint8_t      g_binding_count;                   /* 已绑定条数（0..MAX_BINDING_NUM） */
extern device_t     CH_com_buf[MAX_CH_NUM];            /* 每通道数据（下标即通道号） */

/* ==================================================================
 * 绑定落盘标志 + 0x03「收齐本轮再应答」状态（2026-09-21 契约 §2.3 / §2.6）
 *
 * 【§2.3 落盘】add_binding() 新增一条绑定成功后**只置** g_store_dirty=1；
 *   真正的擦写 Flash 由 Usart3_task.c 的 1 秒事件（START_TIMER_EVT，主循环/TMOS
 *   上下文）执行 binding_store_save() 完成。**严禁在 BLE 广播回调 SACN_DATA()
 *   里直接擦写 Flash**（擦除几十毫秒会阻塞协议栈、丢广播）。
 *   0x05 指令保留，语义不变（手动保存仍然可用）。
 *
 * 【§2.6 0x03 延迟应答】收到 0x03 请求后不立即回包，等"本轮所有在用设备都
 *   收到过一次新数据"再回；30 秒仍收不齐则用旧数据回。
 *   下标与 g_binding_list[] **一一对应**（绑定表只追加、不删中间项，索引稳定）。
 *   计数器只在 1 秒事件里推进/发送（见 Usart3_task.c），
 *   BLE 回调只做 g_dev_fresh[]/g_dev_miss[] 的 0→1 置位，不发送任何帧。
 * ================================================================== */
#define RESP_TIMEOUT_SEC        30    /* 0x03 应答超时（秒）：到点用旧数据应答 */
/* 早应答窗口(秒): 收到请求后最多等这么久就用已有数据应答。
 * ★★ 默认 = RESP_TIMEOUT_SEC(30) = **关闭**，保持"等全部设备到齐、否则 30s 兜底"的
 *    正确语义，**不会丢本轮新数据**（这是推荐值）。
 * ★★ 若现场确认"最慢设备也在 N 秒内必然播到"，可把 N 调小（如 5~8），
 *    收益是缩短息屏获取时间、降功耗；代价是：窗口内仍未播到的设备，本轮用它的
 *    上一轮旧值应答（STM32 收到应答即给 CH584 断电，迟到数据补不回来）。
 *    调小前请先看 `read-window:` / `[LOAD]` 日志确认最慢设备的实际到齐时间。 */
#define RESP_EARLY_SEC          RESP_TIMEOUT_SEC
#define DEV_MISS_ROUNDS_MAX      3    /* 连续 3 轮收不到新数据 → 判该设备损坏，不再等它 */

extern volatile uint8_t g_store_dirty;            /* 1 = 有新增绑定待落盘（1 秒事件消费） */
extern uint8_t  g_rsp_pending;                    /* 1 = 有 0x03 请求正在等数据 */
extern uint8_t  g_dev_fresh[MAX_BINDING_NUM];     /* 本轮该设备是否已收到新数据 */
extern uint8_t  g_dev_miss[MAX_BINDING_NUM];      /* 连续未收到新数据的轮数（0..DEV_MISS_ROUNDS_MAX） */

/* 收到合法 0x03 请求时调用：1 = 已受理（进入等待）；0 = 已有请求在途，本次忽略 */
uint8_t observer_cmd03_request(void);
/* 1 秒事件调用：秒计数 +1（只在 1 秒事件里推进，时基与应答判定同源） */
void    observer_cmd03_sec_tick(void);
/* 1 秒事件调用：1 = 现在应当应答（收齐 或 超过 RESP_TIMEOUT_SEC）；0 = 继续等 */
uint8_t observer_cmd03_ready(void);
/* 应答发送完成后调用：用 fresh 更新 miss（fresh→0，否则 +1 且封顶 3），清 pending */
void    observer_cmd03_answered(void);

/* ---- 解绑全部 / 开机恢复（任务 B、D 的 observer 侧 RAM 操作） ---- */
/* 解绑全部：清绑定表 + 通道表 + 名字快照 + 错误记录；返回 1=清空前有绑定，0=本来就没有 */
uint8_t observer_clear_all_bindings(void);
/* 开机恢复单条记录（Flash 侧已校验通过）：追加绑定 + 占用通道；返回 0 成功 / -1 拒绝 */
int observer_restore_binding(const uint8_t *mac, const char *name, uint8_t name_len,
                             Sensor_Tpye type, uint8_t host_num,
                             uint8_t start_ch, uint8_t ch_num);

/* ---- 采集期逐通道/逐包打印的运行期开关（任务 A-3） ----
 * 编译期开关 SCAN_DBG_CH_LOG 在 observer.c 里定义（默认 0 = 完全编译掉）。
 * 该变量在编译期关闭时恒为 0，仅供调试或上层查询使用。 */
extern volatile uint8_t g_ch_log_enable;

/* ---- 绑定名变更监测（10 分钟巡检）对外接口 ---- */
/* 由 Usart3_task.c 的 1s 事件每秒钟调用一次；累计满 600 次（10 分钟）执行一轮检查 */
void name_change_monitor_tick(void);
/* 打印 RAM 错误记录表全部内容（有变更时由巡检自动调用，也可手工调用） */
void name_change_dump(void);

/* ==================================================================
 * 功能清单新增（2026-09-21）：电压异常计数 / 最近扫描名称缓存 / 一轮采集完成
 * ================================================================== */

/* ---- 电压异常计数（「信息汇总 → 蓝牙电压异常警报:个数」，契约 §1.4/§3） ----
 * 判定点与判定依据见 observer.c 的 volt_err_check()：
 *   广播 payload 里的 1 字节电压 == 0 即计一次（详见函数注释里的理由）。
 * 去重：同一个广播包（同一次 fill_channel_data()）只计一次。
 * 计数在 observer_clear_all_bindings()（0x04 解绑 / 0x07 恢复出厂）里清零。 */
extern uint8_t g_volt_err_count;
uint8_t observer_volt_err_count(void);

/* ---- 「最近扫描到的蓝牙名称」缓存（「设备绑定 → 绑定设备」子页显示用） ----
 * 只在绑定模式（SCAN_MODE_BIND，即停留在"绑定设备"二级子页）里写入，
 * 新名字置顶（下标 0 最新），同 MAC 去重，容量满则挤出最旧的一条。
 * 缓存对象是"扫描到的设备名"，与绑定表无关（**不**复用 device_list[]，理由见 .c）。 */
#define SCAN_NAME_CACHE_NUM  6
typedef struct
{
    uint8_t mac[6];              /* 用于去重 */
    uint8_t name[MAX_NAME_LEN];  /* 名字（'\0' 结尾） */
} scan_name_entry_t;

extern scan_name_entry_t g_scan_name_cache[SCAN_NAME_CACHE_NUM];
uint8_t        scan_name_cache_count(void);          /* 已缓存条数（0..SCAN_NAME_CACHE_NUM） */
const char    *scan_name_cache_name(uint8_t idx);    /* 第 idx 条名字（0 最新），越界返回 NULL */

/* ---- 「一轮采集完成」判定（0x06 主动上报，契约 §5） ----
 * g_round_mask：本轮"已更新通道"位图（bit i = 通道 i，i < MAX_CH_NUM <= 32）。
 * 每个通道数据填入点（fill_channel_data）置位；覆盖"全部已占用通道"即视为
 * 一轮采集完成；上报后由 Usart3_task.c 调 observer_round_clear() 清零重新开始。
 * 时基由 1 秒事件（observer_round_tick）累加，不新开定时器。 */
#define ROUND_REPORT_FALLBACK_SEC  10   /* 兜底：距上次上报超过 10s 且期间有新数据则强制上报 */

extern uint32_t g_round_mask;

void     observer_round_tick(void);                 /* 1 秒事件调用：时间基准 +1s */
uint8_t  observer_round_has_new_data(void);         /* 本轮是否收到过任何新数据 */
uint8_t  observer_round_is_complete(void);          /* 位图是否已覆盖全部已占用通道 */
uint32_t observer_round_seconds_since_report(void); /* 距上次上报（或清零）的秒数 */
void     observer_round_clear(void);                /* 清零位图并记录"现在上报过了" */

extern name_snapshot_t   g_name_snap[MAX_BINDING_NUM];   /* 名字快照表（下标同绑定表） */
extern name_change_err_t g_name_err_list[NAME_CHK_ERR_NUM]; /* 变更错误记录表（RAM） */
extern uint8_t           g_name_err_count;               /* 已写入的错误记录条数 */
extern uint32_t          g_name_chk_sec;                 /* 秒计数（1s 事件驱动） */
extern uint32_t          g_name_chk_round;               /* 已触发的巡检轮次 */
/*
 * Task Initialization for the BLE Application
 */
//int update_device_mfg_data(uint8_t index,uint8_t *mac, uint8_t *mfg_data, uint8_t mfg_len,int rssi);
//
//void pack_name_data(uint8_t *MAC_addr,uint8_t *data, uint16_t data_len, int Rssi);
//void init_device_list(void);
//void hash_init(void);
///*
// * Task Event Processor for the BLE Application
// */
//
//void process_advertising_data(uint8_t *data, uint8_t data_len, int8_t rssi, uint8_t *addr);
// void hash_insert(int idx) ;
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_H */
