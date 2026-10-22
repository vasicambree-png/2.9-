#ifndef	__UI_H__
#define __UI_H__

#include "main.h"

/* ---------------- 页面 / 菜单状态（沿用原有按键逻辑） ---------------- */
typedef enum{
	Page_NONE=0,
	Page_WY,
	Page_MG,
	Page_JG,
	Page_YL,
	Page_END,
}Page;

/* 菜单项索引，与 CH584M 的 rank2_addr 0~7 一一对应：
   0 地址分区 / 1 组网测试 / 2 标定 / 3 安装调试 /
   4 上传设置 / 5 其他设置 / 6 数据置零 / 7 返回主页 */
typedef enum
{
  Opt0 = 0,
  Opt1,
  Opt2,
  Opt3,
  Opt4,
  Opt5,
  Opt6,
  Opt7,
}CDisOpt;

typedef enum{
	Set_1=0,
	Set_2,
	Set_3,
	Set_4,
	Set_5,
	Set_6,
	Set_7,
	Set_8,
	Set_9,
	Set_10,
}Set;

typedef enum{
	Set_Addr1=0,
	Set_Addr2,
	Set_Addr3,
	Set_Addr4,
	Set_Addr5,
	Set_Addr6,
	Set_Addr7,
	Set_Addr8,
	Set_Addr9,
	Set_Addr10,
}Set_addr;

/* --------------------------------------------------------------------------
 * 第 4 层 UI，由冻结契约新启用（见
 * 协议_功能清单_页面码与协议冻结.md 第 0 / 1 / 2.3 节）：
 *   UI.Sub2 -> 0x01 帧的 re_flag   （二级子页选择）
 * 取值已冻结，原样作为 re_flag 发给 CH584M。
 * ------------------------------------------------------------------------ */
typedef enum
{
  Sub2_None    = 0,   /* 没有二级子页                                */
  Sub2_Unbind  = 1,   /* 设备绑定 / 一键解绑   -> 解绑 / 返回         */
  Sub2_BindDev = 2,   /* 设备绑定 / 绑定设备   -> 保存设备 / 返回     */
  Sub2_Signal  = 3,   /* 安装调试 / 信号  -> 20 通道 RSSI，单页全屏 + 返回 */
  Sub2_Volt    = 4,   /* 安装调试 / 电压  -> 20 通道电压，单页全屏 + 返回 */
  Sub2_BindNam = 5,   /* 信息汇总 / 名称  -> 20 通道名称，单页全屏 + 返回 */
  Sub2_Factory = 6,   /* 信息汇总 / 恢复出厂   -> 确认 / 返回         */
} CDisSub2;

typedef struct{
	uint8_t lcd_open;
	uint8_t Power_Lcd_flag;
	uint8_t dis_data_flag;
	uint8_t old_dis_data_flag;
	uint8_t Setting_flag;      /* 0 主页 / 1 菜单 / 2 子页 */
	uint8_t Old_Setting_flag;

	Set Set_page;
	Set Old_Set_page;
	Set_addr Addr;             /* 子选项索引（rank3_addr） */
	Set_addr Old_Addr;

	/* ---- 第 4 层 + 页码（复用 0x01 帧，字段顺序未变） */
	CDisSub2 Sub2;             /* -> re_flag（Sub2_None = 没有子页）       */
	uint8_t MainPage;          /* 主页 1..2 -> chu_num1                    */
	uint8_t SubPage;           /* 已作废：恒 0（三页改单页全屏后 chu_num2 不再当页码）                */
}ALL_UI;

/* 唯一实例定义在 ui.c；在这里声明，让每个包含 ui.h 的地方都能看到它 */
extern ALL_UI UI;


/* ---------------- 协议数据模型（与 CH584M 一致） ---------------- */
typedef struct
{
	uint16_t    version       ; /* 版本号 */
	uint16_t    vbat          ; /* 电池电压（单位 100 mV） */
	uint16_t    host_num      ; /* 主机号 */
	uint16_t    send_host_num ; /* 发送目标号 */
	uint16_t    sub_num       ; /* 分站号 */
	uint8_t     state         ; /* 开关状态 */
	uint8_t     Lora_rssi     ; /* 信号档位 8~11 */
	uint16_t    chu_num1      ; /* 主页面页码（1 = 通道 1~10，2 = 11~20） */
	uint16_t    chu_num2      ; /* 二级子页页码：§4.2 之后恒 0（三页已改单页全屏） */
	uint8_t     re_flag       ; /* 二级子页选择（0~6，见上方第 4 层说明） */
	uint16_t    data[20]      ; /* 20 个通道 */
}data_main;

typedef struct 
{ 
	uint16_t set_host_num; 
	uint16_t set_sub_num; 
} data_addr1;
typedef struct {
	uint16_t now_host_addr;   /* 本机地址（台号） */
	uint16_t net_flag;        /* 组网状态，见 LoraPro.h 的 CRfNetStatus */
	uint16_t cur_index;       /* 正在测试的地址序号 0~2（测完 = 3） */
	uint16_t addr[3];         /* 三个测试地址 */
	uint16_t cnt[3];          /* 各自已测次数（0~10） */
	uint16_t succ[3];         /* 各自成功次数（0~10） */
} data_addr2;
typedef struct {
	uint8_t  pass_buf[4];
	uint16_t pass_flag;
	uint16_t biaoding_flag[3];
	uint16_t biaoding_ad[3][2];
} data_addr3;
typedef struct 
{ 
	uint16_t shishi_buf[20]; 
	uint16_t kaer_time; 
} data_addr4;
typedef struct 
{ 
	uint16_t old_send_addr[3]; 
	uint16_t new_send_addr[3]; 
} data_addr5;
typedef struct { int8_t power; uint16_t time_light; uint8_t state; } data_addr6;
typedef struct {
	uint16_t len_value[3];
	uint16_t ad_value[3];
	uint16_t old_len;
	uint16_t new_len;
	uint16_t chu_num1;
	uint16_t chu_num2;
} data_addr7;

typedef struct
{
	uint8_t  Type;       /* 传感器类型（Data_tpye 取值） */
	uint8_t  menu_rank;  /* 1 主页 / 2 菜单 / 3 子页 */
	uint8_t  rank2_addr; /* 菜单项索引 0~7 */
	uint8_t  rank3_addr; /* 子选项索引 */

	data_main  UI_main;
	data_addr1 Menu_rank1;
	data_addr2 Menu_rank2;
	data_addr3 Menu_rank3;
	data_addr4 Menu_rank4;
	data_addr5 Menu_rank5;
	data_addr6 Menu_rank6;
	data_addr7 Menu_rank7;
}data_LIST;


/* ---------------- UI 对外接口 ---------------- */
uint16_t build_frame_ui(uint8_t *tx_buf, data_LIST *pData, uint8_t ui_data_count);
void UI_send_frame(void);
void UI_display(void);
void Start_UI_display(void);
void UI_force_redraw(void);
/* 提示页消息码（与 CH584M 侧 APP\yuying_TFT.h 的 UI_MSG_xxx 编号必须一致）：
     1 参数保存成功   2 参数保存失败   3 绑定保存成功   4 绑定保存失败
     5 无绑定可保存   6 绑定已清除     7 绑定清除失败
     8 关机提示页（"长按K1+K3开机"）—— CH584M 侧对 8 号**不自动消失**，
       由 STM32 在开机后发 0 号（UI_MSG_NONE）显式清除。 */
/* ★ V1_0_49：0 号 = "显式清除提示页"。CH584M 侧对 8/9 号常驻提示不自动消失，
   开机后必须由 STM32 发这一帧把它清掉（否则屏幕会一直停在"长按K1+K3开机"）。 */
#define UI_MSG_NONE            0u
#define UI_MSG_POWER_OFF   8u
#define UI_MSG_SHUTTING_DOWN  9u   /* "正在关机中"：5 秒后真断电，常驻不自动消失 */

void UI_show_save_result(uint8_t msg_code);
void dis_data_task(void);

/* ==========================================================================
 * ★★ 保存结果提示页（V1_0_2 第六轮新增）—— 与 CH584M 侧约定的消息码
 * --------------------------------------------------------------------------
 * 需求：保存动作要有明确的成功/失败页面提示。分两类来源：
 *   1) **参数保存在 STM32 自己的 Data-EEPROM**（菜单里的"保存"）：
 *      结果只有 STM32 知道，所以由 STM32 发一帧提示帧
 *      （0x01 menu_rank = 6，1 个参数 = 下面的码），CH584M 只负责画；
 *   2) **蓝牙绑定保存在 CH584M 自己的 Data-Flash**（0x05 指令）：
 *      结果只有 CH584M 知道，所以 CH584M 本地直接画提示，不需要 STM32 参与。
 * 这里只需定义第 1 类用到的两个码。3~7 是 CH584M 本地用的
 * （见 CH584 工程 APP\include\yuying_TFT.h 的 UI_MSG_xxx），
 * **两边必须保持一致，改动时请同步。**
 * ========================================================================== */
#define UI_MSG_PARAM_SAVE_OK    1u   /* 参数保存成功 */
#define UI_MSG_PARAM_SAVE_FAIL  2u   /* 参数保存失败 */

#endif
