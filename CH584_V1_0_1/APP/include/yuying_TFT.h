#ifndef YUYING_TFT_H
#define YUYING_TFT_H


#include "u8g2.h"
#include "CONFIG.h"

#define DC_CMD          GPIOA_ResetBits(GPIO_Pin_2);
#define DC_DATA         GPIOA_SetBits(GPIO_Pin_2);
#define CS_LOW          GPIOA_ResetBits(GPIO_Pin_12);
#define CS_HIGH         GPIOA_SetBits(GPIO_Pin_12);
#define RST_LOW         GPIOA_ResetBits(GPIO_Pin_3);
#define RST_HIGH        GPIOA_SetBits(GPIO_Pin_3);

#define SCREEN_WIDTH    384  //定义 屏幕的宽度
#define SCREEN_HEIGHT   168  //定义 屏幕的高度
#define POS_X           20   //定义 菜单x坐标起始位置
#define POS_Y           24   //定义 菜单y坐标起始位置
#define MENU_WIDTH      64   //定义 每条菜单的宽度
#define MENU_HIGH       18   //定义 每条菜单的高度
#define SIDE_WIDTH      2    //定义 与四周边的距离
#define ROUND_RADIUS    6    //定义 圆角半径
#define TIMEX           10   //定义 中断时间

#define MENU_HEIGHT     32

/* Historical u8g2 font selector retained for source compatibility.
 * Current UI text is rendered with ui_menu_assets.h bitmap glyphs through
 * ui_text_draw/ui_text_width. Do not set this unused font in the page path:
 * doing so retains the 131544-byte legacy font in Flash via a relocation.
 * The original font data remains in APP/u8g2_font_cn.c for optional reuse.
 */
#define UI_FONT_CN      u8g2_font_wqy14_t_gb2312a

#define POS_X1          110  //定义菜单x坐标起始位置
#define POS_Y1          30   //定义菜单y坐标起始位置
/*------功能：定义菜单状态枚举类型数组------*/
enum MENU_STATE {
  M_Main,  //主菜单
  M_Menu,  //二级子菜单
  //---M_Main子菜单内容---//
  M_Menu_1,      //三级子菜单
  M_Menu_2,      //三子菜单
  M_Menu_3,      //三子菜单
  M_Menu_4,      //三子菜单
  M_Menu_5,      //三子菜单
  M_Menu_6,      //三子菜单
  M_Menu_7,      //三子菜单
  M_Menu_break,  //三子菜单
};
/*------功能：定义菜单结构体数据组------*/
typedef struct  {
  uint8_t index;  //索引序号
  uint8_t posx;   //x坐标
  uint8_t posy;   //y坐标
  char *name;     //菜单名称
    void (*function)();  //配套回调函数
}MENU_LIST;
typedef enum
{
    TPYE_NONE=0,
    TPYE_MG,  //锚杆
    TPYE_JG,  //激光
    TPYE_WY2, //位移
    TPYE_WY4,
    TPYE_WY6,
    TPYE_WY8,
    TPYE_LF,  //裂缝
    TPYE_QJ,  //倾角
    TPYE_YL,  //转孔应力
    TPYE_YW,  //液位
    TPYE_WZ,  //微震
    TPYE_DY,  //地音
    TPYE_KK,  //测试
    TPYE_END,

}Sensor_Tpye;


typedef struct
{
    Sensor_Tpye Type1         ;
    uint16_t    version       ; //版本号
    uint16_t    vbat          ; //电池电压
    uint16_t    host_num      ;
    uint16_t    send_host_num ;
    uint16_t    sub_num  ;
    uint8_t     state    ;
    uint8_t     Lora_rssi;

    uint16_t    chu_num1 ;
    uint16_t    chu_num2 ;

    uint8_t     re_flag  ;      /* 二级子页选择（契约 §2.3）：0=无 1=一键解绑 2=绑定设备
                                 * 3=设备信号 4=设备电压 5=已绑定设备名称 6=恢复出厂确认 */

    uint16_t    data[20] ;      //配套回调函数

}data_main;


typedef struct
{
    uint16_t set_host_num;
    uint16_t set_sub_num;

}data_addr1;

/* ★★★ V1_0_10：组网测试页一次显示**三个测试地址**的全套结果。
   ------------------------------------------------------------------
   原来的字段（xuhao_num/text_host_addr/text_cnt/bl_numl）只描述"当前正在测的
   那一个地址"，所以屏上只能看到一行、而且测完就被 STM32 复位，
   根本没法统计三个地址的结果。现在 STM32 一次把三组 (地址, 次数, 成功数)
   全发下来，屏端整页排版、一眼看全。**两侧必须同版本烧写。** */
typedef struct  {
    uint16_t now_host_addr;   /* 本机地址（台号） */
    uint16_t net_flag;        /* 组网状态，见 NET_FLAG_xxx */
    uint16_t cur_index;       /* 正在测试的地址序号 0~2（测完=3） */
    uint16_t addr[3];         /* 三个测试地址 */
    uint16_t cnt[3];          /* 各自的已测次数（0~10） */
    uint16_t succ[3];         /* 各自的成功次数（0~10） */
    //配套回调函数
}data_addr2;

/* net_flag 取值（与 STM32 LoraPro.h 的 CRfNetStatus 一一对应）：
     0 = 空闲：显示"开始组网 / 返回"选择页
     1~3 = 组网进行中：显示实时结果 + 底部"组网进行中"
     4 = 组网完成：显示最终结果 + 底部"重新组网 / 返回"（★ V1_0_10 新增，不再自动返回） */
#define NET_FLAG_IDLE    0u
#define NET_FLAG_DONE    4u

typedef struct  {
    uint8_t   pass_buf[4];  //索引序号
    uint16_t  pass_flag;
    uint16_t  biaoding_flag[3];  //存储标定成功标记位
    uint16_t  biaoding_ad[3][2]; //ad标定值

    //配套回调函数
}data_addr3;

typedef struct  {
    uint16_t shishi_buf[20];  //索引序号
    uint16_t  kaer_time;

    //配套回调函数
}data_addr4;

typedef struct  {

    uint16_t  old_send_addr[3];
    uint16_t  new_send_addr[3];
    //配套回调函数
}data_addr5;

typedef struct  {
    char  power;
    uint16_t time_light;
    uint8_t state ;

}data_addr6;
typedef struct  {
    uint16_t  len_value[3];
    uint16_t  ad_value[3];
    uint16_t  old_len ;
    uint16_t  new_len ;

    uint16_t  chu_num1 ;
    uint16_t  chu_num2 ;
}data_addr7;

typedef struct
{
    Sensor_Tpye Type;  //索引序号
    uint8_t  menu_rank;  //页面等级
    uint8_t  rank2_addr; //
    uint8_t  rank3_addr;

    data_main  UI_main;
    data_addr1 Menu_rank1;
    data_addr2 Menu_rank2;
    data_addr3 Menu_rank3;
    data_addr4 Menu_rank4;
    data_addr5 Menu_rank5;
    data_addr6 Menu_rank6;
    data_addr7 Menu_rank7;

}data_LIST;


void UI_Control(data_LIST *list);
/*------功能：预定义子菜单函数------*/
void Blink_Control(void);
void PWM_Control(void);
void Timer_Control(void);
void Timer_Zero(void);
void About(void);
void Timer_Start(void);

void UI_Main_Display(data_LIST *pData);
void UI_Menu_Display(void);

/* ======================================================================
 * ★★ 保存结果提示页（V1_0_2 第六轮新增）
 * ----------------------------------------------------------------------
 * 需求：保存动作必须有**明确的成功/失败页面提示**，分两类来源：
 *   1) 参数保存到 **STM32 自己的 Data-EEPROM** —— 结果由 STM32 知道，
 *      它用一帧 0x01（menu_rank = 6，参数 = 下面这些码）通知本机显示；
 *   2) 蓝牙绑定保存到 **CH584M 自己的 Data-Flash**（0x05 指令）——
 *      结果本机自己知道，直接本地显示。
 * 两类都走同一套提示页机制，所以两边只需要约定这几个码。
 *
 * 显示时长 UI_MSG_HOLD_SEC 秒（由 1 秒事件倒数），到点自动回到普通页面；
 * 显示期间**普通页面帧不会把提示盖掉**（UI_Control 里优先画提示页），
 * 但普通页面的字段照常更新，所以提示消失后画回的就是最新页面。
 * ====================================================================== */
#define UI_MSG_NONE             0u   /* 没有提示                              */
#define UI_MSG_PARAM_SAVE_OK    1u   /* STM32 参数保存成功                    */
#define UI_MSG_PARAM_SAVE_FAIL  2u   /* STM32 参数保存失败                    */
#define UI_MSG_BIND_SAVE_OK     3u   /* CH584M 蓝牙绑定保存成功（0x05）        */
#define UI_MSG_BIND_SAVE_FAIL   4u   /* CH584M 蓝牙绑定保存失败（0x05）        */
#define UI_MSG_BIND_SAVE_EMPTY  5u   /* CH584M 当前没有绑定，无内容可保存     */
#define UI_MSG_BIND_CLEAR_OK    6u   /* CH584M 绑定记录已清除（0x04 / 0x07）  */
#define UI_MSG_BIND_CLEAR_FAIL  7u   /* CH584M 绑定记录清除失败（0x04 / 0x07）*/

/* ★ 第 11 轮新增：STM32 软关机提示页（"长按K1+K3开机"）。
 *   与 1~7 号不同，本号**不自动消失** —— STM32 正式开机后会发
 *   msg=0（UI_MSG_NONE）显式清除它。见 yuying_TFT.c 的 ui_msg_tick_sec()。 */
#define UI_MSG_POWER_OFF        8u   /* 软关机：提示长按 K1+K3 开机        */
#define UI_MSG_SHUTTING_DOWN    9u   /* 正在关机中（5 秒后 STM32 断电）    */

#define UI_MSG_HOLD_SEC         2u   /* 提示页显示时长（秒），1 秒事件倒数     */

void ui_show_msg(uint8_t code);      /* 显示一条保存结果提示（code = UI_MSG_xxx）*/
uint8_t ui_msg_tick_sec(void);       /* 1 秒事件调用；返回 1 表示提示刚刚消失（调用方应请求重画）*/

void addr_Control();
void networking_Control();
void calibration_Control();
void install_Control();
void uploading_Control();
void Other_Settings_Control();
void zero_setting_Control();
void return_main_Control();
/* 功能清单新增页面（契约 §1.2 / §1.4）：菜单第 2 项「设备绑定」、第 6 项「信息汇总」 */
void binding_Control(void);
void summary_Control(void);
/* ★ V1_0_10：networking_Control2() 已删除（被整页结果表取代） */
void password_Control();
void calibration_pass_Control(void);
void new_return(void);
void addr_Control1(void);
void TFT_IO_init(void);
void u8g2Init1(u8g2_t *u8g2);
void u8g2Init(u8g2_t *u8g2);
void UI_TestArtwork(u8g2_t *u8g2);


void spi_write_test(uint16_t data);
void spi_write_test2(uint16_t data);
uint8_t u8x8_CH584M_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,U8X8_UNUSED void *arg_ptr);
uint8_t u8x8_byte_CH584M_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,void *arg_ptr);
void draw(u8g2_t *u8g2);


#endif /* OBSERVER_H */
