#include "CONFIG.h"
#include "yuying_TFT.h"
#include "ui_latency_diag.h"
#include "ui_fast_draw.h"
//#include "u8g2_fonts.c"
#include "u8g2.h"
#include "stdlib.h"
/* ★ 功能清单：主页面/子页要显示 CH584M 本地数据（CH_com_buf[]、绑定表、
 *   异常计数、扫描名称缓存），这些类型与 extern 声明都在 observer.h 里。
 *   observer.h 自身 include yuying_TFT.h（有 include guard），无循环问题。 */
#include "observer.h"
#include "ui_home_assets.h"
#include "ui_menu_assets.h"
#include "ui_wireless_assets.h"
extern u8g2_t u8g2;
data_LIST Data_list1;

const uint8_t bmp[]={
};

uint8_t UI_Select = 0;
uint8_t UI_State = 0;
uint8_t UI_LevelMark = 0;

//高
const uint8_t bitmap1[]={
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x7F,0x00,0x80,0xFF,0xFF,0x01,0xE0,0xFF,0xFF,0x07,0xF8,0x07,0xE0,0x1F,
        0xFC,0x00,0x00,0x3F,0x3E,0x00,0x00,0x7C,0x1F,0xF0,0x0F,0xF8,0x0F,0xFE,0x7F,0xF0,0x80,0xFF,0xFF,0x01,0xC0,0x1F,0xF8,0x03,0xE0,0x07,0xE0,0x07,0xE0,0x01,0x80,0x07,
        0xE0,0x00,0x00,0x07,0x00,0xF0,0x0F,0x00,0x00,0xF8,0x1F,0x00,0x00,0xFC,0x3F,0x00,0x00,0x1C,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x03,0x00,
        0x00,0xC0,0x03,0x00,0x00,0xC0,0x03,0x00,0x00,0xC0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*E:\桌面\信号高.bmp*/0
};
//中
const uint8_t bitmap2[]={
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x0F,0x00,0x00,0xFE,0x7F,0x00,0x80,0xFF,0xFF,0x01,0xC0,0x1F,0xF8,0x03,0xE0,0x07,0xE0,0x07,0xE0,0x01,0x80,0x07,
        0xE0,0x00,0x00,0x07,0x00,0xF0,0x0F,0x00,0x00,0xF8,0x1F,0x00,0x00,0xFC,0x3F,0x00,0x00,0x1C,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x03,0x00,
        0x00,0xC0,0x03,0x00,0x00,0xC0,0x03,0x00,0x00,0xC0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*E:\桌面\信号 中.bmp*/0
};
//低
const uint8_t bitmap3[]={

        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0xF0,0x0F,0x00,0x00,0xF8,0x1F,0x00,0x00,0xFC,0x3F,0x00,0x00,0x1C,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x03,0x00,
        0x00,0xC0,0x03,0x00,0x00,0xC0,0x03,0x00,0x00,0xC0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*E:\桌面\信号低.bmp*/0

};
//无
const uint8_t bitmap4[]={

        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x80,0x03,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0xF7,0x3F,0x00,0x80,0xFE,0xFF,0x01,0xE0,0x1F,0xF0,0x07,
        0xF0,0x1D,0x80,0x0F,0x7C,0x3C,0x00,0x3E,0x1E,0x38,0x00,0x78,0x0C,0xF0,0x07,0x30,0x00,0x7C,0x3F,0x00,0x00,0xEF,0xFE,0x00,0x80,0xEF,0xF1,0x01,0xC0,0xC3,0xC1,0x03,
        0x80,0xC1,0x83,0x01,0x00,0x80,0x03,0x00,0x00,0x60,0x07,0x00,0x00,0xF8,0x07,0x00,0x00,0x7C,0x0E,0x00,0x00,0x18,0x1E,0x00,0x00,0x00,0x1C,0x00,0x00,0x00,0x3C,0x00,
        0x00,0xC0,0x3B,0x00,0x00,0xC0,0x73,0x00,0x00,0xC0,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*E:\桌面\无信号.bmp*/0

};
MENU_LIST Main_List[] = {

  { 0, POS_X, POS_Y+ MENU_HIGH * 0, "地址分区", addr_Control },
  { 1, POS_X, POS_Y+ MENU_HIGH * 1, "组网测试", networking_Control },

  /* ★ 契约 §1：第 2 项由「探头标定」改为「设备绑定」（原标定页保留代码但不再挂载） */
  { 2, POS_X, POS_Y + MENU_HIGH * 2, "设备绑定", binding_Control },
  { 3, POS_X, POS_Y + MENU_HIGH * 3, "安装调试", install_Control },

  { 4, POS_X, POS_Y + MENU_HIGH * 4, "上传设置", uploading_Control },
  { 5, POS_X, POS_Y + MENU_HIGH * 5, "其他设置", Other_Settings_Control },

  /* ★ 契约 §1：第 6 项由「数据置零」改为「信息汇总」（原置零页保留代码但不再挂载） */
  { 6, POS_X, POS_Y + MENU_HIGH * 6, "信息汇总", summary_Control },
  { 7, POS_X, POS_Y + MENU_HIGH * 7, "返回主页", return_main_Control },
};
MENU_LIST Menu_List[] = {

  { 0, POS_X1, POS_Y1, "地址分区", addr_Control },
  { 1, POS_X1, POS_Y1, "组网测试", networking_Control},

  { 2, POS_X1, POS_Y1 , "设备绑定", binding_Control },
  { 3, POS_X1, POS_Y1, "安装调试", install_Control },

  { 4, POS_X1, POS_Y1 , "上传设置", uploading_Control },
  { 5, POS_X1, POS_Y1 , "其他设置", Other_Settings_Control },

  { 6, POS_X1, POS_Y1 , "信息汇总", summary_Control },
  { 7, POS_X1, POS_Y1 , "返回主页", return_main_Control },
};







const uint8_t MAIN_LEN = sizeof(Main_List) / sizeof(MENU_LIST);  //����˵���Ŀ����

/* ==================================================================
 * 功能清单（2026-09-21）新增：CH584M 本地数据的显示支持
 *
 * 普通文字统一为微软雅黑粗体，使用 ui_menu_assets.h 的 11/14/16/18 字模；
 * 首页静态标题同为微软雅黑粗体 20px。Logo 保留原图字形。
 * 实际页面文本由 ui_text_draw 绘制，不设置 u8g2 字库，避免链接未使用的全中文字库。
 * 20通道三级页同屏两列各10条、11px正文和14px行距；轮显列表用11px和13px行距。
 * ================================================================== */

/* 通道类型显示表（下标 = Sensor_Tpye 数值，与 STM32 的 Data_tpye 逐项一致）
 *   名称：sensor 类型中文名（取自枚举注释）；
 *   单位：★ 2026-09-22 按厂商广播格式规格（契约_传感器数据解析与显示标度 §4.2）确定：
 *         锚杆(MG)=kN、激光(JG)=mm、位移(WY2/4/6/8)=mm、裂缝(LF)=mm、
 *         倾角(QJ)=°、应力(YL)=MPa；液位/微震/地音/测试 仍未定义量纲，留空不猜。
 *   倾角的 ° 与中文名称均使用 ui_menu_assets.h 中的现有位图字模。
 */
static const char *const CH_TYPE_NAME[TPYE_END] = {
    "--", "锚杆", "激光", "位移", "位移", "位移", "位移",
    "裂缝", "倾角", "应力", "液位", "微震", "地音", "测试"
};
static const char *const CH_TYPE_UNIT[TPYE_END] = {
    /* 0 NONE  1 MG(kN)  2 JG(mm)  3 WY2  4 WY4  5 WY6  6 WY8  7 LF  8 QJ(°)  9 YL(MPa) 10..13 未定义 */
    "", "kN", "mm", "mm", "mm", "mm", "mm", "mm", "°", "MPa", "", "", "", ""
};

/* ★ 契约 §4.1：显示标度 —— MG / 位移(WY) / 应力(YL) / 倾角(QJ) 上传值带一位小数（显示 = 值/10）；
 *   激光 JG 不带（原样显示）。换算只发生在**显示层**，
 *   CH_com_buf[].CH_data 与 0x03/0x06 协议帧仍为**原始值**（上游口径不变）。 */
static uint8_t ch_disp_scale10(Sensor_Tpye t)
{
    return (t == TPYE_MG || (t >= TPYE_WY2 && t <= TPYE_WY8) ||
            t == TPYE_YL || t == TPYE_QJ) ? 1 : 0;
}
/* 只格式化显示值，保留原始数据中的小数和负倾角符号。 */
static void ch_disp_format(char *dst, size_t size, Sensor_Tpye t, uint16_t raw)
{
    int32_t v = (t == TPYE_QJ) ? (int32_t)(int16_t)raw : (int32_t)raw;
    if (ch_disp_scale10(t))
    {
        uint32_t magnitude = (uint32_t)((v < 0) ? -v : v);
        snprintf(dst, size, "%s%lu.%lu", (v < 0) ? "-" : "",
                 (unsigned long)(magnitude / 10), (unsigned long)(magnitude % 10));
    }
    else
        snprintf(dst, size, "%u", (unsigned int)raw);
}

/* 解析主页显示用的通道类型，不改写任何通道数据。 */
static Sensor_Tpye ui_home_channel_type(uint8_t ch)
{
    uint8_t i;

    if (ch >= MAX_CH_NUM)
        return TPYE_NONE;

    /* 绑定完成后，即使实时数据尚未到达，也已经知道通道类型。 */
    for (i = 0u; i < g_binding_count; i++)
    {
        const scan_binding *b = &g_binding_list[i];
        uint16_t start = b->frist_ch_num;
        uint16_t end   = (uint16_t)(start + b->ch_num);

        if ((uint16_t)ch >= start && (uint16_t)ch < end)
        {
            /* 激光四通道：起始通道是激光，其余三路是倾角。 */
            if (b->Type == TPYE_JG && b->ch_num == 4u)
            {
                if ((uint16_t)ch == start)
                    return TPYE_JG;
                return TPYE_QJ;
            }

            if (b->Type > TPYE_NONE && b->Type < TPYE_END)
                return b->Type;
        }
    }

    /* 兼容已写入通道缓存、但绑定表暂时没有对应记录的情况。 */
    if (CH_com_buf[ch].Type > TPYE_NONE &&
        CH_com_buf[ch].Type < TPYE_END)
    {
        return CH_com_buf[ch].Type;
    }

    return TPYE_NONE;
}

/* 画一行文本并返回其像素宽度
 * ★ x/y 用 uint16_t：主页面单位列/占位符在 x=292 / 312，写成 uint8_t 会被截断
 *   （312→56）导致文字画到错位置，编译期也会报 -Woverflow。 */
static uint16_t ui_main_meta_code(const uint8_t **cursor);

/* One font family for labels, values and names; sizes share the same baseline. */
static const ui_menu_glyph_t *ui_text_glyph(uint16_t code, uint8_t size)
{
    const ui_menu_glyph_t *table;
    uint16_t lo = 0u;
    uint16_t hi = (uint16_t)(sizeof(ui_menu_glyphs_14) / sizeof(ui_menu_glyphs_14[0]));
    if (size == 11u) table = ui_menu_glyphs_11;
    else if (size == 16u) table = ui_menu_glyphs_16;
    else if (size == 18u) table = ui_menu_glyphs_18;
    else table = ui_menu_glyphs_14;
    while (lo < hi)
    {
        uint16_t mid = (uint16_t)(lo + (hi - lo) / 2u);
        if (table[mid].code < code) lo = (uint16_t)(mid + 1u);
        else if (table[mid].code > code) hi = mid;
        else return &table[mid];
    }
    return NULL;
}

static uint16_t ui_text_width(const char *text, uint8_t size)
{
    const uint8_t *cursor = (const uint8_t *)text;
    uint16_t width = 0u;
    while (*cursor != '\0')
    {
        const ui_menu_glyph_t *glyph = ui_text_glyph(ui_main_meta_code(&cursor), size);
        if (glyph == NULL) glyph = ui_text_glyph('?', size);
        if (glyph != NULL) width = (uint16_t)(width + glyph->width);
    }
    return width;
}

static uint16_t ui_text_draw(uint16_t x, uint16_t baseline, const char *text, uint8_t size)
{
    const uint8_t *cursor = (const uint8_t *)text;
    uint16_t left = x;
    uint8_t bitmap_mode = u8g2.bitmap_transparency;
#ifdef UI_PREVIEW
    unsigned missing = 0u;
    extern void ui_preview_text(uint16_t, uint16_t, const char *, uint8_t, unsigned);
#endif
    u8g2_SetBitmapMode(&u8g2, 1u);
    while (*cursor != '\0')
    {
        const ui_menu_glyph_t *glyph = ui_text_glyph(ui_main_meta_code(&cursor), size);
        if (glyph == NULL)
        {
#ifdef UI_PREVIEW
            missing++;
#endif
            glyph = ui_text_glyph('?', size);
        }
        if (glyph != NULL)
        {
            ui_fast_DrawXBMP(&u8g2, x, (uint16_t)(baseline - size + 1u),
                          glyph->width, (uint16_t)(size + 2u), glyph->bits);
            x = (uint16_t)(x + glyph->width);
        }
    }
#ifdef UI_PREVIEW
    ui_preview_text(left, baseline, text, size, missing);
#endif
    u8g2_SetBitmapMode(&u8g2, bitmap_mode);
    return (uint16_t)(x - left);
}

static uint16_t ui_draw(uint16_t x, uint16_t y, const char *s)
{
    return ui_text_draw(x, y, s, 14u);
}

/* 选项高亮框：x/y = 文字起点与基线，w = 文字宽度
 * wqy14 墨迹 = [基线-12, 基线+1] ⇒ 框取 [基线-13, 基线+4]（上下各留 1~3px）
 *
 * ==========================================================================
 * ★★ 全工程**唯一**的高亮框绘制点（本次修正）
 * --------------------------------------------------------------------------
 * 屏幕上"方框同时出现在两个位置"的原因：
 *   原约定（老页面 networking_Control1 / addr_Control / calibration_Control …
 *   一直遵守）是——
 *       menu_rank == 2  → 焦点在**左侧菜单**，左侧框由 UI_Menu_Display() 画，
 *                         右侧页面**不许画框**；
 *       menu_rank == 3  → 焦点在**右侧页面**，左侧框不画，右侧页面才画框。
 *   但后来新增的 binding_* / summary_* / install_* 这几页**漏了 menu_rank 判断**，
 *   于是 menu_rank==2 时左侧画一个框、右侧页面又画一个框 ⇒ 同屏两个框。
 *
 * 修正：把判断收敛到本函数这**唯一一处**，条件互斥 ⇒ 同一时刻屏幕上只有一个高亮框。
 *       （左侧框只在 menu_rank==2 画，右侧框只在 menu_rank!=2 画。）
 * ========================================================================== */
static void ui_hl_box(uint16_t x, uint16_t baseline, uint16_t w, uint8_t on)
{
    if (!on) return;
    if (Data_list1.menu_rank == 2) return;   /* 焦点在左侧菜单：右侧一律不画框 */
    u8g2_DrawFrame(&u8g2, (u8g2_uint_t)(x - 5), (u8g2_uint_t)(baseline - 13),
                   (u8g2_uint_t)(w + 10), 18);
}

/* Long BLE names use the same family at 11px and fit the available width. */
static void ui_draw_name_aligned(uint16_t x, uint16_t y, const uint8_t *src,
                                 uint16_t max_w, uint8_t size, uint8_t centered)
{
    char name[MAX_NAME_LEN];
    uint8_t i;
    const uint8_t *cursor;
    uint16_t width = 0u;
    for (i = 0u; i < MAX_NAME_LEN - 1u && src[i] != '\0'; i++) name[i] = (char)src[i];
    name[i] = '\0';
    if (ui_text_width(name, size) > max_w) size = 11u;
    cursor = (const uint8_t *)name;
    while (*cursor != '\0')
    {
        const uint8_t *start = cursor;
        const ui_menu_glyph_t *glyph = ui_text_glyph(ui_main_meta_code(&cursor), size);
        uint16_t next;
        if (glyph == NULL) glyph = ui_text_glyph('?', size);
        next = glyph ? glyph->width : 0u;
        if ((uint16_t)(width + next) > max_w)
        {
            name[start - (const uint8_t *)name] = '\0';
            break;
        }
        width = (uint16_t)(width + next);
    }
    if (centered) x = (uint16_t)(x + (max_w - width) / 2u);
    ui_text_draw(x, y, name, size);
}

static void ui_draw_name_size(uint16_t x, uint16_t y, const uint8_t *src,
                              uint16_t max_w, uint8_t size)
{
    ui_draw_name_aligned(x, y, src, max_w, size, 0u);
}





short x, x_trg;          //          ֵ
short y, y_trg;                 //
short frame_lenth, frame_lenth_trg;
short frame_y, frame_y_trg;

void UI_MoveSet(short *p, short *p_trg, uint8_t step, uint8_t min) {
  step = abs(*p_trg - *p) > min ? step : 1;
  if (*p < *p_trg) {
    *p += step;
  } else if (*p > *p_trg) {
    *p -= step;
  }
}



uint8_t rank=0;

/* ==================================================================
 * ★★ 保存结果提示页（V1_0_2 第六轮新增）
 * ------------------------------------------------------------------
 * 需求：保存必须有明确的成功/失败页面提示，两类来源：
 *   1) 参数存到 STM32 自己的 Data-EEPROM：STM32 发 0x01（menu_rank=6，
 *      参数=UI_MSG_xxx）通知本机显示；
 *   2) 蓝牙绑定存到 CH584M 自己的 Data-Flash（0x05）：本机自己知道结果，
 *      直接本地显示。
 * 机制：
 *   · g_ui_msg 非 0 时，UI_Control() **优先画提示页**，普通页面帧盖不掉它
 *     （但 Data_list1 照常更新 ⇒ 提示消失后自动画回最新页面）；
 *   · 显示时长由 1 秒事件倒数（ui_msg_tick_sec()），到点自动清除并请求重画；
 *   · 提示文本由 ui_draw 使用 ui_menu_assets.h 的 14px 位图字模绘制；
 *     与普通页面共用现有字模，不依赖 UI_FONT_CN 的历史全中文字库。
 * ================================================================== */
volatile uint8_t g_ui_msg      = UI_MSG_NONE;
static   uint8_t s_ui_msg_hold = 0u;

void ui_show_msg(uint8_t code)
{
    /* ★ 第 11 轮：code == UI_MSG_NONE(0) 表示"显式清除提示"。
       软关机提示页（UI_MSG_POWER_OFF）是常驻的，不会自己超时消失，
       必须由 STM32 在正式开机后发一帧 msg=0 把它清掉。 */
    if (code == UI_MSG_NONE)
    {
        if (g_ui_msg != UI_MSG_NONE)
        {
            g_ui_msg      = UI_MSG_NONE;
            s_ui_msg_hold = 0u;
        }
        return;
    }

    g_ui_msg      = code;
    s_ui_msg_hold = UI_MSG_HOLD_SEC;
    /* 重画请求由调用方负责：
       0x01 路径在 app_uart_process() 解析成功后本来就会置 dis_flag_cnt = 1；
       0x05 路径同理；1 秒事件里的延期清 Flash 路径由那里显式置位。 */
}

uint8_t ui_msg_tick_sec(void)
{
    if (g_ui_msg == UI_MSG_NONE) return 0u;

    /* ★ 第 11 轮：软关机提示页常驻，不参与超时倒数
       （由 STM32 发 msg=0 显式清除，见 ui_show_msg()）。 */
    if ((g_ui_msg == UI_MSG_POWER_OFF) || (g_ui_msg == UI_MSG_SHUTTING_DOWN)) return 0u;

    if (s_ui_msg_hold > 0u) s_ui_msg_hold--;

    if (s_ui_msg_hold == 0u)
    {
        g_ui_msg = UI_MSG_NONE;
        return 1u;                 /* 提示刚消失：调用方应请求一次重画 */
    }
    return 0u;
}

static void UI_Message_Display(void)
{
    const char *line = "保存";
    uint16_t w, x, y;
    uint16_t dw, dh;
    uint16_t bw;

    switch (g_ui_msg)
    {
        case UI_MSG_PARAM_SAVE_OK:   line = "参数保存成功"; break;
        case UI_MSG_PARAM_SAVE_FAIL: line = "参数保存失败"; break;
        case UI_MSG_BIND_SAVE_OK:    line = "绑定保存成功"; break;
        case UI_MSG_BIND_SAVE_FAIL:  line = "绑定保存失败"; break;
        case UI_MSG_BIND_SAVE_EMPTY: line = "无绑定可保存"; break;
        case UI_MSG_BIND_CLEAR_OK:   line = "绑定已清除";   break;
        case UI_MSG_BIND_CLEAR_FAIL: line = "绑定清除失败"; break;
        /* ★ 第 11 轮：软关机提示页（常驻，直到 STM32 发 msg=0 清除） */
        case UI_MSG_POWER_OFF:       line = "长按K1+K3开机"; break;
        case UI_MSG_SHUTTING_DOWN:   line = "正在关机中"; break;
        default:                     line = "保存";         break;
    }

    u8g2_SetFontMode(&u8g2, 1);
    u8g2_SetDrawColor(&u8g2, 1);

    dw = (uint16_t)u8g2_GetDisplayWidth(&u8g2);
    dh = (uint16_t)u8g2_GetDisplayHeight(&u8g2);

    w = (uint16_t)ui_text_width(line, 14u);
    x = (uint16_t)((dw > w) ? ((dw - w) / 2u) : 0u);
    y = (uint16_t)((dh / 2u) + 6u);            /* 基线略低于中线（wqy14 墨迹在基线上方） */

    /* 提示框：居中、左右各留 24px、上下留 27px（框内只放一行大字） */
    bw = (uint16_t)(dw - 48u);
    u8g2_DrawFrame(&u8g2, 24u, (u8g2_uint_t)(y - 34u), (u8g2_uint_t)bw, 54u);

    ui_draw(x, y, line);
}

#if UI_LATENCY_DIAG
__HIGH_CODE
static void ui_send_buffer_profiled(uint32_t render_start)
{
    uint32_t transfer_start = TMOS_GetSystemClock();
    ui_profile_render_ticks = (uint32_t)(transfer_start - render_start);
    u8g2_SendBuffer(&u8g2);
    ui_profile_transfer_ticks = (uint32_t)(TMOS_GetSystemClock() - transfer_start);
}
#endif

__HIGH_CODE
void UI_Control(data_LIST *list)
{
    uint8_t rank2_addr=0,rank3_addr=0;
#if UI_LATENCY_DIAG
    uint32_t render_start = TMOS_GetSystemClock();
#endif
    rank=list->menu_rank;
    rank2_addr=list->rank2_addr;
    rank3_addr=list->rank3_addr;

    /* Control commands/no page must preserve panel RAM, never send blank. */
    if (g_ui_msg == UI_MSG_NONE && (rank < 1u || rank > 4u)) return;
    u8g2_ClearBuffer(&u8g2);

    /* ★★ 保存结果提示页优先：只要 g_ui_msg 非 0，就画提示页**并直接返回**。
       这样 STM32 随后发来的普通页面帧（0x01 menu_rank 1/2/3）不会把提示盖掉；
       普通页面的字段仍然照常写进 Data_list1，提示消失后画回的就是最新页面。 */
    if (g_ui_msg != UI_MSG_NONE)
    {
        UI_Message_Display();
#if UI_LATENCY_DIAG
        ui_send_buffer_profiled(render_start);
#else
        u8g2_SendBuffer(&u8g2);
#endif
        return;
    }

   switch (rank)
   {
       case 1:
           UI_Main_Display(&Data_list1);
           break;
       case 2:
       case 3:
           UI_Select=rank2_addr;
           UI_Menu_Display();
           break;
       case 4:
           new_return();
           break;
       case 5:
//           PRINT("rank == %d ;u8g2Init(&u8g2)\r\n",rank);
//           u8g2Init(&u8g2);
//           rank=0;
//
//            app_drv_fifo_init(&app_uart_rx_fifo, app_uart_rx_buffer, 512);
//            Usart3_Init();
           break;
       default:

           break;
   }
#if UI_LATENCY_DIAG
   ui_send_buffer_profiled(render_start);
#else
   u8g2_SendBuffer(&u8g2);
#endif
}
/* ==================================================================
 * 主页面（menu_rank == 1）：按单色 384×168 屏幕适配工业仪表版式
 *
 * 版式：双层切角外框；品牌、标题与页属信息同排；地址/统计独占一行；
 *       左右切角数据栏各 5 行；第二页显示设备/通道统计。
 *
 * 网格：**5 行 × 2 列 = 10 格，每格 1 个通道**，右列紧接左列：
 *   第 1 页 = 通道 1..10（左列 1..5，右列 6..10）
 *   第 2 页 = 通道 11..20（左列 11..15，右列 16..20）
 *   分页由 0x01 帧的 chu_num1 决定（== 2 为第 2 页，其余按第 1 页）。
 * 每格一行式：`切角编号 类型名 数值(右对齐) 单位`
 *   数值：MG/位移/应力/倾角 = 原始值/10，保留一位小数（倾角按有符号），激光原样
 *   单位：kN（锚杆）/ mm（位移·激光·裂缝）/ °（倾角）/ MPa（应力）（契约 §4.2）
 *
 * 数据分工：STM32 仍提供状态/本机号/分站号/电池/LoRa信号值，
 *           版本字段暂不占用主页版面；
 *           CH584M 本地提供 已绑定数、已用通道数、20 通道类型与数值。
 * ================================================================== */
/* MAIN_* 几何常量仍供其他20通道子页复用，数值保持原样。 */
#define MAIN_GRID_TOP     53                        /* 网格上边线 */
#define MAIN_ROW_H        18                        /* 行高（14px 字 + 4px 间隔） */
#define MAIN_ROWS         5                         /* 行数 */
#define MAIN_COL_SPLIT_X  190                       /* 中间竖线 x（图上 184..194） */
#define MAIN_CELL_L_X     8                         /* 左格起点 */
#define MAIN_CELL_R_X     192                       /* 右格起点 */
#define MAIN_NUM_W        20                        /* 编号列宽 */
#define MAIN_TYPE_W       26                        /* 类型列宽（2 个汉字） */
#define MAIN_VAL_RIGHT    150                       /* 数值右对齐（相对格起点） */
#define MAIN_UNIT_X       154                       /* 单位起点（相对格起点） */
#define MAIN_ROW_BASE(i)  ((uint8_t)(MAIN_GRID_TOP + 13 + MAIN_ROW_H * (i)))

/* 主页专用坐标，避免影响信号、电压和名称子页。 */
#define HOME_GRID_TOP     52
#define HOME_ROW_H        22
#define HOME_GRID_H       110
#define HOME_TEXT_SIZE    18u
#define HOME_ROWS         5
#define HOME_CELL_L_X     14
#define HOME_CELL_R_X     203
#define HOME_NUM_W        24
#define HOME_DATA_RIGHT   166
#define HOME_HEADER_BATTERY_RIGHT 376u
#define HOME_HEADER_BATTERY_BASE  29u
#define HOME_HEADER_BATTERY_SIZE  16u
#define HOME_HEADER_BATTERY_MAX_W 58u
#define HOME_HEADER_STATE_RIGHT   372u
#define HOME_HEADER_STATE_BASE    23u
#define HOME_WIRELESS_TOP         10u
#define HOME_WIRELESS_COMPACT_TOP 14u
#define HOME_WIRELESS_FULL_MAX_W  46u
#define HOME_WIRELESS_GAP         3u
#define HOME_STATUS_BASELINE      44u
#define HOME_STATUS_LEFT          108u
#define HOME_STATUS_HOST_RIGHT    366u
#define HOME_BOUND_UNIT_GAP       4u

/* 切角横纵跨度相等，保持 45 度直线，供主页各类边框共用。 */
static void ui_main_cut_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              uint8_t cut)
{
    uint16_t right = (uint16_t)(x + w - 1u);
    uint16_t bottom = (uint16_t)(y + h - 1u);
    uint8_t cut_y = cut;

    u8g2_DrawLine(&u8g2, (u8g2_uint_t)(x + cut), (u8g2_uint_t)y,
                  (u8g2_uint_t)(right - cut), (u8g2_uint_t)y);
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)(right - cut), (u8g2_uint_t)y,
                  (u8g2_uint_t)right, (u8g2_uint_t)(y + cut_y));
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)right, (u8g2_uint_t)(y + cut_y),
                  (u8g2_uint_t)right, (u8g2_uint_t)(bottom - cut_y));
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)right, (u8g2_uint_t)(bottom - cut_y),
                  (u8g2_uint_t)(right - cut), (u8g2_uint_t)bottom);
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)(right - cut), (u8g2_uint_t)bottom,
                  (u8g2_uint_t)(x + cut), (u8g2_uint_t)bottom);
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)(x + cut), (u8g2_uint_t)bottom,
                  (u8g2_uint_t)x, (u8g2_uint_t)(bottom - cut_y));
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)x, (u8g2_uint_t)(bottom - cut_y),
                  (u8g2_uint_t)x, (u8g2_uint_t)(y + cut_y));
    u8g2_DrawLine(&u8g2, (u8g2_uint_t)x, (u8g2_uint_t)(y + cut_y),
                  (u8g2_uint_t)(x + cut), (u8g2_uint_t)y);
}

/* 品牌只包含参考图的标识；数值与状态不进入静态资源。 */
static void ui_main_draw_brand(void)
{
    ui_fast_DrawXBMP(&u8g2, 9u, 5u, UI_HOME_BRAND_WIDTH, UI_HOME_BRAND_HEIGHT,
                  ui_home_brand);
}

/* Two diagonal bands per wing, mirrored around the screen center. */
static void ui_main_draw_title(void)
{
    uint8_t row, offset;
    uint16_t left;

    /* Leave the enlarged wordmark clear below the diagonal wings. */
    for (row = 0u; row < 22u; row++)
    {
        left = (uint16_t)(78u + row);
        u8g2_DrawHLine(&u8g2, left, (uint16_t)(2u + row), 11u);
        u8g2_DrawHLine(&u8g2, (uint16_t)(383u - left - 10u),
                       (uint16_t)(2u + row), 11u);
    }
    u8g2_SetDrawColor(&u8g2, 0);
    for (offset = 0u; offset < 2u; offset++)
    {
        left = (uint16_t)(85u + offset);
        u8g2_DrawLine(&u8g2, left, 4u, (uint16_t)(left + 21u), 25u);
        u8g2_DrawLine(&u8g2, (uint16_t)(383u - left), 4u,
                      (uint16_t)(383u - left - 21u), 25u);
    }
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawHLine(&u8g2, 98u, 28u, 188u);
    ui_fast_DrawXBMP(&u8g2, 112u, 6u, UI_HOME_TITLE_WIDTH, UI_HOME_TITLE_HEIGHT,
                  ui_home_title);
}

/* Shared UTF-8 decoder for all generated font sizes. */
static uint16_t ui_main_meta_code(const uint8_t **cursor)
{
    const uint8_t *p = *cursor;
    uint16_t code = *p++;
    if ((code & 0xe0u) == 0xc0u && (p[0] & 0xc0u) == 0x80u)
    {
        code = (uint16_t)(((code & 0x1fu) << 6u) | (p[0] & 0x3fu));
        p++;
    }
    else if ((code & 0xf0u) == 0xe0u && (p[0] & 0xc0u) == 0x80u &&
             (p[1] & 0xc0u) == 0x80u)
    {
        code = (uint16_t)(((code & 0x0fu) << 12u) |
                          ((p[0] & 0x3fu) << 6u) | (p[1] & 0x3fu));
        p += 2;
    }
    *cursor = p;
    return code;
}

static void ui_main_meta_draw(uint16_t x, uint16_t baseline, const char *text, uint8_t size)
{
    ui_text_draw(x, baseline, text, size);
}

static void ui_main_draw_wireless(uint8_t signal, uint16_t voltage_x, uint16_t voltage_width)
{
    /* STM32L0 HEX sends codes 8..11. User-confirmed display order:
     * 8 = crossed, 9 = weak, 10 = medium, 11 = strong.
     * Unknown codes remain crossed; this is not a link-timeout detector. */
    uint8_t icon = (signal >= 8u && signal <= 11u) ? (uint8_t)(signal - 8u) : 0u;
    if (voltage_width <= HOME_WIRELESS_FULL_MAX_W)
        ui_fast_DrawXBMP(&u8g2, (uint16_t)(voltage_x - UI_WIRELESS_WIDTH - HOME_WIRELESS_GAP),
                      HOME_WIRELESS_TOP, UI_WIRELESS_WIDTH, UI_WIRELESS_HEIGHT,
                      ui_wireless_icons[icon]);
    else
        ui_fast_DrawXBMP(&u8g2, (uint16_t)(voltage_x - UI_WIRELESS_COMPACT_WIDTH - HOME_WIRELESS_GAP),
                      HOME_WIRELESS_COMPACT_TOP, UI_WIRELESS_COMPACT_WIDTH, UI_WIRELESS_COMPACT_HEIGHT,
                      ui_wireless_compact_icons[icon]);
}

/* The optional device-count unit has its own explicit pixel gap. */
static void ui_main_header_pair(const char *left, const char *unit, const char *right)
{
    const uint8_t size = 14u;
    uint16_t right_x = (uint16_t)(HOME_STATUS_HOST_RIGHT - ui_text_width(right, size));
    ui_main_meta_draw(HOME_STATUS_LEFT, HOME_STATUS_BASELINE, left, size);
    if (unit)
        ui_main_meta_draw((uint16_t)(HOME_STATUS_LEFT + ui_text_width(left, size) +
                                    HOME_BOUND_UNIT_GAP), HOME_STATUS_BASELINE, unit, size);
    ui_main_meta_draw(right_x, HOME_STATUS_BASELINE, right, size);
}

/* 通道号在徽标中居中，并为两位数与框线保留一像素空白。 */
static void ui_main_channel_badge(uint16_t x, uint16_t baseline, uint8_t channel)
{
    char label[4];
    uint16_t label_width;
    uint16_t top = (uint16_t)(baseline - 15u);

    ui_main_cut_frame(x, top, 22u, 18u, 2u);
    sprintf(label, "%u", (unsigned int)channel);
    label_width = ui_text_width(label, 14u);
    ui_text_draw((uint16_t)(x + (22u - label_width) / 2u), baseline, label, 14u);
}

void UI_Main_Display(data_LIST *pData)
{
    char     buf[48];
    char     send_str[12];
    uint8_t  i, first, page, row, col, cx, y;
    uint8_t  bitmap_mode = u8g2.bitmap_transparency;
    uint16_t w, grid_top;

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    /* 两页只替换顶部信息和通道集合，Logo、网格及字号共用。 */
    page = (pData->UI_main.chu_num1 == 2u) ? 2u : 1u;
    grid_top = HOME_GRID_TOP;

    /* ---------- 双层切角外框、左侧品牌区与中部标题牌 ---------- */
    u8g2_SetBitmapMode(&u8g2, 1);
    ui_main_cut_frame(1u, 1u, 382u, 166u, 10u);
    /* 保持外侧两像素、内侧一像素，切角端点与直边对齐。 */
    ui_main_cut_frame(2u, 2u, 380u, 164u, 9u);
    ui_main_cut_frame(4u, 4u, 376u, 160u, 7u);
    ui_main_draw_brand();
    ui_main_draw_title();

    /* Page one owns the voltage and controller wireless indicator. */
    if (page == 1u)
    {
        uint8_t voltage_size = HOME_HEADER_BATTERY_SIZE;
        sprintf(buf, "%d.%d%dV",
                pData->UI_main.vbat / 100, pData->UI_main.vbat % 100 / 10,
                pData->UI_main.vbat % 100 % 10);
        w = ui_text_width(buf, HOME_HEADER_BATTERY_SIZE);
        /* Reserve the enlarged icon and its gap even for uint16_t extremes. */
        if (w > HOME_HEADER_BATTERY_MAX_W)
        {
            voltage_size = 14u;
            w = ui_text_width(buf, voltage_size);
        }
        ui_main_meta_draw((uint16_t)(HOME_HEADER_BATTERY_RIGHT - w),
                          HOME_HEADER_BATTERY_BASE, buf, voltage_size);
        ui_main_draw_wireless(pData->UI_main.Lora_rssi,
                              (uint16_t)(HOME_HEADER_BATTERY_RIGHT - w), w);
        if (pData->UI_main.send_host_num == 122u)      sprintf(send_str, "分站");
        else if (pData->UI_main.send_host_num == 121u) sprintf(send_str, "无");
        else if (pData->UI_main.send_host_num == 0u)   sprintf(send_str, "中继");
        else                                           sprintf(send_str, "%d", pData->UI_main.send_host_num);

        {
            char station_str[20];
            sprintf(buf, "本机号:%d-->%s", pData->UI_main.host_num, send_str);
            sprintf(station_str, "分站号:%d", pData->UI_main.sub_num);
            ui_main_header_pair(station_str, NULL, buf);
        }
    }
    else
    {
        const char *state = pData->UI_main.state == 1u ? "状态:开机" : "状态:关机";
        w = ui_text_width(state, 14u);
        ui_main_meta_draw((uint16_t)(HOME_HEADER_STATE_RIGHT - w),
                          HOME_HEADER_STATE_BASE, state, 14u);
    }

    /* ---------- 两页共用同一位置的左右数据框 ---------- */
    ui_main_cut_frame(7u, grid_top, 181u, HOME_GRID_H, 6u);
    ui_main_cut_frame(196u, grid_top, 181u, HOME_GRID_H, 6u);

    for (i = 1; i < HOME_ROWS; i++)
    {
        uint8_t line_y = (uint8_t)(grid_top + HOME_ROW_H * i);
        u8g2_DrawHLine(&u8g2, 7u, line_y, 181u);
        u8g2_DrawHLine(&u8g2, 196u, line_y, 181u);
    }
    /* ---------- 通道网格：分页、类型、值和单位处理保持原样 ---------- */
    first = (uint8_t)((page == 1u) ? 0u : 10u);

    for (i = 0; i < 10; i++)
    {
        uint8_t   ch  = (uint8_t)(first + i);
        device_t *d   = &CH_com_buf[ch];
        Sensor_Tpye display_type = ui_home_channel_type(ch);

        row = (uint8_t)(i % HOME_ROWS);                /* 0..4 */
        col = (uint8_t)(i / HOME_ROWS);                /* 0 = 左列，1 = 右列 */
        cx  = (uint8_t)(col ? HOME_CELL_R_X : HOME_CELL_L_X);
        y   = (uint8_t)(grid_top + 18u + HOME_ROW_H * row);

        ui_main_channel_badge(cx, y, (uint8_t)(ch + 1u));
        /* Channel labels exist before binding; types/units come from binding metadata. */
        if (display_type > TPYE_NONE && display_type < TPYE_END)
        {
            uint16_t value_w, unit_w, unit_x;
            const char *unit = CH_TYPE_UNIT[display_type];
            ui_text_draw((uint16_t)(cx + HOME_NUM_W), y, CH_TYPE_NAME[display_type], HOME_TEXT_SIZE);
            unit_w = ui_text_width(unit, HOME_TEXT_SIZE);
            unit_x = (uint16_t)(cx + HOME_DATA_RIGHT - unit_w);
            if (d->valid && d->data_re_flag)
                ch_disp_format(buf, sizeof(buf), display_type, d->CH_data);
            else strcpy(buf, "--");
            value_w = ui_text_width(buf, HOME_TEXT_SIZE);
            ui_text_draw((uint16_t)(unit_x - (unit_w ? 4u : 0u) - value_w), y, buf, HOME_TEXT_SIZE);
            if (unit_w) ui_text_draw(unit_x, y, unit, HOME_TEXT_SIZE);
        }
        else
        {
            ui_text_draw((uint16_t)(cx + HOME_NUM_W), y, "--", HOME_TEXT_SIZE);
            ui_text_draw((uint16_t)(cx + HOME_DATA_RIGHT - ui_text_width("--", HOME_TEXT_SIZE)), y, "--", HOME_TEXT_SIZE);
        }
    }

    /* Page two has device/channel counts, without the old alarm field. */
    if (page == 2u)
    {
        char bound_str[20], used_str[24];
        sprintf(bound_str, "已绑定:%u", (unsigned int)g_binding_count);
        sprintf(used_str, "已用通道:%d", count_used_channels());
        ui_main_header_pair(bound_str, "台", used_str);
    }
    u8g2_SetBitmapMode(&u8g2, bitmap_mode);
}

/* Menu-only artwork and layout. Runtime fields and selection IDs stay intact. */
static uint16_t ui_menu_text_width(const char *text, uint8_t size)
{
    return ui_text_width(text, size);
}

static void ui_menu_text(uint16_t x, uint16_t baseline, const char *text, uint8_t size)
{
    ui_text_draw(x, baseline, text, size);
}

static uint8_t ui_menu_selected(uint8_t option)
{
    return (uint8_t)(Data_list1.menu_rank == 3u && Data_list1.rank3_addr == option);
}

/* A single selected control is inverted; passive field outlines stay thin. */
static void ui_menu_control(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            const char *text, uint8_t selected, uint8_t centered)
{
    uint16_t tw = ui_menu_text_width(text, 16u);
    uint16_t tx;
    u8g2_SetDrawColor(&u8g2, 0);
    if (selected) ui_fast_DrawBox(&u8g2, x, y, w, h);
    else u8g2_DrawFrame(&u8g2, x, y, w, h);
    u8g2_SetDrawColor(&u8g2, selected ? 1u : 0u);
    if (tw <= w - 8u)
    {
        tx = centered ? (uint16_t)(x + (w - tw) / 2u) : (uint16_t)(x + 5u);
        ui_menu_text(tx, (uint16_t)(y + (h - 18u) / 2u + 15u), text, 16u);
    }
    else
    {
        /* Long counters use the existing 14px bitmap glyphs. */
        tw = (uint16_t)ui_text_width(text, 14u);
        tx = centered && tw <= w - 8u ? (uint16_t)(x + (w - tw) / 2u) : (uint16_t)(x + 5u);
        ui_draw(tx, (uint16_t)(y + (h - 14u) / 2u + 12u), text);
    }
    u8g2_SetDrawColor(&u8g2, 0);
}

static void ui_menu_button(uint8_t right, const char *text, uint8_t selected)
{
    ui_menu_control(right ? 247u : 117u, 137u, 128u, 25u, text, selected, 1u);
}

static void ui_menu_field(uint16_t y, const char *label, const char *value,
                          uint8_t label_selected, uint8_t value_selected)
{
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawFrame(&u8g2, 117u, y, 207u, 27u);
    if (label_selected) ui_fast_DrawBox(&u8g2, 119u, (uint16_t)(y + 2u), 117u, 23u);
    if (value_selected) ui_fast_DrawBox(&u8g2, 238u, (uint16_t)(y + 2u), 84u, 23u);
    u8g2_SetDrawColor(&u8g2, label_selected ? 1u : 0u);
    ui_menu_text(122u, (uint16_t)(y + 20u), label, 16u);
    u8g2_SetDrawColor(&u8g2, value_selected ? 1u : 0u);
    ui_menu_text(250u, (uint16_t)(y + 20u), value, 16u);
    u8g2_SetDrawColor(&u8g2, 0);
}

static void ui_menu_header(uint8_t menu)
{
    if (menu >= MAIN_LEN) menu = 0u;
    u8g2_SetDrawColor(&u8g2, 1);
    ui_fast_DrawBox(&u8g2, 113u, 1u, 266u, 166u);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawRFrame(&u8g2, 114u, 5u, 264u, 159u, 4u);
    /* Keep the same icon, title baseline and divider at menu ranks 2 and 3. */
    ui_fast_DrawXBMP(&u8g2, 118u, 10u, 29u, 27u, ui_menu_title_icons[menu]);
    ui_menu_text(156u, 31u, Menu_List[menu].name, 18u);
    u8g2_DrawHLine(&u8g2, 119u, 42u, 254u);
}

void UI_Menu_Display(void)
{
    uint8_t i;
    uint8_t bitmap_mode = u8g2.bitmap_transparency;
    uint8_t font_mode = u8g2.font_decode.is_transparent;
    uint8_t draw_color = u8g2.draw_color;
    if (UI_Select >= MAIN_LEN) UI_Select = 0;
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    /* A subpage belongs to its menu; stale re_flag cannot hide another menu. */
    if (Data_list1.menu_rank == 3)
    {
        uint8_t re_top = Data_list1.UI_main.re_flag;
        if (UI_Select == 2u && (re_top == 1u || re_top == 2u))
        {
            binding_Control();
            goto restore_style;
        }
        if (UI_Select == 3u && (re_top == 3u || re_top == 4u))
        {
            install_Control();
            goto restore_style;
        }
        if (UI_Select == 6u && re_top == 5u)
        {
            summary_Control();
            goto restore_style;
        }
    }

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawFrame(&u8g2, 1u, 1u, 382u, 166u);
    u8g2_DrawRFrame(&u8g2, 6u, 5u, 103u, 159u, 4u);
    u8g2_DrawFrame(&u8g2, 8u, 7u, 99u, 155u);
    for (i = 0u; i < MAIN_LEN; i++)
    {
        uint16_t top = (uint16_t)(9u + 19u * i);
        uint8_t selected = (uint8_t)(Data_list1.menu_rank == 2u && UI_Select == i);
        u8g2_SetDrawColor(&u8g2, 1);
        if (selected) ui_fast_DrawBox(&u8g2, 10u, top, 95u, 19u);
        else if (UI_Select == i) u8g2_DrawVLine(&u8g2, 10u, top, 19u);
        u8g2_SetDrawColor(&u8g2, selected ? 0u : 1u);
        ui_fast_DrawXBMP(&u8g2, 14u, (uint16_t)(top + 3u), 13u, 13u, ui_menu_nav_icons[i]);
        ui_menu_text(32u, (uint16_t)(top + 15u), Main_List[i].name, 16u);
    }
    ui_menu_header(UI_Select);
    Menu_List[UI_Select].function();

restore_style:
    /* Invert the completed drawing for all secondary and third-level pages.
     * Ordinary pages use only the right pane; detail data/binding pages
     * replace the whole screen, so their inversion must cover it all. */
    if (Data_list1.menu_rank == 2u || Data_list1.menu_rank == 3u)
    {
        uint8_t re = Data_list1.UI_main.re_flag;
        /* ★ V1_0_10：组网测试页（UI_Select==1）真正进入子页后也整页自绘
           （会盖住左侧菜单列表），所以必须按**整屏**反白，
           否则左半边不会被反白、会残留菜单列表的痕迹。
           注意只在 menu_rank==3（已进入子页）时才算整屏 ——
           rank==2 时这里画的是"菜单列表预览"，仍只占右窗格。 */
        uint8_t full_screen = (uint8_t)((UI_Select == 1u && Data_list1.menu_rank == 3u &&
                                         Data_list1.Menu_rank2.net_flag != NET_FLAG_IDLE) ||
            (Data_list1.menu_rank == 3u &&
            ((UI_Select == 2u && (re == 1u || re == 2u)) ||
             (UI_Select == 3u && (re == 3u || re == 4u)) ||
             (UI_Select == 6u && re == 5u))));
        u8g2_SetDrawColor(&u8g2, 2u);
        if (full_screen) ui_fast_DrawBox(&u8g2, 1u, 1u, 382u, 166u);
        else ui_fast_DrawBox(&u8g2, 113u, 1u, 266u, 166u);
    }
    u8g2_SetBitmapMode(&u8g2, bitmap_mode);
    u8g2_SetFontMode(&u8g2, font_mode);
    u8g2_SetDrawColor(&u8g2, draw_color);
}

/* ==================================================================
 * 「设备绑定」页（rank2_addr == 2，契约 §1.2）
 *
 *   rank3_addr（列表高亮项）：0 = 已绑定设备:个数 / 1 = 一键解绑 /
 *                            2 = 绑定设备 / 3 = 返回
 *   re_flag（二级子页，契约 §2.3）：1 = 一键解绑子页；2 = 绑定设备子页；
 *                            0 且 rank3==0 = 已绑定设备子页
 *
 *   ★ CH584M 只画不执行：真正的解绑由 STM32 发 0x04、保存由 0x05 执行。
 *   ★ 设备行与绑定页一致，只显示序号和名称，不显示 MAC。
 *     全屏两列最多20个设备，不轮显。
 * ================================================================== */

/* 一键解绑全屏子页：最多20台同屏，保留名称及主控按钮编号。 */
static void binding_device_list_page(uint8_t rank3)
{
    char line[48];
    uint8_t i, n, rows_per_column;
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_fast_DrawBox(&u8g2, 1u, 1u, 382u, 166u);
    u8g2_SetDrawColor(&u8g2, 0u);
    u8g2_DrawRFrame(&u8g2, 2u, 1u, 380u, 166u, 4u);
    ui_fast_DrawXBMP(&u8g2, 10u, 6u, 13u, 13u, ui_menu_nav_icons[2]);
    ui_menu_text(29u, 20u, "设备绑定", 18u);
    sprintf(line, "总设备数:%d台", g_binding_count);
    ui_text_draw(108u, 20u, line, 18u);
    n = g_binding_count;
    if (n > MAX_CH_NUM) n = MAX_CH_NUM;
    /* Fixed columns: devices 1-10 on the left, 11-20 on the right. */
    rows_per_column = 10u;
    if (n == 0u) ui_text_draw(14u, 33u, "--", 11u);
    for (i = 0u; i < n; i++)
    {
        uint16_t x = (uint16_t)(14u + 189u * (i / rows_per_column));
        uint16_t y = (uint16_t)(33u + 11u * (i % rows_per_column));
        sprintf(line, "%u:", (unsigned int)(i + 1u));
        ui_text_draw(x, y, line, 11u);
        ui_draw_name_size((uint16_t)(x + 24u), y, g_binding_list[i].name, 147u, 11u);
    }
    ui_menu_control(6u, 140u, 183u, 22u, "解绑", (uint8_t)(rank3 == 0u), 1u);
    ui_menu_control(195u, 140u, 183u, 22u, "返回", (uint8_t)(rank3 != 0u), 1u);
}

/* 绑定设备子页（re_flag==2）：同屏显示全部绑定名称，不分页、不轮显。
 *   "保存目前设备" = STM32 发 0x05（CH584M 在 case 0x05 里落盘绑定表）。 */
static void binding_scan_page(uint8_t rank3)
{
    uint8_t i, n, rows_per_column;
    uint16_t column_step, name_width, row_step;
    char number[32];
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_fast_DrawBox(&u8g2, 1u, 1u, 382u, 166u);
    u8g2_SetDrawColor(&u8g2, 0u);
    u8g2_DrawRFrame(&u8g2, 2u, 1u, 380u, 166u, 4u);
    ui_fast_DrawXBMP(&u8g2, 10u, 6u, 13u, 13u, ui_menu_nav_icons[2]);
    ui_menu_text(29u, 20u, "设备绑定", 18u);
    n = g_binding_count;
    if (n > MAX_BINDING_NUM) n = MAX_BINDING_NUM;
    sprintf(number, "已绑定设备:%u台", (unsigned int)n);
    ui_text_draw(108u, 20u, number, 18u);
    /* Normal distinct devices occupy at least one of twenty channels. Keep
     * every record visible even if a restored table contains duplicates. */
    /* Normal records keep fixed columns: left 1-10, right 11-20. */
    rows_per_column = n <= MAX_CH_NUM ? 10u : 8u;
    column_step = n <= MAX_CH_NUM ? 189u : 92u;
    name_width = n <= MAX_CH_NUM ? 147u : 64u;
    row_step = n <= MAX_CH_NUM ? 11u : 13u;
    if (n == 0u) ui_text_draw(14u, 33u, "--", 11u);
    for (i = 0u; i < n; i++)
    {
        uint16_t x = (uint16_t)(14u + column_step * (i / rows_per_column));
        uint16_t y = (uint16_t)(33u + row_step * (i % rows_per_column));
        sprintf(number, "%u:", (unsigned int)(i + 1u));
        ui_text_draw(x, y, number, 11u);
        ui_draw_name_size((uint16_t)(x + 24u), y, g_binding_list[i].name,
                          name_width, 11u);
    }
    ui_menu_control(6u, 140u, 183u, 22u, "保存目前设备",
                     (uint8_t)(Data_list1.menu_rank == 3u && rank3 == 0u), 1u);
    ui_menu_control(195u, 140u, 183u, 22u, "返回",
                     (uint8_t)(Data_list1.menu_rank == 3u && rank3 != 0u), 1u);
}

void binding_Control(void)
{
    uint8_t re = Data_list1.UI_main.re_flag;
    uint8_t rank3 = Data_list1.rank3_addr;
    char buf[48];
    if (re == 2 && Data_list1.menu_rank == 3u) { binding_scan_page(rank3); return; }
    if (re == 1 && Data_list1.menu_rank == 3u) { binding_device_list_page(rank3); return; }
    sprintf(buf, "已绑定设备:%u台", (unsigned int)g_binding_count);
    ui_menu_control(117u, 50u, 258u, 25u, buf, ui_menu_selected(0u), 0u);
    ui_menu_control(117u, 79u, 258u, 25u, "一键解绑", ui_menu_selected(1u), 0u);
    ui_menu_control(117u, 108u, 258u, 25u, "绑定设备", ui_menu_selected(2u), 0u);
    ui_menu_button(1u, "返回", ui_menu_selected(3u));
}

/* ==================================================================
 * 「信息汇总」页（rank2_addr == 6，契约 §1.4）
 *
 *   rank3_addr：0 = 查看已绑定的设备（子页 re_flag=5）
 *               1 = 蓝牙名称错误警报:个数（就地显示 g_name_err_count）
 *               2 = 蓝牙电压异常警报:个数（就地显示 g_volt_err_count）
 *               3 = 恢复出厂（确认子页 re_flag=6）
 *               4 = 返回
 *   ★ 恢复出厂的实际执行 = STM32 发 0x07（CH584M 在 case 0x07 里清 RAM + 擦 Flash）。
 * ================================================================== */

/* Name/RSSI/voltage pages show all twenty channels in two columns of ten.
 * Old chu_num2 page codes no longer select a channel subset on these pages.
 * The controller still owns the keys and exits via the existing 0x01 frame.
 * Drawing never changes page state, binding metadata or received values.
 * Home keeps its separate two-page layout. */
#define PARAM_ROWS         10u
#define PARAM_GRID_TOP     22u
#define PARAM_ROW_H        14u
#define PARAM_GRID_H       142u
#define PARAM_TEXT_SIZE    11u
#define PARAM_CELL_L_X     13u
#define PARAM_CELL_R_X     202u
#define PARAM_NUMBER_W     16u
#define PARAM_NAME_OFFSET  24u
#define PARAM_DATA_W       144u

static void ui_param_number(uint16_t x, uint16_t baseline, uint8_t channel)
{
    char label[4];
    uint16_t width;
    sprintf(label, "%u", (unsigned int)channel);
    width = ui_text_width(label, PARAM_TEXT_SIZE);
    ui_text_draw((uint16_t)(x + (PARAM_NUMBER_W - width) / 2u), baseline, label, PARAM_TEXT_SIZE);
}

static void ui_param_value(uint16_t x, uint16_t baseline, const char *text)
{
    uint16_t width = ui_text_width(text, PARAM_TEXT_SIZE);
    ui_text_draw((uint16_t)(x + PARAM_NAME_OFFSET + (PARAM_DATA_W - width) / 2u),
                  baseline, text, PARAM_TEXT_SIZE);
}

static void ui_param_frame(void)
{
    uint8_t i;
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_fast_DrawBox(&u8g2, 1u, 1u, 382u, 166u);
    u8g2_SetDrawColor(&u8g2, 0u);
    u8g2_DrawRFrame(&u8g2, 2u, 1u, 380u, 166u, 4u);
    ui_fast_DrawXBMP(&u8g2, 10u, 6u, 13u, 13u, ui_menu_nav_icons[UI_Select]);
    ui_menu_text(29u, 20u, Menu_List[UI_Select].name, 18u);
    /* Compact header: preserve the original upper-right return position. */
    ui_fast_DrawBox(&u8g2, 298u, 3u, 75u, 18u);
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_menu_text((uint16_t)(298u + (75u - ui_menu_text_width("返回", 14u)) / 2u),
                  16u, "返回", 14u);
    u8g2_SetDrawColor(&u8g2, 0u);
    ui_main_cut_frame(6u, PARAM_GRID_TOP, 183u, PARAM_GRID_H, 4u);
    ui_main_cut_frame(195u, PARAM_GRID_TOP, 183u, PARAM_GRID_H, 4u);
    for (i = 1u; i < PARAM_ROWS; i++)
    {
        uint16_t y = (uint16_t)(PARAM_GRID_TOP + PARAM_ROW_H * i + 1u);
        u8g2_DrawHLine(&u8g2, 6u, y, 183u);
        u8g2_DrawHLine(&u8g2, 195u, y, 183u);
    }
}

static void summary_name_page(void)
{
    uint8_t ch;
    uint16_t cx, y, name_x;
    device_t *d;

    ui_param_frame();
    for (ch = 0u; ch < MAX_CH_NUM; ch++)
    {
        cx = (uint16_t)((ch / PARAM_ROWS) ? PARAM_CELL_R_X : PARAM_CELL_L_X);
        y = (uint16_t)(PARAM_GRID_TOP + 11u + PARAM_ROW_H * (ch % PARAM_ROWS));
        d = &CH_com_buf[ch];
        ui_param_number(cx, y, (uint8_t)(ch + 1u));
        name_x = (uint16_t)(cx + PARAM_NAME_OFFSET);
        if (d->valid && d->name[0] != '\0')
            ui_draw_name_aligned(name_x, y, d->name, PARAM_DATA_W, PARAM_TEXT_SIZE, 1u);
        else ui_param_value(cx, y, "--");
    }
}

/* 恢复出厂确认子页（re_flag==6）：两项，高亮由 rank3 决定（0=确认，其余=返回） */
static void summary_factory_page(uint8_t rank3)
{
    /* Center the confirmation within the right content pane. */
    const char *prompt = "确认恢复出厂设置?";
    ui_menu_text((uint16_t)(113u + (266u - ui_menu_text_width(prompt, 16u)) / 2u),
                 83u, prompt, 16u);
    ui_menu_button(0u, "确认", (uint8_t)(Data_list1.menu_rank == 3u && rank3 == 0u));
    ui_menu_button(1u, "返回", (uint8_t)(Data_list1.menu_rank == 3u && rank3 != 0u));
}

void summary_Control(void)
{
    uint8_t re = Data_list1.UI_main.re_flag;
    uint8_t rank3 = Data_list1.rank3_addr;
    char buf[48];
    if (re == 5) { summary_name_page(); return; }
    if (re == 6) { summary_factory_page(rank3); return; }
    sprintf(buf, "查看已绑定的设备:%u台", (unsigned int)g_binding_count);
    ui_menu_control(117u, 48u, 258u, 20u, buf, ui_menu_selected(0u), 0u);
    sprintf(buf, "蓝牙名称错误警报:%d", g_name_err_count);
    ui_menu_control(117u, 70u, 258u, 20u, buf, ui_menu_selected(1u), 0u);
    sprintf(buf, "蓝牙电压异常警报:%d", g_volt_err_count);
    ui_menu_control(117u, 92u, 258u, 20u, buf, ui_menu_selected(2u), 0u);
    ui_menu_control(117u, 114u, 258u, 20u, "恢复出厂", ui_menu_selected(3u), 0u);
    ui_menu_button(1u, "返回", ui_menu_selected(4u));
}

typedef struct {

    uint16_t posx;                   // 文本起始x坐标
    uint16_t posy;                   // 文本基线y坐标
    uint8_t font_size;            // 字体大小（16/24/32）
    uint16_t label_wight;         // 显示文本的宽度
    uint8_t  kuang_wight;         //方框离文字的宽
    uint8_t  kuang_high;          //方框离文字的高

} OptionItem;
void addr_Control(void)
{
    char value[20];
    sprintf(value, "%d", Data_list1.Menu_rank1.set_sub_num);
    ui_menu_field(62u, "分站号:", value, ui_menu_selected(0u), ui_menu_selected(1u));
    sprintf(value, "%d", Data_list1.Menu_rank1.set_host_num);
    ui_menu_field(99u, "本机地址:", value, ui_menu_selected(2u), ui_menu_selected(3u));
    ui_fast_DrawXBMP(&u8g2, 325u, 73u, 47u, 52u, ui_menu_detail_address);
    ui_menu_button(0u, "保存并返回", ui_menu_selected(4u));
    ui_menu_button(1u, "不保存返回", ui_menu_selected(5u));
}

void addr_Control1(void)
{
    uint8_t menu_rank = Data_list1.menu_rank;
    uint8_t rank3_addr = Data_list1.rank3_addr;

    // 定义每个选项的配置：{文本(为NULL表示数值), x, y, 数值来源}
    typedef struct {
        const char* text;       // 标签文本，若为NULL表示动态数值
        uint8_t x;
        uint8_t y;
        uint16_t value;         // 数值（仅当 text==NULL 时使用）
    } ItemConfig;

    // 配置表（共6个元素）
    ItemConfig configs[] = {
        {"分区:",    120, 80,  0},
        {NULL,       250, 80,  Data_list1.Menu_rank1.set_sub_num},   // 分区值
        {"地址:",    120, 120, 0},
        {NULL,       250, 120, Data_list1.Menu_rank1.set_host_num},  // 地址值
        {"保存并重启", 120, 156, 0},
        {"不保存返回", 250, 156, 0}
    };
    #define OPTION_COUNT (sizeof(configs)/sizeof(configs[0]))

    OptionItem laber[OPTION_COUNT];

    // 循环绘制并计算宽度
    for (int i = 0; i < OPTION_COUNT; i++)
    {
        // 填充固定属性
        laber[i].posx = configs[i].x;
        laber[i].posy = configs[i].y;
        laber[i].font_size = 24;
        laber[i].kuang_wight = 4;
        laber[i].kuang_high = 4;

        // 决定显示文本
        char dyn_buf[20] = {0};
        const char* display_text;
        if (configs[i].text == NULL) {
            // 数值选项：生成数字字符串
            sprintf(dyn_buf, "%d", configs[i].value);
            display_text = dyn_buf;
        } else {
            display_text = configs[i].text;
        }

        // 绘制并获取宽度
        ui_draw(laber[i].posx, laber[i].posy, display_text);
        laber[i].label_wight = ui_text_width(display_text, 14u);
    }

    // ---------- 画框 ----------
    if (menu_rank == 3 && rank3_addr < OPTION_COUNT)
    {
        OptionItem *sel = &laber[rank3_addr];
        uint16_t padding = sel->kuang_wight;
        // 微调 y 偏移：使用 font_size - 2 使框更贴合文字（可选）
        uint16_t y_offset = sel->font_size - 2;   // 24-2=22，可改为 sel->font_size 保持原样
        uint16_t x = sel->posx - padding;
        uint16_t y = sel->posy - y_offset;
        uint16_t w = sel->label_wight + padding * 2;
        uint16_t h = sel->font_size + sel->kuang_high * 2;
        u8g2_DrawFrame(&u8g2, x, y, w, h);
    }
}
void networking_Control1(void)
{
    /* Screen-only menu: STM32 must cycle/confirm only 0=start, 1=return.
     * Never remap obsolete STM32 focus IDs 2/3 to either live action. */
    ui_menu_text(123u, 63u, "组网", 16u);
    u8g2_DrawFrame(&u8g2, 130u, 74u, 25u, 18u);
    u8g2_DrawFrame(&u8g2, 133u, 77u, 19u, 12u);
    u8g2_DrawVLine(&u8g2, 142u, 92u, 5u);
    u8g2_DrawHLine(&u8g2, 136u, 97u, 13u);
    u8g2_DrawHLine(&u8g2, 161u, 84u, 63u);
    ui_fast_DrawXBMP(&u8g2, 226u, 71u, 29u, 27u, ui_menu_title_icons[1]);
    u8g2_DrawHLine(&u8g2, 258u, 84u, 63u);
    u8g2_DrawFrame(&u8g2, 327u, 73u, 21u, 25u);
    ui_menu_control(117u, 117u, 128u, 25u, "开始组网", ui_menu_selected(0u), 1u);
    ui_menu_control(247u, 117u, 128u, 25u, "返回", ui_menu_selected(1u), 1u);
}

/* ★ V1_0_10：原来的 networking_Control2()（只显示"当前那一个地址"的五行）
   已被 networking_Result_Page() 整页表取代，函数删除。
   它引用的 text_host_addr / text_cnt / bl_numl / xuhao_num 字段也一并去掉。 */
/* ==================================================================
 * 「组网测试」结果页（★ V1_0_10 全新排版）
 * ------------------------------------------------------------------
 * 需求：一次把**本机地址**和**三个测试地址**的 (地址, 已测次数/10, 成功率)
 *       全部排在同一屏上，一眼看全，不再一行一行翻。
 * 状态（由下发的 net_flag 决定，见 yuying_TFT.h 的 NET_FLAG_xxx）：
 *   0       空闲   —— 底部左键「开始组网」
 *   1~3     进行中 —— 底部左键「组网进行中」（只作状态标签），正在测的那一行反白
 *   4       已完成 —— 底部左键「重新组网」，结果留在屏上不自动返回
 *   两种非进行中状态下，底部右侧都是「返回」，可高亮可选。
 * ------------------------------------------------------------------
 * 区域核算（面板 300x400，U8G2_R3 旋转后可用 400x300，子页整屏框 1..167）：
 *   表头 26..42；三个数据行 44..134（每行 30）；底部按钮 140..162
 *   => 合计 26..162，正好落在 1..167 内，且比原来"右窗格 258x120"宽出 124px，
 *      因此 Table 的"地址 / 次数 / 比率"三列可以各自独立对齐、不用挤。
 * ================================================================== */
static void networking_Result_Page(void)
{
    char buf[24];
    uint8_t i;
    uint16_t flag    = Data_list1.Menu_rank2.net_flag;
    uint8_t  running = (uint8_t)(flag != NET_FLAG_IDLE && flag != NET_FLAG_DONE);
    uint8_t  done    = (uint8_t)(flag == NET_FLAG_DONE);

    /* ---- 整页底 + 圆角外框（与设备绑定/参数页同一版式） ---- */
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_fast_DrawBox(&u8g2, 1u, 1u, 382u, 166u);
    u8g2_SetDrawColor(&u8g2, 0u);
    u8g2_DrawRFrame(&u8g2, 2u, 1u, 380u, 166u, 4u);
    ui_fast_DrawXBMP(&u8g2, 10u, 6u, 13u, 13u, ui_menu_nav_icons[1]);
    ui_menu_text(29u, 20u, "组网测试", 18u);

    /* ---- 右上角：本机地址 ---- */
    sprintf(buf, "本机地址:%d", (int)Data_list1.Menu_rank2.now_host_addr);
    {
        uint16_t w = ui_menu_text_width(buf, 18u);
        ui_menu_text((uint16_t)(376u - w), 20u, buf, 18u);
    }

    /* ---- 表头 ---- */
    u8g2_DrawHLine(&u8g2, 6u, 26u, 372u);
    /* ★ V1_0_11：表头字号由 14 提到 18（与本机地址同号），原来太小看不清。 */
    ui_menu_text( 30u, 40u, "测试地址", 18u);
    ui_menu_text(178u, 40u, "测试次数", 18u);
    ui_menu_text(286u, 40u, "成功比率", 18u);
    u8g2_DrawHLine(&u8g2, 6u, 42u, 372u);

    /* ---- 三个测试地址，各占一行 ---- */
    for (i = 0u; i < 3u; i++)
    {
        uint16_t y    = (uint16_t)(44u + 30u * i);
        uint16_t cnt  = Data_list1.Menu_rank2.cnt[i];
        uint16_t succ = Data_list1.Menu_rank2.succ[i];
        uint16_t pct  = (cnt == 0u) ? 0u
                       : (uint16_t)(((uint32_t)succ * 100u + (uint32_t)(cnt / 2u)) / cnt);
        uint8_t  cur  = (uint8_t)(running && Data_list1.Menu_rank2.cur_index == i);

        if (i != 0u) u8g2_DrawHLine(&u8g2, 6u, y, 372u);

        /* ★★★ V1_0_12：正在测的那一行改成与「菜单/其它选项被选中」**完全一样**的画法。
           ------------------------------------------------------------------
           为什么颜色号看着是反的：右侧页面在 UI_Menu_Display() 末尾会被
           **整体反相一次**（SetDrawColor(2) + DrawBox），所以本函数里的颜色号
           和肉眼看到的正好相反：
               代码 color 0  ->  屏上"亮/白"
               代码 color 1  ->  屏上"暗/黑"
           页底用 color 1（屏上黑）、普通文字用 color 0（屏上白）=> 黑底白字；
           而标准"选中"效果（ui_menu_control 的 selected、以及左侧菜单列表）
           是 **先填 color 0（屏上白）、再把文字换成 color 1（屏上黑）**
           => 白底黑字。这里就照这个来，和别处选中项长得一模一样。 */
        if (cur)
        {
            u8g2_SetDrawColor(&u8g2, 0u);
            ui_fast_DrawBox(&u8g2, 7u, (uint16_t)(y + 1u), 370u, 28u);
        }
        u8g2_SetDrawColor(&u8g2, cur ? 1u : 0u);

        if      (Data_list1.Menu_rank2.addr[i] == 0u)   sprintf(buf, "中继");
        else if (Data_list1.Menu_rank2.addr[i] == 121u) sprintf(buf, "无");
        else if (Data_list1.Menu_rank2.addr[i] == 122u) sprintf(buf, "分站");
        else sprintf(buf, "%d 号", (int)Data_list1.Menu_rank2.addr[i]);
        ui_menu_text(44u, (uint16_t)(y + 21u), buf, 18u);

        sprintf(buf, "%d / 10", (int)cnt);
        ui_menu_text(178u, (uint16_t)(y + 21u), buf, 18u);

        sprintf(buf, "%d %%", (int)pct);
        ui_menu_text(286u, (uint16_t)(y + 21u), buf, 18u);

        u8g2_SetDrawColor(&u8g2, 0u);
    }

    /* ---- 底部：左「开始组网 / 组网进行中 / 重新组网」，右「返回」 ---- */
    if (running)
    {
        /* 组网期间 STM32 忽略全部按键，这里也就不做高亮，只作状态提示。
           ★ V1_0_11：文案避开了字库里**没有**的字（见下）。
           原来写的是「组网进行中」与「重新组网」，而 11/14/16/18 四套字模里
           **"进"、"行"、"新" 三个字都不存在**（每套 237 个字形、139 个汉字），
           屏上会走到 '?' 兜底 —— 现场看到的就是"进行/新"这几个字不对。
           现有字形里 正/在/组/网/重/测 都有，因此改为：
               组网进行中 -> 正在组网
               重新组网   -> 重测组网
           要恢复原词，必须先把这三个字的字模补进 ui_menu_assets.h。 */
        ui_menu_control(  6u, 140u, 183u, 22u, "正在组网", 0u, 1u);
        ui_menu_control(195u, 140u, 183u, 22u, "返回",     0u, 1u);
    }
    else
    {
        ui_menu_control(  6u, 140u, 183u, 22u, done ? "重测组网" : "开始组网",
                        ui_menu_selected(0u), 1u);
        ui_menu_control(195u, 140u, 183u, 22u, "返回", ui_menu_selected(1u), 1u);
    }
}

void networking_Control(void)
{
   /* ★ V1_0_11（现场要求）：从左边菜单进到右边时，**先显示原来的选择页**
      （「开始组网 / 返回」），不要一进来就摊开结果表；
      只有按下「开始组网」、真正开测之后（net_flag 1~3 / 4），才切到整页结果表。
      所以判据是**状态**而不是 menu_rank：
        net_flag == 0（空闲）或还没进入子页 -> 原来的紧凑选择页
        其它（进行中 / 已完成）             -> 整页结果表
   */
   if (Data_list1.menu_rank != 3u || Data_list1.Menu_rank2.net_flag == NET_FLAG_IDLE)
   {
       networking_Control1();
       return;
   }
   networking_Result_Page();
}

void calibration_pass_Control(void)
{
   if( Data_list1.Menu_rank3.pass_flag)
   {

       password_Control(); //密码

   }
   else
   {
       calibration_Control();  //标定值
   }
}
void calibration_Control()
{
    uint8_t menu_rank=0;
         uint8_t rank3_addr=0;
          menu_rank = Data_list1.menu_rank;
          rank3_addr = Data_list1.rank3_addr;
       OptionItem laber[10]=
       {
             {120, 90, 24,0,4,4},
             {120, 120, 24,0,4,4},
             {120, 156, 24,0,4,4},
             {250, 156, 24,0,4,4},
  //           {120, 156, 24,0,4,4},
  //           {250, 156, 24,0,4,4}
       };

     ui_draw(120, 60, "标定点:");          // 框y=72,高26,垂直居中约90
     ui_draw(120, 90,  "0   mm");
     laber[0].label_wight=ui_text_width((const char*)"0   mm", 14u);

     if(Data_list1.Menu_rank3.biaoding_flag[0]==1)//动态数据
     {
         char tmp1[20];
         sprintf(tmp1, "%d", Data_list1.Menu_rank3.biaoding_ad[0][0]);
         ui_draw(200, 90, tmp1);

         char tmp2[20];
         sprintf(tmp2, "%d", Data_list1.Menu_rank3.biaoding_ad[0][1]);
         ui_draw(280, 90, tmp2);

         ui_draw(350, 90,  "OK");
     }

     ui_draw(120, 120, "200 mm");         // 下框y=104,高26,垂直居中约122
     laber[1].label_wight=ui_text_width((const char*)"200 mm", 14u);

     if(Data_list1.Menu_rank3.biaoding_flag[1]==1) //动态数据
     {

          char tmp3[20];
          sprintf(tmp3, "%d", Data_list1.Menu_rank3.biaoding_ad[1][0]);
          ui_draw(200, 120, tmp3);

          char tmp4[20];
          sprintf(tmp4, "%d", Data_list1.Menu_rank3.biaoding_ad[1][1]);
          ui_draw(280, 120, tmp4);



          ui_draw(350, 120,  "OK");
     }


     ui_draw(120, 156, "保存并重启");         // 下框y=104,高26,垂直居中约122
     laber[2].label_wight=ui_text_width((const char*)"保存并重启", 14u);

     ui_draw(250, 156, "不保存返回");
     laber[3].label_wight=ui_text_width((const char*)"不保存返回", 14u);

     if (menu_rank == 3)   // 确保索引有效
     {
         if(rank3_addr<4)
         {
            // u8g2_SetFont(&u8g2, UI_FONT_CN);
             // 获取菜单名称的像素宽度
             uint16_t name_width =laber[rank3_addr].label_wight;
                               // 定义框的边距（与文字保持间距）
                               uint16_t padding = laber[rank3_addr].kuang_wight;
                               // 计算框的坐标（根据字体基线调整）
                               // u8g2 的 y 坐标是基线，假设字体高度约为 16，ascent 约 14，descent 约 2
                               // 框的左上角 y = posy - 14 - 2（适当微调）
                               uint16_t x = laber[rank3_addr].posx - padding;
                               uint16_t y = laber[rank3_addr].posy -laber[rank3_addr].font_size ;  // 根据实际字体调整
                               uint16_t w = name_width + padding * 2;
                               uint16_t h =laber[rank3_addr].font_size+ laber[rank3_addr].kuang_high*2;  // 框高度略大于字体高度
                               // 画框（仅边框，不填充）
                               u8g2_DrawFrame(&u8g2, x, y, w, h);
         }
     }
}

void password_Control()
{
    uint8_t menu_rank=0;
         uint8_t rank3_addr=0;
          menu_rank = Data_list1.menu_rank;
          rank3_addr = Data_list1.rank3_addr;
    OptionItem laber[10]=
     {
           {162, 108, 24,0,8,8},
           {208, 108, 24,0,8,8},
           {254, 108, 24,0,8,8},
           {300, 108, 24,0,8,8},
//           {120, 156, 24,0,4,4},
//           {250, 156, 24,0,4,4}
     };

     ui_draw(160, 80, "请输入标定密码");          // 框y=72,高26,垂直居中约90


     u8g2_DrawRFrame(&u8g2, 160, 90, 28, 28, 4);
     u8g2_DrawRFrame(&u8g2, 206, 90, 28, 28, 4);
     u8g2_DrawRFrame(&u8g2, 252, 90, 28, 28, 4);
     u8g2_DrawRFrame(&u8g2, 298, 90, 28, 28, 4);

      char tmp1[20];
      sprintf(tmp1, "%d", Data_list1.Menu_rank3.pass_buf[0]%10);
      ui_draw(167, 112, tmp1);
      laber[0].label_wight=24;

      char tmp2[20];
      sprintf(tmp2, "%d", Data_list1.Menu_rank3.pass_buf[1]%10);
      ui_draw(213, 112, tmp2);
      laber[1].label_wight=24;

      char tmp3[20];
      sprintf(tmp3, "%d", Data_list1.Menu_rank3.pass_buf[2]%10);
      ui_draw(259, 112, tmp3);
      laber[2].label_wight=24;

      char tmp4[20];
      sprintf(tmp4, "%d", Data_list1.Menu_rank3.pass_buf[3]%10);
      ui_draw(305, 112, tmp4);
      laber[3].label_wight=24;

     if (menu_rank == 3)   // 确保索引有效
     {
         if(rank3_addr<4)
         {
            // u8g2_SetFont(&u8g2, UI_FONT_CN);
             // 获取菜单名称的像素宽度
             uint16_t name_width =laber[rank3_addr].label_wight;
                               // 定义框的边距（与文字保持间距）
               uint16_t padding = laber[rank3_addr].kuang_wight;
               // 计算框的坐标（根据字体基线调整）
               // u8g2 的 y 坐标是基线，假设字体高度约为 16，ascent 约 14，descent 约 2
               // 框的左上角 y = posy - 14 - 2（适当微调）
               uint16_t x = laber[rank3_addr].posx - padding;
               uint16_t y = laber[rank3_addr].posy -laber[rank3_addr].font_size ;  // 根据实际字体调整
               uint16_t w = name_width + padding * 2;
               uint16_t h =laber[rank3_addr].font_size+ laber[rank3_addr].kuang_high*2;  // 框高度略大于字体高度
               // 画框（仅边框，不填充）
               u8g2_DrawFrame(&u8g2, x, y, w, h);
         }
     }
}
void new_return(void)
{

    ui_draw(80, 80, "正在重启保存数据");
}
/* Installation: re_flag 3 = local RSSI, 4 = local voltage.
 * Show all channels, including when the controller still sends an old page code. */
static void install_ch_page(uint8_t kind)
{
    char buf[32];
    uint8_t ch;
    uint16_t cx, y;
    device_t *d;

    ui_param_frame();
    for (ch = 0u; ch < MAX_CH_NUM; ch++)
    {
        cx = (uint16_t)((ch / PARAM_ROWS) ? PARAM_CELL_R_X : PARAM_CELL_L_X);
        y = (uint16_t)(PARAM_GRID_TOP + 11u + PARAM_ROW_H * (ch % PARAM_ROWS));
        d = &CH_com_buf[ch];
        ui_param_number(cx, y, (uint8_t)(ch + 1u));
        if (!(d->valid && d->data_re_flag)) strcpy(buf, "--");
        else if (kind == 3u) sprintf(buf, "%d dBm", (int)(int8_t)d->rssi);
        else sprintf(buf, "%d.%d V", d->voltage / 10, d->voltage % 10);
        ui_param_value(cx, y, buf);
    }
}

void install_Control(void)
{
    uint8_t re = Data_list1.UI_main.re_flag;
    if (re == 3) { install_ch_page(3); return; }
    if (re == 4) { install_ch_page(4); return; }
    ui_menu_control(117u, 62u, 258u, 27u, "设备信号", ui_menu_selected(0u), 0u);
    ui_menu_control(117u, 99u, 258u, 27u, "设备电压", ui_menu_selected(1u), 0u);
    ui_menu_button(1u, "返回", ui_menu_selected(2u));
}
void uploading_Control(void)
{
    uint8_t i;
    char label[16], old_value[20], new_value[20];
    ui_menu_text(190u, 61u, "已存地址", 16u);
    ui_menu_text(290u, 61u, "更改为", 16u);
    for (i = 0u; i < 3u; i++)
    {
        uint16_t y = (uint16_t)(65u + 23u * i);
        uint16_t old = Data_list1.Menu_rank5.old_send_addr[i];
        uint16_t next = Data_list1.Menu_rank5.new_send_addr[i];
        sprintf(label, "上传%d:", i + 1);
        if (old == 0u) sprintf(old_value, "中继");
        else if (old == 121u) sprintf(old_value, "无");
        else if (old == 122u) sprintf(old_value, "分站");
        else sprintf(old_value, "%d号", old);
        if (next == 0u) sprintf(new_value, "中继");
        else if (next == 121u) sprintf(new_value, "无");
        else if (next == 122u) sprintf(new_value, "分站");
        else sprintf(new_value, "%d号", next);
        ui_menu_text(122u, (uint16_t)(y + 17u), label, 16u);
        ui_menu_control(189u, y, 86u, 22u, old_value, 0u, 1u);
        ui_menu_control(279u, y, 96u, 22u, new_value, ui_menu_selected(i), 1u);
    }
    ui_menu_button(0u, "保存并返回", ui_menu_selected(3u));
    ui_menu_button(1u, "不保存返回", ui_menu_selected(4u));
}
void Other_Settings_Control(void)
{
    char value[20];
    sprintf(value, "%d db", Data_list1.Menu_rank6.power);
    ui_menu_field(49u, "通信功率:", value, ui_menu_selected(0u), ui_menu_selected(1u));
    sprintf(value, "%d 秒", Data_list1.Menu_rank6.time_light);
    ui_menu_field(78u, "亮屏时间:", value, ui_menu_selected(2u), ui_menu_selected(3u));
    ui_menu_field(107u, "通信状态:", Data_list1.Menu_rank6.state == 1 ? "开机" : "不开机",
                  ui_menu_selected(4u), ui_menu_selected(5u));
    ui_fast_DrawXBMP(&u8g2, 326u, 68u, 46u, 58u, ui_menu_detail_other);
    ui_menu_button(0u, "保存并返回", ui_menu_selected(6u));
    ui_menu_button(1u, "不保存返回", ui_menu_selected(7u));
}

void zero_setting_Control() {

    uint8_t menu_rank=0;
         uint8_t rank3_addr=0;
          menu_rank = Data_list1.menu_rank;
          rank3_addr = Data_list1.rank3_addr;
    OptionItem laber[10]=
     {
           {120, 120, 16,0,4,4},
           {250, 120, 16,0,4,4},
           {120, 156, 24,0,4,4},
           {250, 156, 24,0,4,4},
//           {120, 156, 24,0,4,4},
//           {250, 156, 24,0,4,4}
     };
       ui_draw(120, 60, "通道一:");          // 框y=72,高26,垂直居中约90
      // ui_draw(250, 60, "1200 mm");

       char tmp1[20];
       sprintf(tmp1, "%d mm", Data_list1.Menu_rank7.len_value[0]/10);
       ui_draw(220, 60, tmp1);

       char tmp2[20];
       sprintf(tmp2, "%d", Data_list1.Menu_rank7.ad_value[0]);
       ui_draw(320, 60, tmp2);
        //  laber[3].label_wight=ui_text_width((const char*)tmp2, 14u);

       ui_draw(120, 80, "通道二:");         // 下框y=104,高26,垂直居中约122
       char tmp3[20];
       sprintf(tmp3, "%d mm", Data_list1.Menu_rank7.len_value[1]/10);
       ui_draw(220, 80, tmp3);

       char tmp4[20];
       sprintf(tmp4, "%d", Data_list1.Menu_rank7.ad_value[1]);
       ui_draw(320, 80, tmp4);
          //   laber[3].label_wight=ui_text_width((const char*)tmp2, 14u);



     //  ui_draw(250, 80, "1200 mm");

       ui_draw(120, 100, "已保存置零点为:");         // 下框y=104,高26,垂直居中约122
      // ui_draw(250, 100, "7.8 mm");
       char tmp5[20];
       sprintf(tmp5, "%d.%d mm", Data_list1.Menu_rank7.old_len/10,Data_list1.Menu_rank7.old_len%10);
       ui_draw(250, 100, tmp5);

       ui_draw(120, 120, "置零点为:");         // 下框y=104,高26,垂直居中约122
       laber[0].label_wight=ui_text_width((const char*)"置零点为:", 14u);
       //ui_draw(250, 120, "6.6 mm");

       char tmp6[20];
       sprintf(tmp6, "%d.%d mm", Data_list1.Menu_rank7.new_len/10,Data_list1.Menu_rank7.new_len%10);
       ui_draw(250, 120, tmp6);
       laber[1].label_wight=ui_text_width((const char*)tmp6, 14u);

       ui_draw(120, 156, "保存并重启");         // 下框y=104,高26,垂直居中约122
       laber[2].label_wight=ui_text_width((const char*)"保存并重启", 14u);

       ui_draw(250, 156, "不保存返回");
       laber[3].label_wight=ui_text_width((const char*)"不保存返回", 14u);

      if (menu_rank == 3)   // 确保索引有效
      {
          if(rank3_addr<4)
          {
             // u8g2_SetFont(&u8g2, UI_FONT_CN);
              // 获取菜单名称的像素宽度
              uint16_t name_width =laber[rank3_addr].label_wight;
                                // 定义框的边距（与文字保持间距）
                                uint16_t padding = laber[rank3_addr].kuang_wight;
                                // 计算框的坐标（根据字体基线调整）
                                // u8g2 的 y 坐标是基线，假设字体高度约为 16，ascent 约 14，descent 约 2
                                // 框的左上角 y = posy - 14 - 2（适当微调）
                                uint16_t x = laber[rank3_addr].posx - padding;
                                uint16_t y = laber[rank3_addr].posy -laber[rank3_addr].font_size ;  // 根据实际字体调整
                                uint16_t w = name_width + padding * 2;
                                uint16_t h =laber[rank3_addr].font_size+ laber[rank3_addr].kuang_high*2;  // 框高度略大于字体高度
                                // 画框（仅边框，不填充）
                                u8g2_DrawFrame(&u8g2, x, y, w, h);
          }
      }
}


/* 菜单第 7 项「返回主页」（rank2_addr == 7，契约 §1 第 43 行 / §6）
 *
 * ★ CH584M 是纯渲染端，**不做任何本地页面跳转**：menu_rank / rank2_addr /
 *   rank3_addr 只能由 STM32 的 0x01 帧写入（Usart3_task.c:479-483），
 *   本函数**绝不**修改这些字段，只画对应的提示。
 *   实际"返回主页"由 STM32 按键（K2 确认）后下发 menu_rank=1 的 0x01 帧完成。 */
void return_main_Control(void)
{
    char version[16];
    const char *slogan = "精确 稳定 可靠";
    uint16_t version_x, slogan_width, slogan_x;
    /* Static reference artwork only; the STM32 still owns the return action. */
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_fast_DrawBox(&u8g2, 113u, 1u, 266u, 166u);
    u8g2_SetDrawColor(&u8g2, 0u);
    ui_fast_DrawXBMP(&u8g2, 113u, 1u, 266u, 166u, ui_menu_return_page);
    /* The reference bitmap carries the same full-size title at both levels. */
    /* Keep the V1_ prefix; controller tenths supply the final two segments. */
    sprintf(version, "V1_%u_%u", (unsigned int)(Data_list1.UI_main.version / 10u),
            (unsigned int)(Data_list1.UI_main.version % 10u));
    version_x = (uint16_t)(368u - ui_text_width(version, 18u));
    slogan_width = UI_RETURN_SLOGAN_WIDTH;
    slogan_x = (uint16_t)(113u + (266u - slogan_width) / 2u);
    u8g2_SetDrawColor(&u8g2, 1u);
    ui_fast_DrawBox(&u8g2, 119u, 134u, 254u, 26u);
    u8g2_SetDrawColor(&u8g2, 0u);
    /* Dedicated binary LiSu artwork uses the same ink color as the version. */
    ui_fast_DrawXBMP(&u8g2, slogan_x, 156u - UI_RETURN_SLOGAN_SIZE,
                  UI_RETURN_SLOGAN_WIDTH, UI_RETURN_SLOGAN_HEIGHT,
                  ui_menu_return_slogan);
#ifdef UI_PREVIEW
    {
        extern void ui_preview_text(uint16_t, uint16_t, const char *, uint8_t, unsigned);
        ui_preview_text(slogan_x, 155u, slogan, UI_RETURN_SLOGAN_SIZE, 0u);
    }
#endif
    ui_text_draw(version_x, 25u, version, 18u);
}








void u8g2Init(u8g2_t *u8g2)
{
    TFT_IO_init();

    u8g2_Setup_st7305_yuying_168x384_f(u8g2, U8G2_R3,u8x8_byte_CH584M_spi, u8x8_CH584M_gpio_and_delay);

    u8g2_InitDisplay(u8g2);
    /* Write a complete requested image before any display-on command. */
    u8g2_SetPowerSave(u8g2, 1u);
    u8g2_ClearBuffer(u8g2);
}

/* Native ST7305 RAM format, not XBMP. Restoring this image needs only a RAM
 * write. A physical S2 reset still runs normal controller initialization. */
void UI_TestArtwork(u8g2_t *u8g2)
{
    u8x8_st7305_yuying_send_boot(u8g2_GetU8x8(u8g2));
}

void u8g2Init1(u8g2_t *u8g2)
{
    TFT_IO_init();

    u8g2_Setup_st7306_300x400_f(u8g2, U8G2_R3,u8x8_byte_CH584M_spi, u8x8_CH584M_gpio_and_delay);

    u8g2_InitDisplay(u8g2);         //̬
    u8g2_SetPowerSave(u8g2, 0);    //

    u8g2_ClearBuffer(u8g2);
    u8g2_SendBuffer(u8g2);
}

__HIGH_CODE
uint8_t u8x8_CH584M_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,U8X8_UNUSED void *arg_ptr)
{
  switch(msg)
  {
   case U8X8_MSG_GPIO_AND_DELAY_INIT:

      break;
    case U8X8_MSG_DELAY_MILLI:                          // delay arg_int * 1 milli second
        DelayMs(arg_int);
        break;
    case U8X8_MSG_DELAY_10MICRO:
        DelayUs(arg_int * 10);
        break;

        case U8X8_MSG_GPIO_CS:
              if(arg_int)                                     // arg_int=1: Input dir with pullup high for I2C clock pin
              {
                  CS_HIGH;
              }
              else
              {
                  CS_LOW;
              }

                break;
        case U8X8_MSG_GPIO_DC:
              if(arg_int)                                     // arg_int=1: Input dir with pullup high for I2C clock pin

              {
                  DC_DATA;
              }
               else
               {
                   DC_CMD;
               }

                break;
        case U8X8_MSG_GPIO_RESET:
                break;


    default:

        break;
  }
  return 1;
}
__HIGH_CODE
uint8_t u8x8_byte_CH584M_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
      SPI0_MasterDMATrans((uint8_t *) arg_ptr, arg_int);
  //  HAL_SPI_Transmit(&hspi1, (uint8_t *) arg_ptr, arg_int, 10000);
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
      if(arg_int)
      {
          DC_DATA;

      }
      else
      {
          DC_CMD;

      }

    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
          u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
          u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
          break;
  case U8X8_MSG_BYTE_END_TRANSFER:
          u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
          u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
          break;
    break;
  default:
    return 0;
  }
  return 1;
}
void TFT_IO_init(void)
{
    GPIOA_SetBits(GPIO_Pin_12);
    GPIOA_ModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    SPI0_MasterDefInit();

    GPIOA_SetBits(GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
    GPIOA_ModeCfg(GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_ModeOut_PP_5mA);

    GPIOB_ResetBits(GPIO_Pin_0|GPIO_Pin_1);
    GPIOB_ModeCfg(GPIO_Pin_0|GPIO_Pin_1, GPIO_ModeOut_PP_5mA);
}
//void UI_Main_Display(void)
//{
//    // -------- 全局设置 --------
//    u8g2_SetDrawColor(&u8g2, 1);          // 前景色（黑色）
//    u8g2_SetFontMode(&u8g2, 1);           // 透明模式（文字不覆盖背景）
//
//    // -------- 1. 主外框（双层） --------
//    u8g2_DrawRFrame(&u8g2, 4, 4, 376, 160, 6);   // 外框
//    u8g2_DrawRFrame(&u8g2, 6, 6, 372, 156, 4);   // 内框（增加层次感）
//
//    u8g2_SetFont(&u8g2, u8g2_font32_lunar);
//    u8g2_DrawLine(&u8g2, 118, 40, 90, 6);                 // 分隔线
//    u8g2_DrawLine(&u8g2, 266, 40, 294, 6);                 // 分隔线
//    u8g2_DrawLine(&u8g2, 118, 40, 266, 40);                 // 分隔线
//
//    u8g2_DrawLine(&u8g2, 114, 42, 86, 10);                 // 分隔线
//    u8g2_DrawLine(&u8g2, 270, 42, 298, 10);                 // 分隔线
//   u8g2_DrawLine(&u8g2, 328, 10, 298, 10);                 // 分隔线
//
//   u8g2_DrawLine(&u8g2, 12, 40, 12, 120);                 // 分隔线
//   u8g2_DrawLine(&u8g2, 12, 40, 30, 30);                 // 分隔线
//   u8g2_DrawLine(&u8g2, 30, 30, 76, 30);                 // 分隔线
//   // -------- 3. 标题（居中） --------
//      u8g2_SetFont(&u8g2, u8g2_font32_lunar);
//      const char* title = "位移系统";
//      uint8_t tw = u8g2_GetUTF8Width(&u8g2, title);
//      u8g2_DrawUTF8(&u8g2, (384 - tw) / 2, 36, title);       // y=60
//   // -------- 2. 顶部状态栏 --------
//       u8g2_SetFont(&u8g2, u8g2_font16_lunar);
//       u8g2_DrawUTF8(&u8g2, 12, 22, "三为矿安");                // 左logo
//
//       u8g2_SetFont(&u8g2, u8g2_font_helvB08_tf);      // 8像素英文
//       u8g2_DrawUTF8(&u8g2, 340, 40, "V1.0.1");        // 左下
//       u8g2_DrawUTF8(&u8g2, 340, 20, "3.3V");         // 右下
//
//       // -------- 5. 数据信息（两行） --------
//       u8g2_DrawRFrame(&u8g2, 30, 56, 246, 28, 3);
//        // 下框：监测点
//        u8g2_DrawRFrame(&u8g2, 30, 92, 246, 28, 3);
//
//       // 填充数据（标签+数值同一行）
//       u8g2_SetFont(&u8g2, u8g2_font24_lunar);
//
//       u8g2_DrawUTF8(&u8g2, 32, 76, "浅基点:");          // 框y=72,高26,垂直居中约90
//       u8g2_DrawUTF8(&u8g2, 120, 76, "1200.0 mm");
//       u8g2_DrawUTF8(&u8g2, 32, 112, "深基点:");         // 下框y=104,高26,垂直居中约122
//             u8g2_DrawUTF8(&u8g2, 120, 112, "1200.0 mm");
//   // -------- 5. 底部状态信息（两行） --------
//  //  u8g2_DrawLine(&u8g2, 12, 134, 372, 134);        // 分割线
//
//    u8g2_DrawLine(&u8g2, 118, 158, 90, 134);                 // 分隔线
//    u8g2_DrawLine(&u8g2, 266, 158, 294, 134);                 // 分隔线
//    u8g2_DrawLine(&u8g2, 118, 158, 266, 158);                 // 分隔线
//
//    u8g2_DrawLine(&u8g2, 90, 134, 12, 134);                 // 分隔线
//    u8g2_DrawLine(&u8g2, 294, 134, 372, 134);                 // 分隔线
//
//    u8g2_SetFont(&u8g2, u8g2_font16_lunar);
//    u8g2_DrawUTF8(&u8g2, 20, 150, "开机");                // 左logo
//     // u8g2_SetFont(&u8g2, u8g2_font16_lunar);
//    u8g2_DrawUTF8(&u8g2, 290, 130, "分站:64号");                // 左logo
//    u8g2_DrawUTF8(&u8g2, 290, 150, "发往:118号");                // 左logo
//
//
//   const char* lb1 = "主机号:";
//   const char* val1 = "120";
//   uint8_t lw1 = u8g2_GetUTF8Width(&u8g2, lb1);
//   uint8_t vw1 = u8g2_GetUTF8Width(&u8g2, val1);
//   int total1 = lw1 + 10 + vw1;                    // 10为间距
//   int start1 = 60 + (266 - total1) / 2;           // 框内居中
//   u8g2_DrawUTF8(&u8g2, start1, 150, lb1);          // 框y=72,高26,垂直居中约90
//   u8g2_DrawUTF8(&u8g2, start1 + lw1 + 10, 150, val1);
//}



//void UI_Menu_Display(void) {
//
//  u8g2_SetFont(&u8g2, u8g2_font16_lunar);
//  for (uint8_t i = 0; i < 8; i++)
//  {
//    u8g2_DrawUTF8(&u8g2,Main_List[i].posx, Main_List[i].posy, Main_List[i].name);
//  }
//
//  u8g2_DrawRFrame(&u8g2, 4, 4, 94, 160, 6);   // 外框
//  u8g2_DrawRFrame(&u8g2, 6, 6, 90, 156, 4);   // 内框（增加层次感）
//
//  u8g2_DrawRFrame(&u8g2, 102, 4, 278, 160, 6);   // 内框（增加层次感）
//
//  u8g2_SetFont(&u8g2, u8g2_font24_lunar);
//  u8g2_DrawUTF8(&u8g2,Menu_List[UI_Select].posx, Menu_List[UI_Select].posy, Menu_List[UI_Select].name);
//
//
//  u8g2_DrawLine(&u8g2, 104, 34, 210, 34);
//  Menu_List[UI_Select].function();
//
//  //------画装饰小条------//
////  u8g2_SetDrawColor(&u8g2,1);
////  ui_fast_DrawBox(&u8g2,120, 0, 6, 2);
////  ui_fast_DrawBox(&u8g2,120, 62, 6, 2);
////  ui_fast_DrawBox(&u8g2,122, 2, 2, 60);
////  u8g2_DrawRBox(&u8g2,120, frame_y + 3, 6, 8, 2);
////   u8g2_SetDrawColor(&u8g2,2);                                                                      //选择反色
////    frame_lenth_trg = u8g2_GetUTF8Width(&u8g2,Main_List[UI_Select].name) + 2;                        //菜单每个条目的长度
////    frame_y_trg = Main_List[UI_Select].posy - POS_Y;                                            //选择框的目标值随着选择数值移动
////    UI_MoveSet(&frame_y, &frame_y_trg, MENU_HEIGHT / 4, MENU_HEIGHT / 4 + 1);                    //移动菜单选中框
////    UI_MoveSet(&frame_lenth, &frame_lenth_trg, frame_lenth_trg / 4, frame_lenth_trg / 4 + 1);  //调整选中框长度
////    u8g2_DrawRBox(&u8g2,1, frame_y, frame_lenth, MENU_HEIGHT, 3);                                     //画空心方框坐标x，y，宽度frame_lenth,高度MENU_HEIGHT
//
//}
//void draw(u8g2_t *u8g2)
//{
//
////                u8g2_DrawStr(&u8g2, 0, 15, "Hello World!");
////                u8g2_DrawCircle(&u8g2, 64, 40, 10, U8G2_DRAW_ALL);
//                // ˢ����Ļ
//
//    u8g2_SetFontMode(u8g2, 1);
//    u8g2_SetFontDirection(u8g2, 0);
//    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
//    u8g2_DrawStr(u8g2, 0, 20, "U");
//
//    u8g2_SetFontDirection(u8g2, 1);
//    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
//    u8g2_DrawStr(u8g2, 21,8,"8");
//
//    u8g2_SetFontDirection(u8g2, 0);
//    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
//    u8g2_DrawStr(u8g2, 51,30,"g");
//    u8g2_DrawStr(u8g2, 67,30,"\xb2");
//
//    u8g2_DrawHLine(u8g2, 2, 35, 47);
//    u8g2_DrawHLine(u8g2, 3, 36, 47);
//    u8g2_DrawVLine(u8g2, 45, 32, 12);
//    u8g2_DrawVLine(u8g2, 46, 33, 12);
//
////    u8g2_SetFont(u8g2, u8g2_font_wqy12_t_chinese3);
////    u8g2_DrawUTF8(u8g2, 200, 0, "��ã����磡");
//    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
//    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
//}
//__HIGH_CODE
//void spi_write_test2(uint16_t data)//ͨ�� FIFO��ʽ ��SPI�ӿڷ��� 16 λ����
//{
//    GPIOA_ResetBits(GPIO_Pin_12);     //Ƭѡ�ź�����
//
//    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;   // �������ݷ���Ϊ���
//    while(!(R8_SPI0_INT_FLAG & RB_SPI_FREE));  // �ȴ�FIFO�е�����ȫ���������
//    R16_SPI0_TOTAL_CNT = 2;    //���� SPI0 �����ݴ������ֽ���Ϊ 2����ʾ��Ҫ���� 2 ���ֽڡ�
//    R8_SPI0_FIFO = data>>8;   //�����ݵĸ��ֽ�д�� SPI0 �� FIFO �Ĵ���
//    R8_SPI0_FIFO = data>>0;   //�����ݵĵ��ֽ�д�� SPI0 �� FIFO �Ĵ���
//    while(!(R8_SPI0_INT_FLAG & RB_SPI_FREE));   // �ȴ�FIFO�е�����ȫ���������
//
//    GPIOA_SetBits(GPIO_Pin_12);   //Ƭѡ�ź�����
//}
//__HIGH_CODE
//void spi_write_test(uint16_t data)
//{
//    GPIOA_ResetBits(GPIO_Pin_12);  //Ƭѡ�ź�����
//
//    SPI0_MasterSendByte(data>>8);  //�� SPI0 �ӿڷ������ݵĸ��ֽڡ�ʹ�� >> ���������������� 8 λ���Ի�ȡ�� 8 λ���ݡ�
//    SPI0_MasterSendByte(data>>0);  //�� SPI0 �ӿڷ������ݵĵ��ֽڡ�ʹ�� >> ���������������� 0 λ���Ի�ȡ�� 8 λ���ݡ�
//
//    GPIOA_SetBits(GPIO_Pin_12);  //Ƭѡ�ź�����
//}

