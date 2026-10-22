#include "ui.h"
#include "usart.h"
#include "LoraPro.h"
#include "New_Key.h"
#include "flash.h"
#include "L_B_ADC.h"
#include <string.h>

/* ---- 沿用原有按键逻辑的全局量 ---- */
ALL_UI  UI = {0};
CDisOpt DisOpt = Opt0;
CDisOpt Old_DisOpt = Opt0;

/* ---- 外部数据来源 ---- */
extern UART_HandleTypeDef huart1;
extern uint8_t  New_version;
extern uint16_t Vdd_value;
extern Flash_Set Setup;
extern Flash_Set backups_Setup;
extern Channel_C Ch_senser[21];
extern uint8_t   dis_Rssi[4];
extern CRfNet     RfNet;
extern CRfRepeater *TestRepeater;

/* 最大帧 = 4(帧头) + 6(固定) + 30*2(参数) + 2(校验和+帧尾) = 72 */
 uint8_t  ui_tx_buf[160];
static data_LIST ui_data;

/* ★ V1_0_7：本帧发送是否失败（超时 / 串口时钟停振等）。
   为什么需要：LoRa 收发的 STOP 期间 USART1 的时钟会停（工程自己在
   Sleep.c 的 STOP 说明里写过），此时 HAL_UART_Transmit 会超时返回 ——
   原来这里**忽略返回值**，而 UI_display() 已经把"脏标志"清掉了，
   于是这一帧**永久丢失、屏幕不刷新**：现场表现就是"按了键但页面没动"。
   现在把失败记下来，由 UI_display() 下一拍补发一次。 */
static volatile uint8_t s_ui_tx_fail = 0U;

/* 通过 USART1 发给 CH584M */
static void CH584M_Send_data(uint8_t *buf, uint16_t len)
{
    if (HAL_UART_Transmit(&huart1, buf, len, 100) != HAL_OK)
    {
        s_ui_tx_fail = 1U;      /* 交给 UI_display() 补发，页面刷新绝不静默丢失 */
    }
}

/* 把原始 RSSI 换算成 8~11 档（8 最差，11 最好） */
static uint8_t rssi_to_level(uint8_t rssi)
{
    if (rssi > 110)  return 11;
    if (rssi > 0x60) return 10;
    if (rssi > 0x50) return 9;
    return 8;
}

/*
 * 组出一帧协议帧（AA EE len 01 Type menu_rank rank2 rank3 param_cnt
 *   param_data[大端 u16]... checksum 0A）并发送给 CH584M。
 * 字段顺序与 CH584M 的 APP/Usart3_task.c parse_received_frame() 一致。
 */
uint16_t build_frame_ui(uint8_t *tx_buf, data_LIST *pData, uint8_t ui_data_count)
{
    if (tx_buf == NULL || pData == NULL) return 0;

    uint16_t param_cnt = 0;
    uint16_t param_data[30];

    if (pData->menu_rank == 1)
    {
        param_data[param_cnt++] = pData->UI_main.version;
        param_data[param_cnt++] = pData->UI_main.vbat;
        param_data[param_cnt++] = pData->UI_main.host_num;
        param_data[param_cnt++] = pData->UI_main.send_host_num;
        param_data[param_cnt++] = pData->UI_main.sub_num;
        param_data[param_cnt++] = pData->UI_main.state;
        param_data[param_cnt++] = pData->UI_main.Lora_rssi;
        param_data[param_cnt++] = pData->UI_main.chu_num1;
        param_data[param_cnt++] = pData->UI_main.chu_num2;
        param_data[param_cnt++] = pData->UI_main.re_flag;

        if (ui_data_count > 20) ui_data_count = 20;
        for (uint8_t i = 0; i < ui_data_count; i++)
        {
            param_data[param_cnt++] = pData->UI_main.data[i];
        }
    }
    else if (pData->menu_rank == 2 || pData->menu_rank == 3)
    {
        switch (pData->rank2_addr)
        {
            case 0: /* 地址分区 */
                param_data[param_cnt++] = pData->Menu_rank1.set_host_num;
                param_data[param_cnt++] = pData->Menu_rank1.set_sub_num;
                break;

            case 1: /* 组网测试：★ V1_0_51 新布局 —— 本机地址 + 三个地址的全套结果 */
                param_data[param_cnt++] = pData->Menu_rank2.now_host_addr;  /* [0] 本机地址 */
                param_data[param_cnt++] = pData->Menu_rank2.net_flag;       /* [1] 状态 */
                param_data[param_cnt++] = pData->Menu_rank2.cur_index;      /* [2] 正在测的序号 */
                for (int k = 0; k < 3; k++) param_data[param_cnt++] = pData->Menu_rank2.addr[k]; /* [3..5] */
                for (int k = 0; k < 3; k++) param_data[param_cnt++] = pData->Menu_rank2.cnt[k];  /* [6..8] */
                for (int k = 0; k < 3; k++) param_data[param_cnt++] = pData->Menu_rank2.succ[k]; /* [9..11] */
                break;

            case 2: /* 设备绑定（原来的"标定"）：打包标定地址参数 */
                for (int i = 0; i < 3; i++) {
                    param_data[param_cnt++] = pData->Menu_rank3.biaoding_ad[i][0];
                    param_data[param_cnt++] = pData->Menu_rank3.biaoding_ad[i][1];
                }
                for (int i = 0; i < 4; i++) {
                    param_data[param_cnt++] = pData->Menu_rank3.pass_buf[i];
                }
                param_data[param_cnt++] = pData->Menu_rank3.pass_flag;
                for (int i = 0; i < 3; i++) {
                    param_data[param_cnt++] = pData->Menu_rank3.biaoding_flag[i];
                }
                break;

            case 3: /* 安装调试 / 实时数据 */
                for (int i = 0; i < 20; i++) {
                    param_data[param_cnt++] = pData->Menu_rank4.shishi_buf[i];
                }
                param_data[param_cnt++] = pData->Menu_rank4.kaer_time;
                break;

            case 4: /* 上传设置 */
                for (int i = 0; i < 3; i++) {
                    param_data[param_cnt++] = pData->Menu_rank5.old_send_addr[i];
                }
                for (int i = 0; i < 3; i++) {
                    param_data[param_cnt++] = pData->Menu_rank5.new_send_addr[i];
                }
                break;

            case 5: /* 其他设置 */
                param_data[param_cnt++] = (uint16_t)(int16_t)pData->Menu_rank6.power;
                param_data[param_cnt++] = pData->Menu_rank6.time_light;
                param_data[param_cnt++] = pData->Menu_rank6.state;
                break;

            case 6: /* 数据置零 */
                for (int i = 0; i < 3; i++) {
                    param_data[param_cnt++] = pData->Menu_rank7.len_value[i];
                }
                for (int i = 0; i < 3; i++) {
                    param_data[param_cnt++] = pData->Menu_rank7.ad_value[i];
                }
                param_data[param_cnt++] = pData->Menu_rank7.old_len;
                param_data[param_cnt++] = pData->Menu_rank7.new_len;
                break;

            case 7: /* 返回主页：没有参数 */
                break;

            default:
                return 0;
        }

        /* 冻结契约 §2.3：chu_num2（二级子页索引，§4.2 之后恒 0）和 re_flag
           （二级子页选择）在 0x01 帧里没有自己的字段，所以对 menu_rank 2/3
           它们被追加在参数块的最后两个 u16。
           原有每个参数的位置、含义和取值都保持不变；
           只遍历自己认识的参数的那种接收方也不受影响
           （它必须从参数块末尾读这两个值，见 CH584M 侧 Usart3_task.c
           的 case 2/3）。
           而 menu_rank 1 是把它们放在 UI_main 块里的，
           所以主页帧的布局没有任何改动。 */
        param_data[param_cnt++] = pData->UI_main.chu_num2;
        param_data[param_cnt++] = pData->UI_main.re_flag;
    }
    else
    {
        /* menu_rank 4/5 等：没有参数 */
    }

    /* ---- 组帧 ---- */
    uint16_t idx = 0;
    tx_buf[idx++] = 0xAA;
    tx_buf[idx++] = 0xEE;

    uint16_t total_len = 6 + param_cnt * 2;
    tx_buf[idx++] = (uint8_t)(total_len >> 8);
    tx_buf[idx++] = (uint8_t)(total_len & 0xFF);

    tx_buf[idx++] = 0x01;                       /* 命令字       */
    tx_buf[idx++] = (uint8_t)pData->Type;       /* 传感器类型   */
    tx_buf[idx++] = pData->menu_rank;
    tx_buf[idx++] = pData->rank2_addr;
    tx_buf[idx++] = pData->rank3_addr;
    tx_buf[idx++] = (uint8_t)param_cnt;

    for (uint8_t i = 0; i < param_cnt; i++) {
        tx_buf[idx++] = (uint8_t)(param_data[i] >> 8);
        tx_buf[idx++] = (uint8_t)(param_data[i] & 0xFF);
    }

    uint8_t checksum = 0;
    for (uint16_t i = 4; i < idx; i++) {
        checksum += tx_buf[i];
    }
    tx_buf[idx++] = checksum;
    tx_buf[idx++] = 0x0A;

    CH584M_Send_data(tx_buf, idx);

    return idx;
}

/* 用 STM32 当前状态填 data_LIST，并向 CH584M 发一帧。 */
void UI_send_frame(void)
{
    memset(&ui_data, 0, sizeof(ui_data));

    ui_data.menu_rank  = UI.Setting_flag + 1;   /* 0->1，1->2，2->3 */
    ui_data.rank2_addr = (uint8_t)DisOpt;
    ui_data.rank3_addr = (uint8_t)UI.Addr;
    ui_data.Type       = T_MG;

    switch (ui_data.menu_rank)
    {
        case 1: /* 主页 */
            ui_data.UI_main.version       = New_version;
            ui_data.UI_main.vbat          = Vdd_value;
            ui_data.UI_main.host_num      = Setup.Host_number;
            ui_data.UI_main.send_host_num = Setup.transmit_number[0];
            ui_data.UI_main.sub_num       = Setup.partition;
            ui_data.UI_main.state         = Setup.On_Off_flag; /* 1 = 开机/工作，0 = 关机
                                                                （冻结契约 §6.5，与
                                                                main.c:135/LoraPro.c:226 一致） */
            ui_data.UI_main.Lora_rssi     = rssi_to_level(dis_Rssi[0]);
            /* 冻结契约 §2.3：主页第 1 页 = 通道 1..10，第 2 页 = 11..20 */
            ui_data.UI_main.chu_num1      = (UI.MainPage == 2) ? 2U : 1U;
            ui_data.UI_main.chu_num2      = 0;
            ui_data.UI_main.re_flag       = 0;
            for (uint8_t i = 0; i < 20; i++) 
				    {
                ui_data.UI_main.data[i] = Ch_senser[i].CH_data;
            }
            build_frame_ui(ui_tx_buf, &ui_data, 20);
            break;

        case 2: /* 菜单列表 */
        case 3: /* 子页 */
            /* 冻结契约 §2.3：这两个值挂在参数块末尾
               （由 build_frame_ui 追加）。菜单列表本身永远不会选中二级子页，
               所以它始终发 0/0。 */
            if (ui_data.menu_rank == 3)
            {
                ui_data.UI_main.re_flag  = (uint8_t)UI.Sub2;
            }
            else
            {
                ui_data.UI_main.re_flag  = 0;
            }

            /* 冻结契约（协议_息屏省电数据链与页面体系_冻结.md §4.2）：
               设备信号 / 设备电压 / 绑定名称 这三页在 CH584M 侧已经改成
               「单页全屏 20 条（左 10 右 10）」，不再分两页各 10 条，
               所以 chu_num2 对它们不再表示页码 —— 现在任何层级都发 0
               （字段本身保留，位置不变，CH584M 仍从参数块末尾读它）。
               这三页的 K1/K3 翻页同时作废（见 New_Key.c 的 sub2_nav()），
               退出统一走 K2 的「返回」语义。 */
            ui_data.UI_main.chu_num2 = 0;

            switch (DisOpt)
            {
                case Opt0: /* 地址分区 */
                    ui_data.Menu_rank1.set_host_num = backups_Setup.Host_number;
                    ui_data.Menu_rank1.set_sub_num  = backups_Setup.partition;
                    break;

                case Opt1: /* 组网测试：★ V1_0_7 起**只有两项** */
                    /* rank3_addr 是原样转发的（ui_data.rank3_addr = UI.Addr），
                       所以 CH584M 的高亮框始终跟着本页的子选项顺序。
                       ★ V1_0_7（按键与页面最终版）修正：
                         CH584M 屏端的组网页（yuying_TFT.c 的 networking_Control1()）
                         已经**只画两项**：「开始组网(0) / 返回(1)」，旧的两项
                         「一键解绑(2) / 保存绑定(3)」在屏上已无任何绘制点
                         （其注释明确写着 obsolete STM32 focus IDs 2/3 不得再映射到
                          任何动作）。所以 New_Key.c 里本页的 K1/K3 循环与 K2 确认
                         都收窄成只认 0 和 1 —— 原来 STM32 能走到 2/3 并发出
                         0x04 解绑全部 / 0x05 保存绑定，而屏上没有任何高亮提示，
                         属于"零提示地清空全部绑定 / 写 Flash"的高危行为，已删除。
                       并且只在选择视图里才真正动作，选择视图由 CH584M 依据
                       net_flag = RfNet.Status 决定：0（RFNET_IDLE）= 选择视图，
                       其它值 = 组网信息视图。
                       这里没有改动任何字段或布局：帧格式未变。 */
                    /* ★★★ V1_0_51：一次把**三个测试地址**的全套结果发下去，
                       屏端才能整页排版、一眼看全。
                       旧版只发"当前正在测的那一个"，另外两个地址的结果
                       屏端根本拿不到，所以永远只能看到一行。 */
                    ui_data.Menu_rank2.now_host_addr = Setup.Host_number;
                    ui_data.Menu_rank2.net_flag      = (uint16_t)RfNet.Status;
                    /* ★★★ V1_0_52 修正差一：RfNet.TestIndex 在 CYCLEINDEX 里
                       是**装载完当前地址之后才自增**的，所以正在测的那一个槽位是
                       TestIndex-1（0x0C 回复处理里读 dis_Rssi[TestIndex-1] 也是这个道理）。
                       原来直接发 TestIndex，屏端就把正在测的行高亮到了**下一个**地址上。 */
                    ui_data.Menu_rank2.cur_index     = (uint16_t)((RfNet.TestIndex > 0u) ? (RfNet.TestIndex - 1u) : 0u);
                    {
                        uint8_t k;
                        for (k = 0u; k < 3u; k++)
                        {
                            ui_data.Menu_rank2.addr[k] = Setup.transmit_number[k];
                            ui_data.Menu_rank2.cnt[k]  = RfNet.Repeaters[k].TestCnt;
                            ui_data.Menu_rank2.succ[k] = RfNet.Repeaters[k].SuccessCnt;
                        }
                    }
                    break;

                case Opt2: /* 设备绑定（原来的标定）：本页的所有取值
                              都由 CH584M 本地持有，
                              所以 build_frame_ui 发出的 14 个全零 data_addr3 参数
                              照旧保留 —— 参数个数是 CH584M 校验布局的一部分
                              （param_cnt < 14 -> 整帧被拒） */
                    break;

                case Opt3: /* 安装调试 / 实时数据 */
                    for (uint8_t i = 0; i < 20; i++) {
                        ui_data.Menu_rank4.shishi_buf[i] = Ch_senser[i].CH_data;
                    }
                    ui_data.Menu_rank4.kaer_time = 0;
                    break;

                case Opt4: /* 上传设置 */
                    for (uint8_t i = 0; i < 3; i++) {
                        ui_data.Menu_rank5.old_send_addr[i] = Setup.transmit_number[i];
                        ui_data.Menu_rank5.new_send_addr[i] = backups_Setup.transmit_number[i];
                    }
                    break;

                case Opt5: /* 其他设置 */
                    ui_data.Menu_rank6.power      = backups_Setup.transmitted_power;
                    ui_data.Menu_rank6.time_light = backups_Setup.brightness;
                    ui_data.Menu_rank6.state      = backups_Setup.On_Off_flag;
                    break;

                case Opt6: /* 信息汇总（原来的数据置零）：计数和名字
                              都由 CH584M 自己绘制，所以 build_frame_ui 发出的
                              8 个全零 data_addr7 参数照旧保留
                              （param_cnt < 8 -> CH584M 会拒绝整帧） */
                    break;

                case Opt7: /* 返回主页：没有参数 */
                default:
                    break;
            }
            build_frame_ui(ui_tx_buf, &ui_data, 0);
            break;

        default:
            break;
    }
}

/* 由软定时器（20ms）调用：UI 状态 / 数据变化时发一帧 */
void UI_display(void)
{
    if (UI.Setting_flag != UI.Old_Setting_flag ||
        DisOpt != Old_DisOpt ||
        UI.Addr != UI.Old_Addr ||
        UI.dis_data_flag)
    {
        UI_send_frame();
#if TEST_KEY_SRC
		/* V1_0_55 诊断：每次给 CH584M 发 0x01 帧时打印导航状态。
		   用来确认【从主页进设置页一律回到最初位置】这条修复
		   （期望：从主页进入后一定是 flag=1 sub2=0 addr=0 dis=0）。 */
		printf("[UI] flag=%u sub2=%u addr=%u dis=%u\r\n",
		       (unsigned)UI.Setting_flag, (unsigned)UI.Sub2,
		       (unsigned)UI.Addr, (unsigned)DisOpt);
#endif
        UI.Old_Setting_flag = UI.Setting_flag;
        Old_DisOpt          = DisOpt;
        UI.Old_Addr         = UI.Addr;
        UI.dis_data_flag    = 0;

        /* ★ V1_0_7：如果刚才那一帧其实没发出去（串口在 STOP 期间停振 / 超时），
           就把脏标志重新立起来 —— 下一拍 UI_display() 会再发一次。
           影子变量保持"已更新"，所以这里不会和其它分支互相打架；
           重发上限就是主循环节拍（20ms 一次），不会堆积。 */
        if (s_ui_tx_fail != 0U)
        {
            s_ui_tx_fail     = 0U;
            UI.dis_data_flag = 1U;
        }
    }
}

/* --------------------------------------------------------------------------
 * UI_force_redraw ：强制下一次 UI_display() 无条件发一帧。
 *   UI_display() 是靠"当前值 != 上次记录的值"来判断要不要发帧的，
 *   把三个影子变量置成不可能相等的 0xFF 即可让判断必定成立。
 *   用在：开机首帧、息屏唤醒后（CH584M 刚上电重启，必须重画一次）。
 * ------------------------------------------------------------------------ */
void UI_force_redraw(void)
{
    UI.Old_Setting_flag = 0xFFU;
    UI.Old_Addr         = (Set_addr)0xFFU;
    Old_DisOpt          = (CDisOpt)0xFFU;
    UI.dis_data_flag    = 1U;
}

/* --------------------------------------------------------------------------
 * Start_UI_display ：开机首帧。
 *   用户把 menu_rank 改成 5（CH584M 侧 parse_received_frame() 的 case 5
 *   会调用 u8g2Init()），用它当"让屏幕重新初始化一次"的触发 —— 这个改动
 *   本身没问题，保留。
 *   这里只补齐原实现漏掉的两件事：
 *   1) 组帧前先把 ui_data 清零。它是 static，不清零会把上一次（或未初始化）
 *      的 Type / rank2_addr / rank3_addr 一起发出去；
 *   2) 调 UI_force_redraw() 同步三位影子变量，保证紧接着的第一帧
 *      UI_display() 一定会把真正的页面帧发出去，而不会因为
 *      "看起来没变化"被吃掉。
 * ------------------------------------------------------------------------ */
void Start_UI_display(void)
{
    memset(&ui_data, 0, sizeof(ui_data));
    ui_data.menu_rank = 5U;
    ui_data.Type      = T_MG;

    build_frame_ui(ui_tx_buf, &ui_data, 0);

    UI_force_redraw();
}

/* --------------------------------------------------------------------------
 * UI_show_save_result ：把"参数保存"的结果让 CH584M 在屏幕上提示一下。
 * --------------------------------------------------------------------------
 * 为什么要单独发一帧：参数是存在 **STM32 自己的 Data-EEPROM** 里的
 * （flash.c 的 Save_data() -> Write_data_to_EEPROM()），保存成功还是失败
 * 只有 STM32 知道；CH584M 那边只是块显示屏，拿不到这个结果。
 * 而用户要的是"保存成功要有明确的页面提示"，所以由这边组一帧提示帧下发。
 *
 * 帧格式与普通显示帧（build_frame_ui 的输出）**完全同构**，只是内容固定：
 *   AA EE | LEN(2B 大端) | 01 | Type | menu_rank(6) | rank2 | rank3 |
 *   param_cnt(1) | msg(2B 大端) | SUM | 0A
 *   LEN = 6 + 2*1 = 8，整帧 8 + 6 = 14 字节
 *   SUM = 从 CMD（下标 4）累加到最后一个数据字节，取低 8 位
 * CH584M 侧对应 APP\Usart3_task.c 的 parse_received_frame() -> case 6：
 * 它只把 msg 画成提示页（UI_MSG_HOLD_SEC 秒后自动返回原页面），
 * **不会**把 menu_rank=6 当成页面状态，所以提示消失后画回的是最新页面。
 *
 * 调用点：flash.c 的 Save_data()（菜单里"保存"确认之后），主循环上下文。
 * ------------------------------------------------------------------------ */
void UI_show_save_result(uint8_t msg_code)
{
    uint8_t  buf[16];
    uint16_t idx = 0U;
    uint8_t  sum = 0U;
    uint16_t i;

    buf[idx++] = 0xAA;                                  /* 帧头 0 */
    buf[idx++] = 0xEE;                                  /* 帧头 1 */
    buf[idx++] = 0x00;                                  /* LEN 高字节 */
    buf[idx++] = 0x08;                                  /* LEN 低字节 = 6 + 2*1 */
    buf[idx++] = 0x01;                                  /* CMD：显示帧 */
    buf[idx++] = (uint8_t)T_MG;                         /* Type：与普通页面帧一致 */
    buf[idx++] = 0x06;                                  /* menu_rank = 6：提示页 */
    buf[idx++] = 0x00;                                  /* rank2_addr（提示页不用） */
    buf[idx++] = 0x00;                                  /* rank3_addr（提示页不用） */
    buf[idx++] = 0x01;                                  /* param_cnt = 1 */
    buf[idx++] = 0x00;                                  /* msg 高字节 */
    buf[idx++] = (uint8_t)(msg_code & 0xFFU);           /* msg 低字节 = UI_MSG_xxx */

    for (i = 4U; i < idx; i++)
    {
        sum = (uint8_t)(sum + buf[i]);                  /* SUM 从 CMD 起累加 */
    }
    buf[idx++] = sum;
    buf[idx++] = 0x0A;                                  /* 帧尾 */

    CH584M_Send_data(buf, idx);                          /* 阻塞发送，14 字节 ≈ 1.2ms */
}

/* 由软定时器（100ms）调用：周期性置数据刷新标志（1 秒一次） */
void dis_data_task(void)
{
    static uint16_t cnt = 0;
    if (++cnt >= 10)
    {
        cnt = 0;
        UI.dis_data_flag = 1;
    }
}
