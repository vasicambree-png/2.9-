/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 观察应用主函数及任务系统初始化
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "HAL.h"
#include "observer.h"
#include "yuying_TFT.h"

#include "Usart3_task.h"
#include "app_drv_fifo.h"
#include "Flash.h"


uint8_t Version=1;//版本号

extern uint8_t app_uart_rx_buffer[512] ;

extern app_drv_fifo_t app_uart_rx_fifo;
u8g2_t u8g2;
/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   主循环
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        app_uart_process();
        TMOS_SystemProcess();
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    /* Capture before library/flash initialization; these registers are RO. */
    const uint8_t reset_cause = SYS_GetLastResetSta();
    const uint8_t reset_enabled = (R8_GLOB_CFG_INFO & RB_CFG_RESET_EN) != 0u;
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    HSECFG_Capacitance(HSECap_18p);

    SetSysClock(SYSCLK_FREQ);

#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)

    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);

#endif

#ifdef DEBUG
    GPIOA_SetBits(GPIO_Pin_9);

    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);

    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);

    UART1_DefInit();
#endif
    DelayMs(50);
    PRINT("%s\n", VER_LIB);
    PRINT("[FW] BLE_POWER_PERSIST_20261028_3\r\n");
    PRINT("[RESET] enabled=%u cause=0x%02X\r\n",
          (unsigned int)reset_enabled, (unsigned int)reset_cause);
    if (!reset_enabled)
        PRINT("[RESET] S2 test unavailable: enable RST input in ISP settings\r\n");

    PRINT("[BOOT] 1  CH58x_BLEInit\r\n");
    CH58x_BLEInit();

    PRINT("[BOOT] 2  HAL_Init\r\n");
    HAL_Init();


    PRINT("[BOOT] 3  u8g2Init\r\n");
    u8g2Init(&u8g2);
    screen_reset_display_init(reset_enabled, reset_cause);

    PRINT("[BOOT] 4  fifo_init\r\n");
    app_drv_fifo_init(&app_uart_rx_fifo, app_uart_rx_buffer, 512);

    PRINT("[BOOT] 5  Usart3_Init\r\n");
    Usart3_Init();

    PRINT("[BOOT] 6  usart_task\r\n");
    usart_task();

    PRINT("[BOOT] 7  GAPRole_ObserverInit\r\n");
    GAPRole_ObserverInit();

    PRINT("[BOOT] 8  Observer_Init\r\n");
    Observer_Init();

    /* ★ 任务 D：开机从 Data-Flash 自动加载已保存的绑定（含通道占用恢复）。
     *
     *   时机：Usart3_Init() 之后（PRINT 可用）、Main_Circulation() 之前。
     *   此时 TMOS 主循环还没跑起来，BLE 广播回调（SACN_DATA）不可能并发访问
     *   绑定表/通道表，因此不需要任何临界区。
     *
     *   流程（实现见 Flash.c 的 binding_store_load()）：
     *     EEPROM_READ → 校验 magic('CHBD') / 版本 / 条数(≤32) / 校验和
     *                 → 逐条校验(类型/通道范围/区间冲突/MAC 非全 0 非全 FF)
     *                 → observer_restore_binding()：
     *                       写 g_binding_list[] + g_binding_count++
     *                       + occupy_channels() 恢复 CH_com_buf[] 通道占用
     *                       + 建立名字快照基线
     *   任何一处校验失败都安全回退为"无绑定"（RAM 已预先清空），
     *   绝不会带着 Flash 里的垃圾数据运行。 */
    PRINT("[BOOT] 9  binding_store_load\r\n");
    binding_store_load();

    /* Before the TMOS loop/first BLE callback: restore without waiting for
     * the controller's first page. load primes the save cache, so applying
     * the restored value does not erase or write Flash. */
    {
        uint8_t saved_state = 0;
        if (power_state_store_load(&saved_state) == 0u)
            observer_set_collection_enabled(saved_state);
    }

    PRINT("[BOOT] 10 Main_Circulation\r\n");
    Main_Circulation();
}

/******************************** endfile @ main ******************************/
