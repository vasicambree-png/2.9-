/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* V1_0_60：ALL_init 步骤标记 —— 现场日志里能直接看出卡在哪一步（0 可关掉） */
#define INIT_TRACE 1
#if INIT_TRACE
  #define INIT_MARK(n)   printf("[INIT] %d\r\n", (int)(n))
#else
  #define INIT_MARK(n)   do { } while (0)
#endif
#if TEST_PARSER
extern uint8_t usart_parser_selftest_step(void);
void usart_parser_dump(void);
#endif
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "soft_timer.h"
#include "New_Key.h"
#include "Sx126x_Driver.h"
#include "LoraPro.h"
#include "Time_out.h"
#include "ui.h"
#include "flash.h"
#include "sleep.h"
#include "iwdg.h"
#include "adc.h"
#include "L_B_ADC.h"
#include "lptim.h"

uint8_t New_version = 3; // version V1_0_%d
UART_WakeUpTypeDef WakeUpSelection={0};

extern uint32_t Rt_tick;
void SystemClock_Config(void);
extern ALL_UI UI;
extern Flash_Set Setup;
extern uint8_t sleep_mode;
extern uint8_t  Lora_Rx_state;
extern CRfNet RfNet;
extern float Light_value;
extern uint16_t Vdd_value;
extern CDisOpt DisOpt;
extern CDisOpt Old_DisOpt;

volatile uint8_t  LCD_TX_state=0 ;
uint16_t Time_data_up=0;
uint16_t dis_cnt_setting=0;
uint8_t dis_data_flag=0;

uint8_t time_scan=0;
uint16_t sleep_cnt=0;
uint8_t Frist_start_flag=0;
uint16_t VREFCAL = 0;  // VREFINT cal value (VDDA=3V)

volatile uint8_t reset_flag = 0;
extern  uint8_t  s_rx_byte;
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  SystemPower_Config();

  MX_USART2_UART_Init(); //printf

 // HAL_Delay(100);

  CheckResetReason();

  MX_GPIO_Init();

  HAL_Delay(UI_BOOT_WAIT_MS);

  MX_DMA_Init();

  MX_USART1_UART_Init(); // CH584M display (was serial screen)

  sensor_rx_start(); // cmd 0x03：启动 1 字节中断接收

  Start_read_EEPROM();

  Frist_start_flag=0;

  System_check_Init(); //监测当时的灯光状态

  MX_IWDG_Init(); // IWDG

  re_iwdg();

  MX_TIM2_Init(); // 10ms timer


  key_touch_mute(300U);

  HAL_TIM_Base_Start_IT(&htim2);

  soft_timer_init();

  Lptim_Init(); // 100ms timer

  re_iwdg();

  Lora_Gpio_Init();

  if(SX126x_Lora_init() == NORMAL)
  {
      SX126X_WOR_Execute(Sx1268_Sleep);
      printf("Lora_sucessful into sleep\r\n");
  }

  MX_ADC_Init();     // ADC

  VREFCAL = *(__IO uint16_t *)(0x1FF80078);

  get_vdd_adc_value_sleep();

  UI.lcd_open=1;

  UI.dis_data_flag=1;


  UI.Setting_flag = 0u;
  UI.Sub2         = Sub2_None;
  UI.Addr         = Set_Addr1;
  DisOpt          = Opt0;


  soft_timer_create(Key_task ,10,   SOFT_TIMER_CB_MAIN);

  soft_timer_create(key_tick_isr, 10, SOFT_TIMER_CB_IRQ);
  soft_timer_create(Time_1s   ,1000, SOFT_TIMER_CB_MAIN);


  Start_UI_display();

  dis_data_flag=1;

  printf("main  while\r\n");



  while (1)
  {

    if (power_off_service() != 0u)
    {
      re_iwdg();   /* 关机/提示页态：本拍跳过全部业务，但必须喂狗 */
      continue;
    }

    sensor_rx_task();    // cmd 0x03：分帧 + 校验 + 填 Ch_senser[0..19]


    if(UI.lcd_open==1)
    {
		  	UI_display();
        soft_timer_loop_async();
    }

    if(Setup.On_Off_flag==1)
    {

        Time_check_task();


        Lora_task();     // lora
    }


    sleep_check();


    if (key_k1k3_hold_2s() != 0u)
    {
      power_off_enter();
      continue;
    }


    re_iwdg();       // feed dog
  }
}

void Time_1s(void)
{

  static uint8_t vdd_sec = 0;
  if (++vdd_sec >= 10U)
  {
    vdd_sec = 0;
    get_vdd_adc_value_sleep();
  }



  dis_cnt_setting++;
  if(dis_cnt_setting>3)
  {
    UI.dis_data_flag=1;
    dis_cnt_setting=0;
  }


  {
      static uint8_t s_prev_setting_flag = 0U;
  
      if ((s_prev_setting_flag == 0U) && (UI.Setting_flag != 0U))
      {
          sleep_cnt = 0U;
      }
      if ((s_prev_setting_flag != 0U) && (UI.Setting_flag == 0U))
      {
          sleep_cnt = 0U;
      }
      s_prev_setting_flag = UI.Setting_flag;
  }

  sleep_cnt++;

  if(UI.Setting_flag==0)
  {
      if(sleep_cnt>Setup.brightness)
      {
        sleep_cnt=0;
#if !TEST_KEEP_AWAKE
        sleep_mode=1;
#endif
        UI.Addr=Set_Addr1;
        UI.Setting_flag=0;
      }
  }
  else
  {
      if(sleep_cnt>240)
      {
        UI.Addr=Set_Addr1;
        UI.Setting_flag=0;
		/* ---- V1_0_55：息屏前把整份 UI 导航状态清干净，别把【当时那一页】留到下次进来 ----
		   原来这里只清 Setting_flag / UI.Addr，UI.Sub2 与 DisOpt 会原样留下，于是
		   【设置页静置息屏 -> 唤醒回主页 -> 再从主页进设置】会直接停在上次那一页。
		   （New_Key.c 的【进入设置】处也做了同样的归位，这里是第二道保险。） */
		UI.Sub2 = Sub2_None;
		DisOpt  = Opt0;
        sleep_cnt=0;
#if !TEST_KEEP_AWAKE
        sleep_mode=1;
#endif
      }
  }

  /* 冻结契约（协议_息屏省电数据链与页面体系_冻结.md §3.1）：
     常态（未息屏）**不再**每秒发 0x03 轮询：数据改由 CH584M 每轮采集完成后
     主动上报（cmd 0x06），由 sensor_rx_task() 走与 0x03 应答完全相同的那条
     代码路径校验并写入 Ch_senser[0..19]。
     0x03 现在只剩一个触发点：息屏期间由 usart.c 的 sleep_read_service()
     在"上电->读->断电"窗口里发一次（那里仍然走在途守卫）。
     所以这里只保留"错误计数有变化才打印"的诊断上报。 */
  sensor_rx_stats_report();
}

void ALL_init(void)
{
  HAL_Init();

  SystemClock_Config();

  SystemPower_Config();

  HAL_Delay(100);

  /* V1_0_60：IWDG 启动挪到最前面。原来它在"读完 EEPROM"之后才启动，一旦前面
     任何一步慢过 2.07s，就成了"打完 BOR 就静默复位"——现场日志 3 次全是这个形状：
     `BOR level is 255` x2 之后再没有任何打印，1.97s 后 Reset Cause: IWDG。
     提前启动 + 下面每个长步骤后补 re_iwdg()，任何单步都不会把整机拖进看门狗复位。 */
  MX_IWDG_Init();
  re_iwdg();

//  CheckResetReason();

  Quick_BOR_Test();
  re_iwdg();
  INIT_MARK(1);

  MX_GPIO_Init();
  re_iwdg();
  INIT_MARK(2);

  MX_DMA_Init();
  re_iwdg();
  INIT_MARK(3);

  System_check_Init();
  re_iwdg();
  INIT_MARK(4);

  MX_USART1_UART_Init(); // CH584M display
  re_iwdg();
  INIT_MARK(5);

  sensor_rx_start(); // cmd 0x03：重新启动 1 字节中断接收

  /* 唤醒/软开机保留 Setup 中当前路由；冷启动已在 main() 中读取 EEPROM。 */
  re_iwdg();
  INIT_MARK(6);

  Frist_start_flag=0;

  MX_IWDG_Init(); // IWDG

  re_iwdg();

  MX_TIM2_Init(); // 10ms timer

  /* ★★★ V1_0_53：上电 / 开机 / 息屏唤醒都走 ALL_init()，统一在这里开一个 3 秒的
     触摸静默窗口 —— 触摸控制器刚上电时会把 PB14/PC6 拉低约 5 秒，实测必然产生一串
     假按键（页面自己乱跳、甚至误判成长按关机）。这 3 秒内只认机械键，触摸完全忽略。 */
  key_touch_mute(300U);

  HAL_TIM_Base_Start_IT(&htim2);

  soft_timer_init();

  Lptim_Init(); // 100ms
  re_iwdg();
  INIT_MARK(7);

  re_iwdg();

  Lora_Gpio_Init();

  if(SX126x_Lora_init() == NORMAL)
  {
      SX126X_WOR_Execute(Sx1268_Sleep);
      printf("Lora_sucessful into sleep\r\n");
  }
  re_iwdg();
  INIT_MARK(8);

  MX_ADC_Init();
  if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
  }
  VREFCAL = *(__IO uint16_t *)(0x1FF80078);

  get_vdd_adc_value_sleep();
  re_iwdg();
  INIT_MARK(9);
  UI.lcd_open=1;

  UI.dis_data_flag=1;


  UI.Setting_flag = 0u;
  UI.Sub2         = Sub2_None;
  UI.Addr         = Set_Addr1;
  DisOpt          = Opt0;


  soft_timer_create(Key_task  ,10, SOFT_TIMER_CB_MAIN);
  soft_timer_create(key_tick_isr, 10, SOFT_TIMER_CB_IRQ);
  soft_timer_create(Time_1s   ,1000, SOFT_TIMER_CB_MAIN);

  if(reset_flag==2)
  {
    sleep_mode=1;
  }


  UI_force_redraw();
  key_state_reset();   /* ★ V1_0_6：历史名 key_evt_reset（函数已改名，见 New_Key.c） */
  sleep_cnt = 0;

  dis_data_flag=1;
}

/* ==========================================================================
 * SystemClock_Config ：系统时钟 + 外设时钟源配置
 * --------------------------------------------------------------------------
 * ★★★ V1_0_2 修正（第四轮）：三处 HAL 失败**不再**调 Error_Handler()
 * --------------------------------------------------------------------------
 * 为什么必须改：本函数在**每次 STOP 唤醒后**都会被调用
 * （Sleep.c 的 sleep_stop_ms() / sleep_timer() 尾部），息屏期间约 10 次/秒。
 * 而 Error_Handler() 是 `__disable_irq(); while(1){}` —— 只等看门狗复位。
 * 也就是说：一次 HAL 配置失败 -> **"每 2 秒复位一次"的死循环**，
 * 而且 __disable_irq() 会让串口日志也出不来，现场完全无从定位。
 * 时钟配置失败**不该**让整机反复重启：
 *   · 这一步本来的目的只是"从 STOP 唤醒后把 HSI 和外设时钟恢复到已知状态"；
 *   · 失败时最坏是时序变慢（LPTIM/IWDG 的周期不变），但主循环照常跑、
 *     按键/光检测/LoRa 照常工作、看门狗照常被喂；
 *   · 而且真的失败时，硬件上通常也撑不住，反复复位反而把唯一一次
 *     可诊断的机会（串口日志）也毁掉了。
 * 所以统一改成"打印 + 继续"。这三行日志如果出现，说明时钟树有问题，
 * 应该作为现场排查的第一手线索。
 * ========================================================================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|
                       RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    printf("clk osc fail\r\n");       /* ★ 不致命：见上面的说明 */
    return;                           /* 后面两步都依赖振荡器，直接收工 */
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    printf("clk sys fail\r\n");       /* ★ 不致命 */
    return;
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    //printf("clk periph fail\r\n");    /* ★ 不致命：外设时钟源保持复位默认值 */
  }
}

void CheckResetReason(void)
{

    static uint8_t s_reset_reason_reported = 0U;

    if (s_reset_reason_reported != 0U)
    {
        return;                     /* 本次上电已经报过，不再重复(也不清标志以外的动作) */
    }
    s_reset_reason_reported = 1U;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) {
        reset_flag=1;
        printf("Reset Cause: Power-On Reset (POR/PDR)\n");
    }

    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        reset_flag=2;
        printf("Reset Cause: Independent Watchdog Reset (IWDG)\n");
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) {
        reset_flag=3;
        printf("Reset Cause: Window Watchdog Reset (WWDG)\n");
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        reset_flag=4;
        printf("Reset Cause: Software Reset (SW)\n");
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        reset_flag=5;
        printf("Reset Cause: External Pin Reset (NRST)\n");
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) {
        reset_flag=6;
        printf("Reset Cause: Low Power Reset (LPWR)\n");
    }
    else
		{

        reset_flag=7;
        printf("Reset Cause: Unknown (RCC_CSR 无任何复位标志) -- 疑似深度掉电/欠压! 查电源(共用VCC?) RCC_CSR=%08lX\r\n",
               (unsigned long)RCC->CSR);
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();
}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


void assert_failed(uint8_t *file, uint32_t line)
{
}

