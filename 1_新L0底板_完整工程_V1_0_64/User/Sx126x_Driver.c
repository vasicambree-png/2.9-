/**
  ******************************************************************************
  * 文件名 ：   SX126X_Driver.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Aug-2018
  * 文件描述：
  *     该文件为SX126X模块的驱动层，包含对SX126X模块的工作模式控制，FIFO操作，发
  *射功率、频率等设置，以及一些数据处理的公共函数
  *    客户在使用SX126X模块时候需要移植该文件，如果SX126X_HAL.c中各个函数功能都已
  *实现且函数名等都没有改动，那本文件可以不用做更改直接使用，客户在应用层直接调
  *用本文件函数就可以实现各种操作。
*******************************************************************************/
#include "RadioDiag.h"
#include <string.h>
#include "Sx126x_Driver.h"
#include "spi.h"
#include "usart.h"
#include "usart.h"
#include "LoraPro.h"
#include "sleep.h"
#include "iwdg.h"       /* re_iwdg()：驱动里的有界等待必须自己喂狗（见下面的注释） */


/* --------------------------------------------------------------------------
 * ★★★ V1_0_11：驱动层要能"搬运串口"
 * --------------------------------------------------------------------------
 * 本文件的两个"等 DIO1(TxDone)"循环最长要等一整个发射过程：
 *   · 周期自发上报的前导是 6000 符号 @SF5/BW500 = 385.55 ms；
 *   · 组网测试帧前导 2000 符号 = 128.5 ms。
 * 而 Core/Src/usart.c 的 USART1 接收是"1 字节中断 + 128 字节环形缓冲"
 * （SENSOR_FIFO_SIZE=128，见 usart.c:267），115200 波特率下 127 字节
 * 只够存 **约 11 ms** —— 也就是说主循环只要停 11 ms 以上，CH584M 的
 * 0x03/0x06 上报就会整帧丢，现象就是"串口数据不对 / 少了 / 偶发跳数"。
 * 所以这两个循环里必须把 USART1 的分帧器也跑起来（与 LoraPro.c 的
 * CAD/RX 等待循环同一套做法）。
 * 只搬串口、不派发按键：避免"按键回调 -> 发 LoRa -> 递归回本函数"。
 * -------------------------------------------------------------------------- */
extern void sensor_rx_task(void);
extern CRfStatus     RfStatus;

/* ==========================================================================
 * ★★ V1_0_2 修正：SX126x 所有等待的超时上限（**必须放在本文件所有使用点之前**）
 * --------------------------------------------------------------------------
 * 注意：这几个宏原来被我放在文件中间（第 100 行附近），而
 * SX126X_ReadWriteByte() 在第 66 行就用到了 SX126X_SPI_TIMEOUT_MS
 * —— 宏在预处理阶段必须"先定义后使用"，否则编译直接报
 *   #20: identifier "SX126X_SPI_TIMEOUT_MS" is undefined
 * 所以统一提到这里（extern 区之后、所有函数之前）。
 * ========================================================================== */
#define SX126X_BUSY_TIMEOUT_MS      100U   /* 等 BUSY 引脚拉低的上限          */
#define SX126X_STANDBY_TIMEOUT_MS   100U   /* 等 ChipMode 变成 STDBY 的上限   */
#define SX126X_SPI_TIMEOUT_MS        10U   /* 单字节 SPI 传输的上限（1字节@4MHz = 2us） */
#define LORA_TX_WAIT_TIMEOUT_SEC    5U     /* 等 DIO1(TxDone) 的上限；用 Rt_tick
                                              计时（100ms 节拍，STOP 下仍在走）*/

#define VERSION        LSD4RFC_2L722N10     //选择所选用产品型号，例如：
                                            //     LSD4RFC_2L722N10																							         
																																  //     LSD4RFC_2L714N10
																																	 //    L_LRMRL22_97NN4
 extern lora_d Lo_data;
 extern volatile uint32_t Rt_tick;
//static RadioPacketTypes_t PacketType;

float G_BandWidthKHz = 500.0;//本地计算Symbol周期使用
float G_TsXms = 1.024;//1.024ms
u32 G_CadTimeout = 0x20;
RadioLoRaCadSymbols_t G_CadSymbolNum = LORA_CAD_02_SYMBOL;

//static RadioPacketTypes_t PacketType;

S_LoRaConfig G_LoRaConfig = {
    490000000,                  //频率
    LORA_BW_500,                //带宽
    LORA_SF5,                   //因子
    LORA_CR_4_8,                //码率
    10,                         //发射功率
    LORA_PACKET_EXPLICIT,       //头类型  显式
    LORA_CRC_OFF,               //CRC
    LORA_IQ_NORMAL,             //
    12,                         //前导码长
    100,                         //数据包长
};

S_LoRaPara G_LoRaPara;

RadioOperatingModes_t OperatingMode;
/*
  注意，本文件的函数实现了从RF的驱动与MCU的兼容

*/

void Delay_Us(u16 Times)
{
  for(;Times > 0;Times --)
    for(vu8 i = 0;i < 10;i ++);
}

u8 SX126X_ReadWriteByte(u8 Data)
{
	 uint8_t	RxDat = 0U;
	 /* ★★ 看门狗审查补丁（V1_0_2 第三轮）：超时从 1000ms 收紧到 SX126X_SPI_TIMEOUT_MS(10ms)。
	    1 字节在 4MHz SPI 下只要 2us，1000ms 是 50 万倍余量，毫无意义；
	    更糟的是 HAL_SPI_TransmitReceive 的等待循环里**不喂狗**，而
	    SX126xWriteCommand(size=8) 会连着调 9 次 -> SPI1 一旦不响应
	    （时钟被关、外设异常），就是 9 × 1000ms = **9 秒不喂狗**
	    （IWDG 只有约 2.08s）-> 必然复位。10ms 仍有 5000 倍余量，
	    最坏 9 × 10ms = 90ms。 */
   if (HAL_SPI_TransmitReceive(&hspi1, &Data, &RxDat, 1, SX126X_SPI_TIMEOUT_MS) != HAL_OK) { g_radio_spi_fail_count++; }  // 同步传输
   return RxDat;
  //return Spi_WRByte(Data);
}

/* ==========================================================================
 * ★★ V1_0_2 修正：SX126x 的所有"等寄存器/等引脚"必须**有超时 + 喂狗**
 * --------------------------------------------------------------------------
 * 这是"按键不灵敏、甚至一段时间内完全按不动，然后设备自己复位"的一个
 * **直接来源**，和按键代码本身无关：
 *   驱动里的等待循环都是 `while(条件);` 或 `do{...}while(条件 && nTimes);`，
 *   循环里既没有超时、也没有 re_iwdg()。一旦 SX126x 不按预期回应
 *   （模块未上电 / SPI 受扰 / 时钟或引脚配置不符 / 芯片进异常态），
 *   CPU 就停在这些循环里：
 *     · 主循环不再推进 -> Key_task()、UI_display()、Light_check()、
 *       sensor_rx_task() 全都不跑 -> **按键完全没反应**；
 *     · 没人喂狗 -> IWDG（Prescaler 64 / Reload 1200 / LSI≈37kHz ≈ 2.08 s）
 *       到点复位 -> 现场看到"卡一会儿自己重启"；
 *     · 而按键 EXTI 只能把 CPU 从 STOP 里叫醒，**叫不醒一个正在跑 while 的
 *       CPU** —— 这就是为什么"按键不灵敏"和"看门狗复位"总是同时出现。
 *   统一策略：给每处等待加**时间上限**（HAL_GetTick，SysTick 在非 STOP 下
 *   照常计数），循环内 re_iwdg()，超时只累加一个计数变量然后放弃本次操作，
 *   把控制权还给主循环。绝不把整机拖死。
 * ========================================================================== */
volatile uint16_t SX126x_BusyTimeoutCnt    = 0;   /* BUSY 等待超时次数（诊断用） */
volatile uint16_t SX126x_StandbyTimeoutCnt = 0;   /* STDBY 等待超时次数（诊断用） */
volatile uint16_t Lora_TxWaitTimeoutCnt    = 0;   /* TX 等 DIO1 超时次数（诊断用） */

/* CAD configuration is valid only until the next RF hardware reset. */
static uint8_t s_cad_cfg_done = 0U;
volatile RadioDiagIrqCounters g_radio_diag_irq = {0};
volatile uint32_t g_radio_spi_fail_count = 0U;

void SX126xWaitOnBusy( void )
{
  uint32_t t0 = HAL_GetTick();

  while(LORA_BUSY_VAL == 1)
  {
    re_iwdg();                                   /* 等待期间也必须喂狗 */
    if ((uint32_t)(HAL_GetTick() - t0) > SX126X_BUSY_TIMEOUT_MS)
    {
      SX126x_BusyTimeoutCnt++;                   /* 超时：放弃等待，不卡死整机 */
      break;
    }
  }
}

void SX126x_Dly_Us(void)
{
	vu16 i = 2000;
	while(i --);
}

/**
  * @简介：SX126X 唤醒
  * @参数：无
  * @返回值：无
  */
void new_wakeup(void)
{
	 LORA_NSS_L;
	  HAL_Delay(1);
	 LORA_NSS_H;
	HAL_Delay(1);
}
void SX126xWakeup( void )
{
	//new_wakeup();
  LORA_NSS_L;
  //SX126x_Dly_Us();
  SX126X_ReadWriteByte( RADIO_GET_STATUS );
  SX126X_ReadWriteByte( 0x00 );
  LORA_NSS_H;
  // Wait for chip to be ready.
  SX126xWaitOnBusy( );
}
/**
  * @简介：获取当前工作模式
  * @参数：无
  * @返回值：OperatingMode
  */
RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}
/**
  * @简介：检查SX126X是否处于准备状态
  * @参数：无
  * @返回值：无
  */
void SX126xCheckDeviceReady( void )
{
    /* ★★★ V1_0_13：不确定就先"叫醒"，绝不盲等 BUSY
     * ------------------------------------------------------------------
     * 现场故障（用户实测）：LoRa 从 CAD 进 STOP，被定时器立刻叫醒后马上去让
     *   芯片进 Sleep，结果卡在 while 里，最后被看门狗复位。
     * 手册依据（DS_SX1268_V1.1）：
     *   · §8.3.1 / §9.3：**Sleep 模式下 BUSY 引脚被 20k 电阻上拉保持高电平**，
     *     只有 NSS 出现下降沿才能让芯片退出 Sleep（§8.2.2）；
     *   · §9.3：芯片**只能从 STDBY_RC 进入 Sleep**。
     * 于是只要"软件以为芯片是 STDBY、而芯片其实已经睡了"，
     *   SX126xWaitOnBusy() 就会去等一个**永远不会变低**的 BUSY：
     *   老代码（无超时）就是永久死等；加了超时后变成每条命令白等 100ms、
     *   而且命令可能被芯片忽略 —— 芯片留在 RX(约 10mA) 而软件以为它睡了。
     * 修法：把 MODE_UNKNOWN（SetStandby 超时后置）也当作"可能睡着"，
     *   先做一次 SX126xWakeup()（NSS 拉低->读状态->拉高）再等 BUSY。
     *   醒着时这一下无害（读命令不经过内部状态机，§8.3.1），
     *   睡着时正好把它叫醒 —— 死等从根上消失。
     * 注：MODE_RX_DC（占空比接收）时芯片大部分时间在睡，同样必须先唤醒。 */
    RadioOperatingModes_t m = SX126xGetOperatingMode( );
    if( ( m == MODE_SLEEP ) || ( m == MODE_RX_DC ) || ( m == MODE_UNKNOWN ) )
    {
        SX126xWakeup( );
    }
    SX126xWaitOnBusy( );
}
/**
  * @简介：写命令
  * @参数：command，buffer，size
  * @返回值：无
  */
void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );
    LORA_NSS_L;
    SX126X_ReadWriteByte(( uint8_t )command);
    for( uint16_t i = 0; i < size; i++ )
    {
        SX126X_ReadWriteByte(buffer[i]);     
    }
    LORA_NSS_H;
    if( command != RADIO_SET_SLEEP )//0x84指令 配置芯片在 SLEEP 模式
    {
        SX126xWaitOnBusy( );
    }
}
/**
  * @简介：读命令
  * @参数：command，buffer，size
  * @返回值：无
  */
void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{

    SX126xCheckDeviceReady( );
		 // printf("VVVaaaaaaa222\r\n");
    LORA_NSS_L;
    SX126X_ReadWriteByte(( uint8_t )command);
    SX126X_ReadWriteByte(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SX126X_ReadWriteByte(0x00);
    }
    LORA_NSS_H;
//		printf("2222\r\n");
    SX126xWaitOnBusy( );
}
/**
  * @简介：写寄存器组
  * @参数：address，buffer，size
  * @返回值：无
  */
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );
    LORA_NSS_L;
    SX126X_ReadWriteByte(RADIO_WRITE_REGISTER);
    SX126X_ReadWriteByte((address & 0xFF00 ) >> 8);
    SX126X_ReadWriteByte( address & 0x00FF );
    for( uint16_t i = 0; i < size; i++ )
    {
        SX126X_ReadWriteByte( buffer[i] );
    }
    LORA_NSS_H;
    SX126xWaitOnBusy( );
}
/**
  * @简介：写单个寄存器
  * @参数：address，value
  * @返回值：无
  */
void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}
/**
  * @简介：读寄存器组
  * @参数：address，buffer，size
  * @返回值：无
  */
void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    LORA_NSS_L;
    SX126X_ReadWriteByte(RADIO_READ_REGISTER );
    SX126X_ReadWriteByte((address & 0xFF00 ) >> 8);
    SX126X_ReadWriteByte( address & 0x00FF );
    SX126X_ReadWriteByte(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SX126X_ReadWriteByte(0x00);
    }
    LORA_NSS_H;
    SX126xWaitOnBusy( );
}
/**
  * @简介：读单个寄存器
  * @参数：address
  * @返回值：data
  */
uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}
/**
  * @简介：写Buffer
  * @参数：offset,buffer,size
  * @返回值：无
  */
void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
    LORA_NSS_L;
    SX126X_ReadWriteByte( RADIO_WRITE_BUFFER );//主机下发的数据
    SX126X_ReadWriteByte( offset ); //传入偏移值
    for( uint16_t i = 0; i < size; i++ )
    {
        SX126X_ReadWriteByte( buffer[i] );
    }
    LORA_NSS_H;
    SX126xWaitOnBusy( );
}
/**
  * @简介：读Buffer
  * @参数：offset,buffer,size
  * @返回值：无
  */
void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
    LORA_NSS_L;
    SX126X_ReadWriteByte( RADIO_READ_BUFFER );
    SX126X_ReadWriteByte( offset );
    SX126X_ReadWriteByte(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SX126X_ReadWriteByte(0x00);
    }
    LORA_NSS_H;
    SX126xWaitOnBusy( );
}
void SX126xWriteFifo( uint8_t *buffer, uint8_t size )
{
    SX126xWriteBuffer( 0, buffer, size );
}

void SX126xReadFifo( uint8_t *buffer, uint8_t size )
{
    SX126xReadBuffer( 0, buffer, size );
}


//-------------------------SX126X 基本设置-----------------------//
//该部分函数为对SX126X模块复位、发射功率、发射频率等参数设置，以
//及SX126X工作模式设置、数据包读写等
//--------------------------------------------------------------//

/**
  * @简介：SX126X  复位设置
  * @参数：无
  * @返回值：无
  */
void SX126X_Reset(void)
{
    s_cad_cfg_done = 0U; /* Reapply the original CAD parameters after RF reset. */
    Delay_Us(50);
    LORA_RST_L;
    Delay_Us(50);
    LORA_RST_H;
    Delay_Us(50);
}
void Lora_Gpio_Init(void)
{

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	 
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;   //CTL1 ,CLT2 ,   RST          
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	 
	GPIO_InitStruct.Pin = GPIO_PIN_11;   //CTL1 ,CLT2 ,   RST          
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		  /*Configure GPIO pins : PA11 PA15 */
	
  GPIO_InitStruct.Pin = GPIO_PIN_4;  //LORA SPI CS
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	 
	GPIO_InitStruct.Pin = GPIO_PIN_10;   //BUSY                  
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	 

	 
  GPIO_InitStruct.Pin = GPIO_PIN_0;   //DIO3                   
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//上升沿触发
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH  ;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
	
  GPIO_InitStruct.Pin = GPIO_PIN_4;     //DIO1               
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//上升沿触发
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH  ;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	 GPIO_InitStruct.Pin = GPIO_PIN_5;     //DIO2               
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//上升沿触发
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH  ;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	


	 HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
/**
  * @简介：SX126X  IO初始化及配置
  * @参数：无
  * @返回值：无
  */
extern uint8_t sleep_mode;
void SX126X_InitIo(void)
{

	LORA_NSS_H;
  LORA_SW1_L;
  LORA_SW2_L;
  LORA_RST_H;
//	printf("SX126X_InitIo\r\n");
//	if(sleep_mode==0)
//	{
	  MX_SPI1_Init();
//	}
//	else{
//		MX_SPI1_Init_Stop();
//	}
}
/**
  * @简介：SX126X  TX/RX的PA切换
  * @参数：bool txEnable  切换逻辑；真：作为TX。假：作为RX   为硬件两个PA控制IO口
  * @返回值:无
  */
void SX126XWriteRxTx( u8 txEnable )
{
    if( txEnable != 0 )       //如果为真，为TX
    {
        LORA_SW1_L;
        LORA_SW2_H;
    }
    else  //为假，为RX
    {
        LORA_SW1_H;
        LORA_SW2_L;
    }
}
/**
  * @简介：SX126X 切断高频开关，防止高频开关工作，增加功耗
  * @参数：无
  * @返回值:无
  */
void SX126XSwitchOff(void)
{
    LORA_SW1_L;
    LORA_SW2_L;
}

/**
  * @简介：DIO1中断开启使能
  * @参数：无
  * @返回值：无
  */
/**
  * @简介：该函数为DIO1输入中断开启使能；
  * @参数：无
  * @返回值：无
  */
void SX126X_DIO1_INTENABLE(void)
{
	  __HAL_RCC_GPIOB_CLK_ENABLE();
  //__HAL_RCC_GPIOC_CLK_ENABLE();
   GPIO_InitTypeDef GPIO_InitStructure= {0};
	GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 
	//    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
/**
  * @简介：该函数为DIO1输入中断关闭使能；
  * @参数：无
  * @返回值：无
  */
void SX126X_DIO1_INTDISABLE(void)
{
	  __HAL_RCC_GPIOB_CLK_ENABLE();
  //__HAL_RCC_GPIOC_CLK_ENABLE();
   GPIO_InitTypeDef GPIO_InitStructure= {0};
	GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//   HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void DIO1_EnableInterrupt(void)
{
   SX126X_DIO1_INTENABLE();
	//    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
}
/**
  * @简介：DIO0中断关闭使能
  * @参数：无
  * @返回值：无
  */
void DIO1_DisableInterrupt(void)
{
    SX126X_DIO1_INTDISABLE();
}
/**
  * @简介：DIO1引脚状态获取
  * @参数：无
  * @返回值：State表示DIO1获取的电平，高电平"1",低电平"0"
  */
u8 DIO1_GetState(void)
{
    return 0;
}
/**
  * @简介：DIO2中断开启使能
  * @参数：无
  * @返回值：无
  */
void DIO2_EnableInterrupt(void)
{
//    SX126X_DIO2_INTENABLE();
}
/**
  * @简介：DIO2中断关闭使能
  * @参数：无
  * @返回值：无
  */
void DIO2_DisableInterrupt(void)
{
//    SX126X_DIO2_INTDISABLE();
}
/**
  * @简介：DIO2引脚状态获取
  * @参数：无
  * @返回值：State表示DIO2获取的电平，高电平"1",低电平"0"
  */
u8 DIO2_GetState(void)
{
    return 0;
}
/**
  * @简介：Busy中断开启使能
  * @参数：无
  * @返回值：无
  */
void Busy_EnableInterrupt(void)
{
//    SX126X_Busy_INTENABLE();
}
/**
  * @简介：Busy中断关闭使能
  * @参数：无
  * @返回值：无
  */
void Busy_DisableInterrupt(void)
{
//    SX126X_Busy_INTDISABLE();
}
/**
  * @简介：Busy引脚状态获取
  * @参数：无
  * @返回值：State表示Busy获取的电平，高电平"1",低电平"0"
  */
u8 Busy_GetState(void)
{
    return LORA_BUSY_VAL;
}
/**
  * @简介：Standby模式
  * @参数：STDBY_RC or STDBY_XOSC
  * @返回值：无
*/
void SX126xSetStandby( RadioStandbyModes_t standbyConfig )
{
		RadioStatus_t status;
	  uint32_t t0 = HAL_GetTick();      /* ★ V1_0_2 修正：用"时间上限"替代原来的 65535
	                                       次盲等。射频不回应时 65535 次 SPI 读要 3 s 以上，
	                                       而循环里既没有超时、也没有 re_iwdg() ——
	                                       主循环被拖死 + IWDG(≈2.08s) 复位，
	                                       现场就是"按键按不动，然后自己重启"。 */
    SX126xWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );//STDBY 模式配置 0设置为 STDBY_RC 模式，运行 RC13M 1设置为 STDBY_XOSC，运行 XTAL 32MHz
		status=SX126xGetStatus();

    if( standbyConfig == STDBY_RC )
    {
			 while( status.Fields.ChipMode != 0X02 )
			{
				 re_iwdg();
				 if((uint32_t)(HAL_GetTick() - t0) > SX126X_STANDBY_TIMEOUT_MS)
				 {
					 SX126x_StandbyTimeoutCnt++;    /* 超时：放弃等待，不卡死整机 */
            OperatingMode = MODE_UNKNOWN;   /* ★★★ V1_0_13：超时了就**不能再谎称**已在 STDBY
                                             * ------------------------------------------------------------------
                                             * 原来这里无条件写 MODE_STDBY_RC。一旦芯片因为
                                             * "刚从 Sleep/异常态醒来、CAD 还在跑、SPI 受扰"
                                             * 而没真正到达 STDBY，这个假状态会让
                                             * SX126xCheckDeviceReady() 认为"芯片醒着、不用叫醒"，
                                             * 接着 SX126xWaitOnBusy() 去等一个**永远不会变低**的
                                             * BUSY（手册 §8.3.1：Sleep 中 BUSY 被 20k 上拉保持高）
                                             * —— 现场就是"卡在 while 里，然后看门狗复位"。
                                             * 现在改成 MODE_UNKNOWN：下一次命令会先做一次
                                             * SX126xWakeup()（NSS 下降沿）把芯片拉回确定态。 */
            return;
				 }
				 status=SX126xGetStatus();
			}
      OperatingMode = MODE_STDBY_RC;
    }
    else
    {
			 while( status.Fields.ChipMode != 0X03 )
			{
				 re_iwdg();
				 if((uint32_t)(HAL_GetTick() - t0) > SX126X_STANDBY_TIMEOUT_MS)
				 {
					 SX126x_StandbyTimeoutCnt++;
            OperatingMode = MODE_UNKNOWN;   /* ★★★ V1_0_13：超时了就**不能再谎称**已在 STDBY
                                             * ------------------------------------------------------------------
                                             * 原来这里无条件写 MODE_STDBY_RC。一旦芯片因为
                                             * "刚从 Sleep/异常态醒来、CAD 还在跑、SPI 受扰"
                                             * 而没真正到达 STDBY，这个假状态会让
                                             * SX126xCheckDeviceReady() 认为"芯片醒着、不用叫醒"，
                                             * 接着 SX126xWaitOnBusy() 去等一个**永远不会变低**的
                                             * BUSY（手册 §8.3.1：Sleep 中 BUSY 被 20k 上拉保持高）
                                             * —— 现场就是"卡在 while 里，然后看门狗复位"。
                                             * 现在改成 MODE_UNKNOWN：下一次命令会先做一次
                                             * SX126xWakeup()（NSS 下降沿）把芯片拉回确定态。 */
            return;
				 }
				 status=SX126xGetStatus();
			}
      OperatingMode = MODE_STDBY_XOSC;
    }
}

/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSleep( void )
{	 
    SleepParams_t params = { 0 };
    params.Fields.WarmStart = 1;
    SX126xSetSleep( params );
    SX126XSwitchOff();//切断高频开关，防止休眠模式下高频开关增加功耗
		
   // HAL_Delay(2);
}
/**
  * @简介：SX126X进入LORA睡眠模式
  * @参数：无
  * @返回值：无
  */
void SX126xSetSleep( SleepParams_t sleepConfig )
{
    SX126xWriteCommand( RADIO_SET_SLEEP, &sleepConfig.Value, 1 );
	  OperatingMode = MODE_SLEEP;
}

/**
  * @简介：Lora模式
  * @参数：packetType
  * @返回值：无
*/
void SX126xSetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    //PacketType = packetType;

    if( packetType == PACKET_TYPE_GFSK )
    {
        SX126xWriteRegister( REG_BIT_SYNC, 0x00 );
    }
    SX126xWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}
/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}
/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xCalibrate( CalibrationParams_t calibParam )
{
    SX126xWriteCommand( RADIO_CALIBRATE, ( uint8_t* )&calibParam, 1 );
}

/**
  * @简介：该函数主要用于防止当低温等情况出现PLL Lock等异常时，可能出现RF无法工作，及时清除错误标志及进行校准
  * @参数：
  * @返回值：无
*/
void SX126xCalibrate_err()
{
		RadioError_t error;
		error=SX126xGetDeviceErrors();
	 // printf("SX126xCalibrate_err\r\n");
		if(error.Value!=0)
		{
			SX126xClearDeviceErrors();
			CalibrationParams_t calibParam;
			calibParam.Value = 0x7F;
			SX126xCalibrate(calibParam);
		}
}	

/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xCalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    SX126xWriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}
/**
  * @简介:频率设置
  * @参数：
  * @返回值：无
*/
tSX126xError SX126xSetRfFrequency(uint32_t frequency )
{
    if((frequency > 960000000) || (frequency < 410000000))
        return PARAMETER_INVALID;
    uint8_t buf[4];
    uint32_t freq = 0;
    SX126xCalibrateImage(frequency);
    freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
    SX126xWriteCommand( RADIO_SET_RFFREQUENCY, buf, 4);
		 return NORMAL;
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];
    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX126xWriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SX126xWriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}


#if (VERSION==LSD4RFC_2L714N10)
/**
  * @简介：发射功率设置，该函数配置是结合硬糒SD4RFC-2L714N10考虑功耗及功率最优设置； 
  * @参数：Power输入值范围：-3~22；
  * @返回值:
  */

void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
	uint8_t buf[2];
    if( power >= 14 )
    {
        power = 15;
    }
    else if( power < -3 )
    {
        power = -3;
    }	 
    SX126xSetPaConfig( 0x04, 0x06, 0x00, 0x01 );
		
    SX126xWriteRegister( REG_OCP, 0x38 ); // current max 140mA for the whole device
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

#else
/**
  * @简介：发射功率设置，该函数配置是结合硬件LSD4RFC-2L722N10考虑功耗及功率最优设置； 
  * @参数：Power输入值范围：-3~22；
  * @返回值:
  */
void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];
    if( power > 22 )
    {
        power = 22;
    }
    else if( power < -3 )
    {
        power = -3;
    }	 
    SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 ); //设置使用的PA以及配置
		
    SX126xWriteRegister( REG_OCP, 0x38 ); // current max 140mA for the whole device
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}
  #endif


/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_800_US );
}

/**
  * @简介:调制参数设置 ，SF、BW、CR、低数率优化设置
  * @参数：无
  * @返回值：无
*/
void SX126xSetLoraModulationParams(void)
{
    uint8_t n;

    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00};
    n = 4;
    buf[0] = G_LoRaConfig.SpreadingFactor;
    buf[1] = G_LoRaConfig.BandWidth;
    buf[2] =  G_LoRaConfig.CodingRate;

		if(G_TsXms > 16.0)
    {
        buf[3] = LOWDATARATEOPTIMIZE_ON;
    }
    else
    {
        buf[3] = LOWDATARATEOPTIMIZE_OFF;
    }

		
	//当LoRa模式下时，BW为500KHz 时寄存器0x0889的第二位为0，否则其它BW或GFSK模式时第二位应该为1
 	if(  G_LoRaConfig.BandWidth == LORA_BW_500 )
     {
			 SX126xWriteRegister(TxModulation, SX126xReadRegister(TxModulation) & 0xfB);
     } 
		 else
		 {
			  SX126xWriteRegister(TxModulation, SX126xReadRegister(TxModulation) | 0x04);
		 }
		
    SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n ); //设置调制参数
}
/**
  * @简介:前导码、报头格式、PL、CRC等设置
  * @参数：无
  * @返回值：无
*/
void SX126xSetLoraPacketParams(void)
{
    //SX126xSetPacketType( PACKET_TYPE_LORA );
    uint8_t n;
    uint8_t buf[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

		//SF6、SF7时要求前导长度大于等于12
		if( ( G_LoRaConfig.SpreadingFactor == LORA_SF5 ) || (G_LoRaConfig.SpreadingFactor == LORA_SF6 ) )
      {
				if( G_LoRaConfig.PreambleLength < 12 )
					{
						G_LoRaConfig.PreambleLength = 12;
					}
              
      }   

    n = 6;
    buf[0] = (G_LoRaConfig.PreambleLength >> 8 ) & 0xFF;
    buf[1] = G_LoRaConfig.PreambleLength;
    buf[2] = G_LoRaConfig.HeaderType;
    buf[3] = G_LoRaConfig.PayloadLength;
    buf[4] = G_LoRaConfig.CrcMode;
    buf[5] = G_LoRaConfig.InvertIQ;
    SX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );//设置数据包处理块的参数
		
		 // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
		if( G_LoRaConfig.InvertIQ == LORA_IQ_INVERTED )
		{
				// RegIqPolaritySetup = @address 0x0736
				SX126xWriteRegister( 0x0736, SX126xReadRegister( 0x0736 ) & ~( 1 << 2 ) );
		}
		else
		{
				// RegIqPolaritySetup @address 0x0736
				SX126xWriteRegister( 0x0736, SX126xReadRegister( 0x0736 ) | ( 1 << 2 ) );
		}
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetStopRxTimerOnPreambleDetect( u8 enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetLoRaSymbNumTimeout( uint8_t SymbNum )
{
    SX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1 );
}
/**
  * @简介：SX126X 配置频率检查判断
  * @参数：无
  * @返回值：1 or 0
  */
u8 LoRaConfig_Check(void)
{
if((G_LoRaConfig.LoRa_Freq < 410000000) || (G_LoRaConfig.LoRa_Freq > 960000000))
        return 0;

G_LoRaConfig.BandWidth = (RadioLoRaBandwidths_t)(G_LoRaConfig.BandWidth & 0x0F);

    if((0x3>=G_LoRaConfig.BandWidth)&(0x7<=G_LoRaConfig.BandWidth))//LLCC68 BW 限制
        return 0;
  //计算BandWidth
    switch(G_LoRaConfig.BandWidth) {
    case  LORA_BW_500:
        G_BandWidthKHz = 500.0;
        break;
    case  LORA_BW_250:
        G_BandWidthKHz = 250.0;
        break;
    case  LORA_BW_125:
        G_BandWidthKHz = 125.0;
        break;
    case  LORA_BW_062:
        G_BandWidthKHz = 62.5;
        break;
    case  LORA_BW_041:
        G_BandWidthKHz = 41.67;
        break;
    case  LORA_BW_031:
        G_BandWidthKHz = 31.25;
        break;
    case  LORA_BW_020:
        G_BandWidthKHz = 20.83;
        break;
    case  LORA_BW_015:
        G_BandWidthKHz = 15.63;
        break;
    case  LORA_BW_010:
        G_BandWidthKHz = 10.42;
        break;
    case  LORA_BW_007:
        G_BandWidthKHz = 7.81;
        break;
    }
		
		//LLCC68 速率限制
		 G_LoRaConfig.SpreadingFactor = (RadioLoRaSpreadingFactors_t)(G_LoRaConfig.SpreadingFactor & 0x0F);
			if(G_LoRaConfig.BandWidth==LORA_BW_125)
			{
				if((G_LoRaConfig.SpreadingFactor >= LORA_SF10) || (G_LoRaConfig.SpreadingFactor < LORA_SF5))
				return 0;
			}
			if(G_LoRaConfig.BandWidth==LORA_BW_250)
			{
				if((G_LoRaConfig.SpreadingFactor >= LORA_SF11) || (G_LoRaConfig.SpreadingFactor < LORA_SF5))
				return 0;
			}
			if(G_LoRaConfig.BandWidth==LORA_BW_500)
			{
				if((G_LoRaConfig.SpreadingFactor >= LORA_SF12) || (G_LoRaConfig.SpreadingFactor < LORA_SF5))
				return 0;
			}
		
		//计算LoRa码元周期，单位ms
    G_TsXms = pow(2, G_LoRaConfig.SpreadingFactor) / G_BandWidthKHz;
			printf("G_TsXms = %d\r\n", (int)G_TsXms );
		//G_CadTimeout = G_TsXms * (pow(2, G_CadSymbolNum)) * 1100 / 15.625;
			
		 G_LoRaConfig.CodingRate = (RadioLoRaCodingRates_t)(G_LoRaConfig.CodingRate & 0x07);
    if((G_LoRaConfig.CodingRate > LORA_CR_4_8) || (G_LoRaConfig.CodingRate < LORA_CR_4_5))
        return 0;
    if(G_LoRaConfig.PowerCfig > 22)
        return 0;
		 return 1;
   
}
/**
* @简介:SX126X初始化
  * @参数：无
  * @返回值：tSX126xError   错误枚举内容
  */

tSX126xError SX126x_Lora_init(void)
{
    if(0 == LoRaConfig_Check()) //如果输入参数错误
    {
      return PARAMETER_INVALID;  //报错输入
    }

//		SX126X_SPI_Init();           
    SX126X_InitIo();                //SPI初始化
		
		/*------------------------------------------------
        SX126X Lora模式初始化 --------  */
    SX126X_Reset();                 //复位RF
		SX126xWakeup();
		SX126xSetStandby(STDBY_RC);    //切换到LoRamode，standby状态
		SX126xWriteRegister(TxClampConfig, SX126xReadRegister(TxClampConfig) | 0x1E); //SX126X 芯片PA内部有过压保护，如果失配会导致功率降低问题；
		                                                                //修改寄存器TxClampConfig来优化PA的阈值，位4-1必须
		                                                                //设置为"1111“（默认为0100）---2019.07.31
		SX126xSetRegulatorMode( USE_DCDC );
		if (VERSION==L_LRMRL22_97NN4||VERSION==LSD4RF_2R717N40||VERSION==LSD4RF_2R822N30)
		{
			SX126xClearDeviceErrors(); //如果使用DIO3去控制TCXO，那么每当上电之后或者是以Cold-Start的条件从Sleep唤醒，需要
		                              //调用ClearDeviceErrors命令去清除OSC_START_ERR标志
			                            //如果不清除在低温冷启动时可能会出现TCXO不工作情况---2019.07.29
			SX126xSetDio3AsTcxoCtrl(TCXO_CTRL_2_7V, 320);  //超时时间设置 320*15.625us=5ms
			SX126xWriteRegister( REG_XTA_TRIM, 0x2F ); 
			
			CalibrationParams_t calibParam;
			calibParam.Value = 0x7F;
			SX126xCalibrate(calibParam); //Calibrate the RC13, RC64, ADC , Image、PLL
																	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
		}
		
		SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
    /*------------------------------------------------
    SPI 验证                   */
    uint8_t test = 0;
    SX126xWriteRegister( REG_LR_PAYLOADLENGTH, 0x09);   //选一个用不到的寄存器来做验证
    test = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
		printf("test==%d\r\n",test);
    if(test != 0x09)
        return SPI_READCHECK_WRONG;
		
		SX126xWriteRegister(REG_RX_GAIN, 0x96);//开启Rx Boosted gain,默认0x94不开启  // RX Boosted 增益设置
    SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);  // 指定频段镜像校准
    SX126xSetRfTxPower( G_LoRaConfig.PowerCfig );  //设置PA的输出功率和Ramping时间
		SX126xSetBufferBaseAddress( 0x00, 0x00 );//设置 Tx 和 Rx 模式下包处理的数据缓存区的基地址
    SX126xSetLoraModulationParams();   //SF、BW、CR、LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams();  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );//用来设置 IRQ 标志位
    return NORMAL;
}
/**
  * @简介:清标志
  * @参数：irq
  * @返回值：无
*/
void SX126xClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}
/**
  * @简介:获取标志
  * @参数：无
  * @返回值：对应标志
*/
uint16_t SX126xGetIrqStatus( void )
{
    uint8_t irqStatus[2];

    SX126xReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

RadioError_t SX126xGetDeviceErrors( void )
{
    uint8_t err[] = { 0, 0 };
    RadioError_t error = { .Value = 0 };

    SX126xReadCommand( RADIO_GET_ERROR, ( uint8_t* )err, 2 );
    error.Fields.PaRamp     = ( err[0] & ( 1 << 0 ) ) >> 0;
    error.Fields.PllLock    = ( err[1] & ( 1 << 6 ) ) >> 6;
    error.Fields.XoscStart  = ( err[1] & ( 1 << 5 ) ) >> 5;
    error.Fields.ImgCalib   = ( err[1] & ( 1 << 4 ) ) >> 4;
    error.Fields.AdcCalib   = ( err[1] & ( 1 << 3 ) ) >> 3;
    error.Fields.PllCalib   = ( err[1] & ( 1 << 2 ) ) >> 2;
    error.Fields.Rc13mCalib = ( err[1] & ( 1 << 1 ) ) >> 1;
    error.Fields.Rc64kCalib = ( err[1] & ( 1 << 0 ) ) >> 0;
    return error;
}

void SX126xClearDeviceErrors( void )
{
    uint8_t buf[2] = { 0x00, 0x00 };
    SX126xWriteCommand( RADIO_CLR_ERROR, buf, 2 );
}


/**
  * @简介:当在LoRa数据包接收模式时，开启RxTimeout，且在隐性报头（Implicit Mode）模式时，可能出现接收完成后，超时定时器不关闭现象，需要进行停止RTC和清除超时事件
  * @参数：
  * @返回值：无
*/
void SX126xClearTimeoutEvent( void )
{
		// WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
		// RegRtcControl = @address 0x0902
		SX126xWriteRegister( 0x0902, 0x00 );
		// RegEventMask = @address 0x0944
		SX126xWriteRegister( 0x0944, SX126xReadRegister( 0x0944 ) | ( 1 << 1 ) );
		// WORKAROUND END
}
						
/**
  * @简介:DIO2开关控制
  * @参数：enable
  * @返回值：无
*/
void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
    SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

/**
  * @简介:DIO3 TCXO电源控制及功能开启
  * @参数：enable
  * @返回值：无
*/
void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}

RadioStatus_t SX126xGetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status;

    SX126xReadCommand( RADIO_GET_STATUS, &stat, 1 );
//	 printf(" stat==%d\r\n",stat);
    status.Value = stat;
    return status;
}
/**
  * @简介:获人彩薄RSSI值
  * @参数：无
  * @返回值：RSSI值
*/
int8_t SX126xGetRssiInst( void )
{
    uint8_t buf[1];
    int8_t rssi = 0;

    SX126xReadCommand( RADIO_GET_RSSIINST, buf, 1 );
    rssi = -buf[0] >> 1;
    return rssi;
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xGetRxBufferStatus( uint8_t *payloadLength )  //, uint8_t *rxStartBufferPointer
{
    uint8_t status[2];

    SX126xReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if( ( SX126xReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )   
    {
        *payloadLength = SX126xReadRegister( REG_LR_PAYLOADLENGTH );  
    }                         //这里的判断其实和外层判断报头为显隐性是一样的，这两个寄存器规格书中未给出，但是验证是等效成立的。
    else
    {
        *payloadLength = status[0];
    }
    // *rxStartBufferPointer = status[1];
}
/**
  * @简介:获取数据包RSSI及SNR值
  * @参数：
  * @返回值：无
*/
void SX126xGetPacketStatus( S_LoRaPara *pktStatus )
{
    uint8_t status[3];

    SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );

    pktStatus->AvgPacket_RSSI = -status[0] >> 1;   //上一个数据包的RSSI平均值
    ( status[1] < 128 ) ? ( pktStatus->Packet_SNR = status[1] >> 2 ) : ( pktStatus->Packet_SNR = ( ( status[1] - 256 ) >> 2 ) );
    pktStatus->LastPacket_RSSI = -status[2] >> 1;
    
    pktStatus->Ack_RSSI = status[2] >> 1;
		//printf("AvgPacket_RSSI == %d ;LastPacket_RSSI == %d ;Ack_RSSI == %d\r\n", pktStatus->AvgPacket_RSSI,pktStatus->LastPacket_RSSI,pktStatus->Ack_RSSI);
}

void SX126xSetFs( void )
{
    SX126xWriteCommand( RADIO_SET_FS, 0, 0 );
}

/**
  * @简介:进入发射
  * @参数：
  * @返回值：无
*/
void SX126xSetTx( uint32_t timeout )
{
    uint8_t buf[3];
    SX126xClearIrqStatus( IRQ_RADIO_ALL );
    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TX, buf, 3 ); //超时时间长度
    OperatingMode = MODE_TX;          /* ★ V1_0_13：同上，发射态要记下来 */
}
/**
  * @简介:进入接收
  * @参数：
  * @返回值：无
*/
void SX126xSetRx( uint32_t timeout )
{
    uint8_t buf[3];
    SX126xClearIrqStatus( IRQ_RADIO_ALL );
    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
    OperatingMode = MODE_RX;          /* ★ V1_0_13：状态变量必须真实。
                                       * 原来这里什么都不记，进入 RX 连续接收后
                                       * OperatingMode 还停在 STDBY_RC，
                                       * 后续判断与手册语义完全脱节。 */
}

/**
  * @简介:发送数据
  * @参数：PL、size、timeout
  * @返回值：无
*/
void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
    SX126xWriteBuffer(0x00, payload, size );
    SX126xSetTx( timeout );
}
/**
  * @简介:发送数据包
  * @参数：data
  * @返回值：无
*/
extern uint8_t communication_states;
void SX126X_TxPacket( uint8_t*data ,uint16_t data_len,uint16_t PreambleLength)
{
//	  SX126xSetStandby(STDBY_XOSC);  //这里如果不是使用TCXO版本，STDBY_XOSC和STDBY_RC都无影响，如果使用的是TCXO版本，需要使用STDBY_XOSC，否则超时时间会增加到切换到TX模式周期中；
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
		SX126xSetStandby(STDBY_RC);//设置运行在哪一个时钟的模式
	  SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(1);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_TX_DONE|IRQ_RX_TX_TIMEOUT ,
                           IRQ_TX_DONE|IRQ_RX_TX_TIMEOUT ,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );//开启Tx、Tx超时中断，映射到DIO1；
                           //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
//    SX126xSetLoraModulationParams( );   //SF、BW、CR、LOWDATARATEOPTIMIZE
//	  printf("G_LoRaConfig.PreambleLength==%d\r\n",G_LoRaConfig.PreambleLength);
  	G_LoRaConfig.PreambleLength =PreambleLength; //前导长度
	  G_LoRaConfig.PayloadLength=data_len; //数据长度
    SX126xSetLoraPacketParams( );  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
//		printf("LORA_send\r\n");
//	  hex_p(data,data_len);
    SX126xSendPayload(data, data_len, 0x2fffff );// 开启了发射超时，时间为：0x2fffff*15.625≈50s,
	             //这是一个安全措施，可能由于某种突发导执行了该命令但是不产生TxDOne,这时设置Tx超时就可以保证状态不被打乱，用户根据实际应用配置情况进行设置超时时间
} 
/**
  * @简介  :发送数据包
  * @参数  ：uint8_t*cbuf:发送数据指针
  * @返回值：无
  * @说明  ：接收完数据后DIO1从低电平变成高电平
*/
void LSD_RF_RXmode()
{
	 DIO1_EnableInterrupt();
   SX126X_StartRx();//进入接收模式，准备接收数据
}

/**
  * @简介  :发送数据包 ，要写的时候设置前置包的长度
  * @参数  ：uint8_t*cbuf:发送数据指针
  * @返回值：无
  * @说明  ：数据发送完成后DIO1从低电平变成高电平，每次调用次函数，会自动现将DIO1变为低电平，等待高电平
*/
extern   volatile uint8_t  Lora_Rx_state ;
uint8_t  cnt_lora_tx=0;
void LSD_RF_SendPacket(uint8_t*cbuf,uint16_t data_len,uint16_t q_num)
{
//   unsigned long int j=16777215;                   //超时用，用户需要根据实际情况来调整
 
   SX126X_TxPacket(cbuf ,data_len,q_num);
   Lora_Rx_state=1;
	 cnt_lora_tx=0;
  /* ★★★ V1_0_2 修正：这个等待原来**没有超时**，cnt_lora_tx 就是为超时准备的
     计数器，但从来没有被判断过（下面那行注释掉的 j 是同一种意图）。
     为什么必须补：
       · SX126X_TxPacket() 把 DIO1 映射成 TxDone|TxTimeout，射频自己的超时是
         0x2fffff × 15.625us ≈ 50 s；也就是说"DIO1 一直不来"这件事在射频侧
         要 50 s 才会变成一次超时中断；
       · 而本循环里既没有 re_iwdg() 也没有出口 -> 主循环停转（按键、
         UI_display、Light_check、sensor_rx_task 全不跑）-> 约 2.08 s 后被
         IWDG 复位。现场表现："按了没反应，过一下自己重启"。
     现在：用 Rt_tick（100ms 节拍，STOP 下仍在走）做 LORA_TX_WAIT_TIMEOUT_SEC
     上限，每轮喂狗；超时就把 Lora_Rx_state 收尾成 0 并记一次计数，
     绝不留一个卡在 1 的状态把后续流程带偏。 */
  {
    uint32_t t0 = Rt_tick;
    /* ★★ 看门狗审查补丁（第三轮）：第二计时基准。
       Rt_tick 由 LPTIM1 的中断累加，是本工程所有软件超时的共同时基。
       万一 LPTIM1 停走，上面那条 Rt_tick 超时永远不成立，而本循环每轮都
       喂狗 -> 变成"喂着狗死等"的永久死锁（比看门狗复位更糟，连自动复位
       自愈都没有了）。所以再加一条与 LPTIM1 无关的基准。
       注意 HAL_GetTick 用的是 SysTick，**在 STOP 下不走**，所以它只统计
       "醒着"的时间；这正好够用：它保证即使 Rt_tick 冻住也一定会退出循环
       （此时 sleep_stop_ms() 会因为 g_rt_tick_alive==0 而拒绝进 STOP，
       work_sleep() 退化成 HAL_Delay(1)，SysTick 正常推进）。 */
    uint32_t h0 = HAL_GetTick();

    do
    {
      /* ★★★ V1_0_11：等 TxDone 期间必须继续搬运 USART1
         （环形缓冲只有 128B ≈ 11ms @115200，不搬就丢 CH584M 的数据）。
         本循环最长 385ms（6000 符号前导），不搬串口必丢帧。 */
      sensor_rx_task();
      work_sleep();
      cnt_lora_tx++;
      re_iwdg();                                   /* 等待期间必须喂狗 */
      if (((uint32_t)(Rt_tick - t0) > ((uint32_t)LORA_TX_WAIT_TIMEOUT_SEC * 10U)) ||
          ((uint32_t)(HAL_GetTick() - h0) > ((uint32_t)LORA_TX_WAIT_TIMEOUT_SEC * 3000U)))
      {
        Lora_TxWaitTimeoutCnt++;                   /* 超时：放弃等待 */
        Lora_Rx_state = 0;
        break;
      }
    }
    while(Lora_Rx_state == 1);
  }
//   while( (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)&j-- );
	// printf("Lora_Data_state==%d\r\n",Lora_Rx_state);
}


void SX126X_CADinit_to_bc(void)
{

  SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
 
	SX126xSetStandby(STDBY_RC);

  SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
		
  SX126xSetRegulatorMode( USE_DCDC );

  //这里按照SF=5时推荐配置；参数配置与SF、BW配置及功耗考虑有关，可以参照本函书说明配置
  SX126xSetCadParams( LORA_CAD_04_SYMBOL, 21, 10, LORA_CAD_ONLY, 0x20 );

  SX126XWriteRxTx(0);

  SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
		
  SX126xSetDioIrqParams( IRQ_CAD_DONE|IRQ_CAD_ACTIVITY_DETECTED ,
	                       IRQ_RADIO_NONE,
                         
	                       IRQ_CAD_ACTIVITY_DETECTED,
                         IRQ_CAD_DONE
	                       
	                       );
                                //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
}
/**
  * @简介:进入接收
  * @参数：无
  * @返回值：无
*/
void SX126X_StartRx( void )
{
    SX126xSetStandby(STDBY_XOSC);  //这里如果不是使用TCXO版本，STDBY_XOSC和STDBY_RC都无影响，如果使用的是TCXO版本，需要使用STDBY_XOSC，否则超时时间会增加到切换到RX模式周期中；
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
		SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(0);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_CRC_ERROR|IRQ_RX_TX_TIMEOUT, 
                           IRQ_RX_DONE|IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );  //打开RX、CRC、RX超时中断，映射RX中断到DIO1；
	  SX126xSetStopRxTimerOnPreambleDetect( 0 );//用来选择定时器停止是检测前导码还是报头；
    SX126xSetLoRaSymbNumTimeout(0); //设置用来验证正确LoRa信号的字符个数，多数情况下设置为0即可；
	      //一般设置为0时，即表示接收机验证一个LoRa信号字符，但是由于各种突发情况存在极低的可能误判，所以加大验证数值可以提高一定的正确性；
//    SX126xSetLoraModulationParams();   //SF、BW、CR、LOWDATARATEOPTIMIZE设置
//    SX126xSetLoraPacketParams();  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0x2fffff ); // 开启了接收超时，时间为：0x2fffff*15.625us≈50s,用户根据实际应用配置情况进行设置超时时间；（默认情况是检测到报头（LoRa）或同步字（FSK）才会终止该定时器）
}
/**
  * @简介:获取数据
  * @参数：buffer，size
  * @返回值：无
*/
void SX126xGetPayload( uint8_t *buffer, uint8_t *size)
{
    SX126xReadBuffer( 0x00, buffer, *size );
}

/**
  * @简介：RF接收数据包
  * @参数：uint8_t*cbuf接收数组指针
  * @返回值：无
  */
void SX126X_RxPacket(uint8_t*cbuf)
{

	 SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xGetPacketStatus(&G_LoRaPara);//收到一包数据包后，返回SNR、RSSI等各种信息
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    if( G_LoRaConfig.HeaderType == LORA_PACKET_EXPLICIT ) {
        //有头那么从寄存器中读，否则按照G_LoRaConfig设置长度读取FIFO
        SX126xGetRxBufferStatus(&G_LoRaConfig.PayloadLength);
    }
    SX126xGetPayload(cbuf, &G_LoRaConfig.PayloadLength);

}

/**
  * @简介：WOR初始化
  * @参数：无
  * @返回值：无
  */

/*
cadSymbolNum:表示用来检测CAD信号的符号数，影响功耗及CAD误判率，数字越大，功耗越大，误判率越低；
cadDetPeak：
cadDetMin：这个两个参数设置和灵敏度有关，受影响于SF和BW；
cadExitMode:CAD_ONLY\CAD_RX 指执行完CAD后进入的状态；
cadTimeout:指CAD完成后如果进入RX，执行RX的时间；
对于cadSymbolNum、cadDetPeak、cadExitMode这几个参数推荐有下：
1、按照通信成功率最佳推荐：
  SF   	 cadSymbolNum 				cadDetPeak				cadExitMode						CAD Consumption(nAh)
	7			LORA_CAD_02_SYMBOL				22								10											2.84				
	8			LORA_CAD_02_SYMBOL				22								10											5.75
	9			LORA_CAD_04_SYMBOL				23								10											20.44
	10		LORA_CAD_04_SYMBOL				24								10											41.36
	11		LORA_CAD_04_SYMBOL				25								10											134.55

2、按照通信成功率兼容考虑消耗最最佳推荐：
  SF   	 cadSymbolNum 				cadDetPeak				cadExitMode						CAD Consumption(nAh)
	7			LORA_CAD_02_SYMBOL				22								10											2.84				
	8			LORA_CAD_02_SYMBOL				22								10											5.75
	9			LORA_CAD_02_SYMBOL				24								10											11.7
	10		LORA_CAD_02_SYMBOL				25								10											23.86
	11		LORA_CAD_02_SYMBOL				26								10											48.79
	
	以上推荐是根据实际测试结果给出来的，实际应用中如有更严格要求可调整参数进行实测判断结果；
*/
void SX126xSetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout )
{
    uint8_t buf[7];

    buf[0] = ( uint8_t )cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = ( uint8_t )cadExitMode;
    buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
    buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
    buf[6] = ( uint8_t )( cadTimeout & 0xFF );
    /* ★★★ V1_0_11：这里原来只发 5 字节，漏掉了 cadTimeout 的高 2 字节
     * ------------------------------------------------------------------
     * SX1268 手册 §13.1.6 SetCadParams（命令 0x88）的参数是 **7 字节**：
     *   cadSymbolNum(1) + cadDetPeak(1) + cadDetMin(1) + cadExitMode(1)
     *   + cadTimeout(3，MSB 在前)
     * 上面的 buf[7] 明明已经按 3 字节把 cadTimeout 填好了，写命令时却只发
     * buf,5：芯片收到的第 5 个字节被当成 cadTimeout 的最高字节，剩下两个
     * 字节保持芯片内的原值 —— 也就是说 **cadTimeout 配的根本不是我们要的值**。
     * 本工程 CAD 用的是 LORA_CAD_RX（见 SX126X_CADinit），按手册 cadTimeout
     * 只在这个退出模式下有意义：它决定"CAD 命中后芯片自己先在 RX 待多久"。
     * 当前值 0x20 = 32 × 15.625us = 0.5ms，因为代码随后立刻调用
     * SX126X_WOR_Exit() 重新配置成 RX Continuous，所以现象上看不出来；
     * 但只要有人去调这个 0x20（本函数是唯一出口），**改大改小都静默无效**。
     * 一并修掉，避免留坑。★ 参数值、调用点、CAD 判据一律未改。 */
    SX126xWriteCommand( RADIO_SET_CADPARAMS, buf, 7 );
}


/**
  * @简介：SX126X 进CAD
  * @参数：无
  * @返回值：无
  */
void SX126xSetCad( void )
{
    SX126xWriteCommand( RADIO_SET_CAD, 0, 0 );
    OperatingMode = MODE_CAD;         /* ★ V1_0_13：CAD 也是一种模式，必须记下来。
                                       * 否则"CAD 还没做完就被当成 STDBY"，
                                       * 正是这次卡死链条上的一环。 */
}
/**
  * @简介：SX126X CAD初始化
  * @参数：无
  * @返回值：无
  */
/* ==========================================================================
 * ★★★ V1_0_13：确保芯片站在 STDBY_RC 上
 * --------------------------------------------------------------------------
 * 手册 §9.3 明确：**芯片只能从 STDBY_RC 进入 Sleep**；
 *   SetCad / SetRx 也都要求先到 STDBY。
 * 而 SX126xSetStandby() 超时时会把 OperatingMode 置成 MODE_UNKNOWN
 *   （见那里的说明），所以这里用"看状态 + 补一次"的方式确认：
 *   失败了再发一次 SetStandby；两次都不行就只能在 UNKNOWN 下继续 ——
 *   但至少下一次命令会先 Wakeup，不会死等 BUSY。
 * 返回值：1 = 已确认在 STDBY_RC；0 = 仍未确认
 * ========================================================================== */
static uint8_t SX126xEnsureStandbyRC( void )
{
    SX126xSetStandby( STDBY_RC );
    if( SX126xGetOperatingMode( ) != MODE_STDBY_RC )
    {
        SX126xSetStandby( STDBY_RC );      /* 补一次（手册要求的前置条件） */
    }
    return ( SX126xGetOperatingMode( ) == MODE_STDBY_RC ) ? 1U : 0U;
}

void SX126X_CADinit(void)
{
   //printf("77777\r\n");
  SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
 // printf("666aaaa\r\n");
  (void)SX126xEnsureStandbyRC();   /* ★ V1_0_13：原来直接 SX126xSetStandby(STDBY_RC)，
                                    * 超时了也算数。改走"确认 + 补一次"的路径，
                                    * 因为 SetCad 要求芯片在 STDBY（手册 §9.2/§9.3）。
                                    * 返回 0 也继续 —— 不能因为射频一时不回应就卡住整机。 */

  /* ★★★ V1_0_20：CAD 周期里**不再重发三条"配置类"命令**（温启动配置是保留的）
   * ------------------------------------------------------------------
   * 现场实测：CAD 路径每 100ms 一个周期，MCU 清醒 3.00ms/周期（J-Link 实测
   *   g_cad_awake_ms），占单台息屏功耗约 42μA，是空闲功耗的第一大项。
   * 手册依据（DS_SX1268_V1.1 §13.1.1 SetSleep）：
   *   · **warm start（配置保留）**：本工程 SX126xSleep() 用的就是
   *     `params.Fields.WarmStart = 1`（Sx126x_Driver.c 的 SX126xSleep），
   *     唤醒后 PacketType / RegulatorMode / CadParams **本来就还在芯片里**；
   *   · 而这三条命令每次 CAD 都重发一遍，纯属浪费 SPI 时间（约 3 条命令
   *     占每次 CAD 清醒时间的一大半）。
   * 所以改成"**只在本次上电后第一次 CAD 时配置一次**"，之后每周期只做
   *   EnsureStandbyRC（Standby）+ WriteRxTx(0)（射频开关切回接收）+
   *   ClearIrqStatus + SetDioIrqParams（CAD 中断映射）+ SetCad。
   * ★ 为什么后面两条不能省：发射路径会把 DIO 映射改成 TxDone、把射频开关
   *   切到 TX（SX126X_Awake -> WriteRxTx(1) / SetDioIrqParams(TX_DONE)），
   *   所以每个 CAD 周期必须把它们切回来，否则收不到 CAD 中断。
   * Every RF hardware reset invalidates the cache; the next CAD configures it again.
   * ========================================================================== */
  if (s_cad_cfg_done == 0U)
  {
      SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式（温启动保留，只配一次）
      SX126xSetRegulatorMode( USE_DCDC );       //开DCDC（温启动保留，只配一次）
      SX126xSetCadParams( LORA_CAD_04_SYMBOL, 21, 10, LORA_CAD_RX, 0x20 );
  SX126xSetLoRaSymbNumTimeout(2);   /* ★★★ V1_0_23：验证 LoRa 信号所需的符号数 0 -> 2
                                     * 手册 §13.1.10：该参数用来"验证收到的确实是 LoRa 信号"，
                                     * 默认 0 = 只验证 1 个符号，噪声毛刺也容易通过；
                                     * 设成 2 可剔除大部分噪声假命中 —— 密集区里每一次假命中
                                     * 都会让本机进一次 RX（4.6mA × 整包时间），代价极高。
                                     * ★ 只加严"假命中"判据，不影响真包的 CAD 灵敏度
                                     *   （真包前导 2000 符号远长于 2 个符号）。 */
      s_cad_cfg_done = 1U;
  }

  SX126XWriteRxTx(0);

  SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
		
  SX126xSetDioIrqParams( IRQ_CAD_DONE|IRQ_CAD_ACTIVITY_DETECTED ,
	                       IRQ_RADIO_NONE,
                         
	                       IRQ_CAD_ACTIVITY_DETECTED,
                         IRQ_CAD_DONE
	                       
	                       );

                                //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
}
/**
  * @简介：WOR初始化
  * @参数：无
  * @返回值：无
  */
void SX126X_WORInit(void)
{
    SX126X_CADinit();        //CAD功能初始化
}
/**
  * @简介：SX126X 启动CAD，采样信道情况一次   采样时间约为(2^SF+32)/BW
  * @参数：无
  * @返回值：无
  */
void SX126X_CAD_Sample(void)
{
 
    SX126xSetCad();
}

void SX126xSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
    uint8_t buf[6];

    buf[0] = ( uint8_t )( ( rxTime >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( rxTime >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( rxTime & 0xFF );
    buf[3] = ( uint8_t )( ( sleepTime >> 16 ) & 0xFF );
    buf[4] = ( uint8_t )( ( sleepTime >> 8 ) & 0xFF );
    buf[5] = ( uint8_t )( sleepTime & 0xFF );
    SX126xWriteCommand( 0x94, buf, 6 );
    OperatingMode = MODE_RX_DC;       /* ★ V1_0_13：占空比接收（芯片大部分时间在睡），
                                       * 记成 MODE_RX_DC，下一次命令前先唤醒 */

}
#define LORA_LISTEN_MODE_MS(x)    (x/0.015625)
#define LORA_LISTEN_MODE_RX_MS   LORA_LISTEN_MODE_MS(15)
#define LORA_LISTEN_MODE_SLEEP_MS   LORA_LISTEN_MODE_MS(285)
/* ==========================================================================
 * ★★★ 注意（V1_0_10 按 DS_SX1268_V1.1 §13.1.7 p.70 钉死，别随手接上这个函数）：
 *   手册规定 **T_preamble + T_header <= 2 * rxPeriod + sleepPeriod**。
 *   本函数用 SetRxDutyCycle(15ms, 285ms) -> 右式 = 2*15 + 285 = **315 ms**，
 *   而本工程发射端"正常 10 分钟周期自发"的**前导是 6000 符号 = 385.55 ms > 315 ms**
 *   （超限 22.4%）=> 一旦把 Listen_mode() 接上（改用硬件占空比接收），
 *   就会**静默收不到包**。要接上必须先满足其中之一：
 *     · sleepPeriod >= 356ms（例如 SetRxDutyCycle(15ms, 360ms)），或
 *     · 把发射前导降到 <= ~4800 符号（同时要改 LoraPro.c 的 for_send_num）。
 *   当前本函数**全工程无调用**（接收靠 CAD_work() 的 CAD 周期），所以暂无风险。
 *   另：本函数 cadDetPeak=20 与另外两处 SetCadParams 的 21 不一致，
 *       且三处都硬编码 LORA_CAD_04_SYMBOL（全局 G_CadSymbolNum 并未参与配置），
 *       接入前建议按 Semtech AN1200.48 复核。
 * ========================================================================== */
void Listen_mode(void)
{
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
	  SX126xSetStandby(STDBY_RC);
	  SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
	  SX126xSetRegulatorMode( USE_DCDC );
	
  //这里按照SF=7时推荐配置；参数配置与SF、BW配置及功耗考虑有关，可以参照本函书说明配置
    SX126xSetCadParams( LORA_CAD_04_SYMBOL, 20, 10, LORA_CAD_RX, 0x20 );
	
	
	  SX126XWriteRxTx(0);//设置为读
		SX126xClearIrqStatus( IRQ_RADIO_ALL );
  	SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_RX_TX_TIMEOUT ,
												IRQ_RX_DONE|IRQ_RX_TX_TIMEOUT ,
												IRQ_RADIO_NONE,
												IRQ_RADIO_NONE );
	  SX126xSetRxDutyCycle(LORA_LISTEN_MODE_RX_MS,LORA_LISTEN_MODE_SLEEP_MS);
}
/**
  * @简介：执行WOR操作
  * @参数：uint8_t cclen  0：进入睡眠;1：进入CAD检测模式
  * @返回值：无
  */
void SX126X_WOR_Execute(uint8_t cclen)
{
    switch(cclen)
    {
    case 0:   //启动睡眠
    {
       /* ★★★ V1_0_13：进 Sleep 之前**必须确认站在 STDBY_RC 上**
        * ------------------------------------------------------------------
        * 手册 §9.3：芯片只能从 STDBY_RC 进入 Sleep。
        * 原来这里直接 SX126xSetStandby(STDBY_RC) 然后 SX126xSleep()：
        *   如果那次 SetStandby 实际超时了（芯片刚从 CAD/RX/异常态过来，
        *   又被定时器立刻叫醒），SetSleep 会被芯片忽略 ——
        *   芯片继续留在 RX(约 10mA)，而软件把 OperatingMode 记成 SLEEP，
        *   之后每一条命令都会去等一个"被上拉保持高的 BUSY"（手册 §8.3.1），
        *   这就是现场"卡在 while 里然后看门狗复位"的完整链条。
        * 现在用 SX126xEnsureStandbyRC()：确认 + 失败补一次。 */
       (void)SX126xEnsureStandbyRC();
       SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags

       SX126xSleep();  //进入睡眠模式
		//	printf("SX126X_WOR_Execute\r\n");
       Lora_Rx_state=0;
       /* ★★★ V1_0_15：把这里的 **HAL_Delay(2) 删掉**（原来每 100ms 就忙等 2ms）
        * ------------------------------------------------------------------
        * 现场事实：本工程的 HAL_Delay 没有被改写，就是 STM32 HAL 原生的
        *   **忙等**（while 轮询 SysTick），CPU 在 16MHz 满速空转约 1.4mA。
        *   而这个函数在 CAD 路径上**每个周期都被调用一次**（息屏 10 次/秒），
        *   于是 2ms × 10 = **20ms/秒的纯空转** ≈ 0.02 × 1.4mA ≈ **28μA**，
        *   比整个射频 CAD 的功耗（约 19μA）还大，是息屏功耗的第一大项。
        * 为什么可以删（手册依据 DS_SX1268_V1.1）：
        *   · §13.1.1 SetSleep：发完 SetSleep 芯片即进入 Sleep，**手册没有要求
        *     主机侧等待**；BUSY 在 Sleep 期间被 20k 上拉保持高（§8.3.1）；
        *   · §8.2.2：芯片靠 **NSS 下降沿**退出 Sleep，唤醒时序由随后的
        *     BUSY 拉低来界定（Table 8-2：温启动 340us）；
        *   · V1_0_13 已把 SX126xCheckDeviceReady() 改成"见到 SLEEP/UNKNOWN
        *     就先做一次 SX126xWakeup() 再等 BUSY"——**唤醒与时序已经完全由
        *     硬件信号（BUSY）保证**，主机再盲等 2ms 没有任何意义。
        * 现在只保留"喂狗 + 泵一次串口"（都是本来就要做的活，~10us）。
        * ★ 若后续多台实测发现异常，第一顺位是把它加回来（但请不要用 2ms，
        *   改成 sleep_stop_ms(2) 让 CPU 进 STOP 而不是空转）。 */
       re_iwdg();
       sensor_rx_task();          /* 环形缓冲只有 128B，顺手搬一次 */
    }
    break;
    case 1:   //进入CAD检测模式
    {
        /* ★★★ V1_0_11：先置状态、再启动 CAD（顺序绝对不能反）
         * ------------------------------------------------------------------
         * 现场问题："LoRa 在切换模式的时候卡死 / 之后再也收不到包"。
         * 根因（状态告知竞态）：
         *   · 本工程 SF5/BW500 下 CAD 极短 —— 手册 T_CAD ≈ (2^SF+32)/BW，
         *     SF5 只有 32 个码片，也就是几百 μs 量级；而 EXTI 是**边沿**触发：
         *       DIO2 = IRQ_CAD_ACTIVITY_DETECTED，DIO3 = IRQ_CAD_DONE
         *       （映射见 SX126X_CADinit()）。
         *   · 原来写成：先 SX126X_CAD_Sample()，再 Lora_Rx_state = 5。
         *     于是存在竞态：CAD_DONE 的边沿可能在"状态还没写成 5"时就被
         *     中断服务了，那时 ISR 读到的还是上一轮的值（0 或 7）：
         *        switch(Lora_Rx_state){ case 3:->8; case 5:->7; }
         *     两个 case 都不命中 -> **这次 CAD 的结果被直接丢弃**（而且
         *     EXTI 挂起位已经被清掉，不可能再来一次），随后才写入 5。
         *   · 状态于是永久停在 5：
         *       不满足 ==6 -> 永远不进接收窗口 -> **一个包都收不到**；
         *       也不满足 ==7 -> 永远不被当成"信道空闲"收尾。
         *   现场表现就是"切到 CAD 之后 LoRa 像死了一样"。
         * 修法：先写状态（预告"CAD 进行中"），再启动 CAD —— ISR 无论多快
         *   都能命中 case 5，从而正常走到 7（无活动）或 6（有活动）。
         * ★ 只把一条赋值提前了一条指令，中断语义、CAD 参数、超时全未改。 */
        Lora_Rx_state = 5;       /* 先预告：CAD 进行中（必须放在 SetCad 之前） */
        SX126X_CAD_Sample();     //启动CAD一次
    }
    break;
		case 2://
		
			break;
    default:
        break;
    }
}
void CAD_MODE(void)
{
//  	printf("CAD_MODE1\r\n");
    SX126X_WORInit();
//	printf("CAD_MODE2\r\n");
    SX126X_WOR_Execute(Sx1268_CAD);
//		printf("CAD_MODE3\r\n");
}

/**
  * @简介：WOR到RX
  * @参数：无
  * @返回值：无
  */
void SX126X_WOR_Exit(void)
{
	SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
	SX126xSetStandby(STDBY_RC);//设置模式STDBY_RC
	
	SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
	SX126xSetRegulatorMode( USE_DCDC );//开DCDC
	SX126XWriteRxTx(0); //切换为读
	SX126xClearIrqStatus( IRQ_RADIO_ALL );
	SX126xSetDioIrqParams( IRQ_RX_DONE,   //配置映射脚来映射中断
                                IRQ_RX_DONE,
                                IRQ_RADIO_NONE,
                                IRQ_RADIO_NONE );                                 //打开RX、CRC中断，映射RX中断到DIO1；
	SX126xSetStopRxTimerOnPreambleDetect( 0 ); //检测到同步字或包头时才停止超时时钟
	SX126xSetLoRaSymbNumTimeout(0);
	G_LoRaConfig.PreambleLength=0xffff;
//	SX126xSetLoraModulationParams( );   //SF、BW、CR、LOWDATARATEOPTIMIZE
	SX126xSetLoraPacketParams( );  				//PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
	SX126xSetBufferBaseAddress( 0x00, 0x00 ); //设置读写存放基地址
//	SX126xSetRx( 0x004B00 );		// Rx 300mS  命令将芯片设置在接收模式
	/* ★ V1_0_10 更正（DS_SX1268_V1.1 Table 13-9 p.68）：
	   0xFFFFFF 不是 "Rx 350ms"，而是 **Rx Continuous = 无限接收**（射频侧永不超时；
	   350ms 应该是 0x005780）。所以这条路径上：
	     **主机侧的 Time_RX_lora 是唯一的收包窗口上限** —— 它必须覆盖
	     "剩余前导 + 头 + 载荷"的最坏情况（推导见 LoraPro.c 的 Time_RX_lora）。 */
	SX126xSetRx( 0xffffff );		// Rx Continuous（无限接收，见上）
	Lora_Rx_state=3;
	Lo_data.RfRxTick = Rt_tick;

//        Sx1268.RfRxTick = Sys_Tick;
	
}

//180ms rx超时中断
void SX126X_WOR_Exit_ms(void)
{
	SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
	SX126xSetStandby(STDBY_RC);//设置模式STDBY_RC
	
	SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
	SX126xSetRegulatorMode( USE_DCDC );//开DCDC
	SX126XWriteRxTx(0); //切换为读
	SX126xClearIrqStatus( IRQ_RADIO_ALL );
	SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_RX_TX_TIMEOUT,   //配置映射脚来映射中断
                                IRQ_RX_DONE,
                                IRQ_RADIO_NONE,
                                IRQ_RX_TX_TIMEOUT );                                 //打开RX、CRC中断，映射RX中断到DIO1；
	SX126xSetStopRxTimerOnPreambleDetect( 0 ); //检测到同步字或包头时才停止超时时钟
	SX126xSetLoRaSymbNumTimeout(0);
	G_LoRaConfig.PreambleLength=0xffff;
//	SX126xSetLoraModulationParams( );   //SF、BW、CR、LOWDATARATEOPTIMIZE
	SX126xSetLoraPacketParams( );  				//PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
	SX126xSetBufferBaseAddress( 0x00, 0x00 ); //设置读写存放基地址
	SX126xSetRx( 0xffffff );		// Rx Continuous（无限接收，见上）
//	SX126xSetRx( 0xffffff );								// Rx 350mS  命令将芯片设置在接收模式
	Lora_Rx_state=3;
	Lo_data.RfRxTick = Rt_tick;

//        Sx1268.RfRxTick = Sys_Tick;
	
}
/**
  * @简介：SX126X发送唤醒包
  * @参数：uint8_t*data：发送数据指针,前导长度
  * @返回值：无
  */
void SX126X_Awake(uint8_t*cbuf, uint16_t Preamble_Length)
{
	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
	SX126xCalibrate_err();
	SX126xSetStandby(STDBY_RC);//设置运行在哪一个时钟的模式
	SX126xSetRegulatorMode( USE_DCDC );//regModeParam  0: 芯片在所有模式下只使用LDO 1: STBY_XOSC、FS、RX 和 TX 模式下使用 DC_DC+LDO


	SX126XWriteRxTx(1); //控制引脚切换为TX
	SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
	SX126xSetDioIrqParams( IRQ_TX_DONE, //设置 IRQ 标志位
                                IRQ_TX_DONE ,
                                IRQ_RADIO_NONE,
                                IRQ_RADIO_NONE );
        //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
	G_LoRaConfig.PreambleLength=Preamble_Length;
	SX126xSetLoraPacketParams( );//PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
	SX126xSetBufferBaseAddress( 0x00, 0x00 );
//			hex_p(cbuf,12);
	SX126xSendPayload( cbuf, G_LoRaConfig.PayloadLength, 0xffffff );  //350mS  //262秒
	Lora_Rx_state=1;
	/* ★★★ V1_0_2 修正：这里原来是 do { HAL_Delay(1); } while(Lora_Rx_state == 1);
	   —— 同样**没有超时、没有 re_iwdg()**。而上面 SX126xSendPayload 的第 3 个
	   参数是射频侧 Tx 超时 0xffffff × 15.625us ≈ **262 秒**，也就是说
	   "DIO1 一直不来"时这个循环能空转 262 s -> 主循环停转 + 看门狗复位。
	   （HAL_Delay(1) 依赖 SysTick，还是忙等，按键 EXTI 也救不了它。）
	   现在：用 Rt_tick 做上限 + 每轮喂狗，超时收尾成 Lora_Rx_state = 0。 */
	{
		uint32_t t0 = Rt_tick;
		/* ★★ 看门狗审查补丁（第三轮）：第二计时基准（与 LPTIM1 无关）。
		   万一 Rt_tick 冻住，Rt_tick 那条超时永远不成立而本循环每轮都在
		   喂狗 -> "喂着狗死等"的永久死锁。本循环体内没有 STOP（只有
		   HAL_Delay(1)），所以 HAL_GetTick 这一条是完全准确的。 */
		uint32_t h0 = HAL_GetTick();
		do
		{
			/* ★★★ V1_0_11：同上——等 TxDone 的这段时间里 USART1 不能没人管。
			   本循环是"唤醒包 + 数据包"的发射等待（SX126X_Awake），
			   前导最长达 2000 符号 = 128.5ms，超过 128B 缓冲的 11ms 上限。 */
			sensor_rx_task();
			HAL_Delay(1);
			re_iwdg();
			if (((uint32_t)(Rt_tick - t0) > ((uint32_t)LORA_TX_WAIT_TIMEOUT_SEC * 10U)) ||
			    ((uint32_t)(HAL_GetTick() - h0) > ((uint32_t)LORA_TX_WAIT_TIMEOUT_SEC * 1000U)))
			{
				Lora_TxWaitTimeoutCnt++;
				Lora_Rx_state = 0;
				break;
			}
		}
		while(Lora_Rx_state == 1);
	}
//   while( (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)&j-- );
//	 printf("Lora_Data_state==%d\r\n",Lora_Rx_state);
//	RfStatus = RfTxing;
//        FeedWDG;
//        do        //数据发送不成功则直接重启
//        {
//          LPM3;
//        }while(RfStatus == RfTxing);
//        FeedWDG;
}

//#pragma vector = PORT2_VECTOR
//__interrupt void plint2(void)
//{
//  if(P2IFG & 0x01)   //P2.0 --- DIO1
//  {
//    if(RfStatus == RfRxing)
//      RfStatus = RfRxOver;
//    if(RfStatus == RfTxing)
//      RfStatus = RfTxOver;
//  }
//  // 如果监测到信号，则timeout和detect同时产生，故先执行timeout判断
//  if(P2IFG & 0x02)   //P2.1 --- DIO2
//    if(RfStatus == Cad)
//      RfStatus = CadTimeOut;
//  if(P2IFG & 0x04)   //P2.2 --- DIO3
//    RfStatus = CadDetected;
//  if(P2IFG & 0x07)
//    SX126xClearIrqStatus( IRQ_RADIO_ALL );                                        //clear flags
//  P2IFG = 0;
//  LPM3_EXIT;
//}

//#pragma vector = PORT1_VECTOR
//__interrupt void plint1(void)
//{
//  P1IFG = 0;
//  LPM3_EXIT;
//}


/* Capture only a currently awake radio. Do not wake/clear IRQ/change modes for
 * diagnostics. IRQ counters are individual aligned ISR snapshots, not a single
 * atomic capture of all hardware signals. */
static uint8_t radio_diag_can_read(void)
{
    return ((OperatingMode != MODE_SLEEP) && (OperatingMode != MODE_UNKNOWN) &&
            (OperatingMode != MODE_RX_DC) && (LORA_BUSY_VAL == 0U)) ? 1U : 0U;
}
static uint8_t radio_diag_read_ok(uint32_t spi_before, uint16_t busy_before)
{
    return ((spi_before == g_radio_spi_fail_count) &&
            (busy_before == SX126x_BusyTimeoutCnt) && (LORA_BUSY_VAL == 0U)) ? 1U : 0U;
}
void SX126xCaptureDiagnostic(RadioDiagSnapshot *s)
{
    uint32_t spi_before;
    uint16_t busy_before;
    if (s == NULL) { return; }
    memset(s, 0, sizeof(*s));
    s->rt = Rt_tick; s->sw_mode = (uint8_t)OperatingMode;
    s->pins = (uint8_t)((LORA_BUSY_VAL != 0U ? 1U : 0U) |
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) != GPIO_PIN_RESET ? 2U : 0U) |
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) != GPIO_PIN_RESET ? 4U : 0U) |
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) != GPIO_PIN_RESET ? 8U : 0U) |
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) != GPIO_PIN_RESET ? 16U : 0U) |
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) != GPIO_PIN_RESET ? 32U : 0U));
    s->exti_mask = (uint16_t)(EXTI->IMR & 0x31U);
    s->exti_pending = (uint16_t)(EXTI->PR & 0x31U);
    if (radio_diag_can_read() != 0U) {
        spi_before = g_radio_spi_fail_count; busy_before = SX126x_BusyTimeoutCnt;
        s->irq = SX126xGetIrqStatus();
        if (radio_diag_read_ok(spi_before, busy_before) != 0U) { s->read_flags |= 2U; }
    }
    if (radio_diag_can_read() != 0U) {
        spi_before = g_radio_spi_fail_count; busy_before = SX126x_BusyTimeoutCnt;
        s->status = SX126xGetStatus().Value;
        if (radio_diag_read_ok(spi_before, busy_before) != 0U) { s->read_flags |= 1U; }
    }
    if (radio_diag_can_read() != 0U) {
        spi_before = g_radio_spi_fail_count; busy_before = SX126x_BusyTimeoutCnt;
        s->errors = SX126xGetDeviceErrors().Value;
        if (radio_diag_read_ok(spi_before, busy_before) != 0U) { s->read_flags |= 4U; }
    }
    s->busy_to = SX126x_BusyTimeoutCnt; s->standby_to = SX126x_StandbyTimeoutCnt;
    s->tx_to = Lora_TxWaitTimeoutCnt; s->spi_fail = g_radio_spi_fail_count;
    s->dio1 = g_radio_diag_irq.dio1; s->dio2 = g_radio_diag_irq.dio2;
    s->dio3 = g_radio_diag_irq.dio3; s->cad_done = g_radio_diag_irq.cad_done;
    s->cad_activity = g_radio_diag_irq.cad_activity; s->rx_irq = g_radio_diag_irq.rx_irq;
}
