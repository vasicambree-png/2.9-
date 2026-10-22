////#include "Public.h"

///*

//P2.0    DIO1
//P2.1    DIO2
//P2.2    DIO3

//P4.0    CTL1
//P4.1    CTL2
//P4.2    BUSY
//P4.3    RST

//*/

//void Lora_Gpio_Init(void)
//{
//  GPIO_SetAsInput(GPIO2, P0 | P1 | P2); //P2.0 1 2   ‰»Î
//  
//  P2IE  |=  0x07;
//  P2IES &= ~0x07;
//  P2IFG &= ~0x07;
//  
//  GPIO_SetAsInput(GPIO4, P2);           //P4.2   ‰»Î
//  
//  GPIO_SetAsOutput(GPIO4, P0 | P1 | P3);//P4.0 1 3   ‰≥ˆ
//  
//  LORA_NSS_H;
//  LORA_SW1_L;
//  LORA_SW2_L;
//  LORA_RST_H;

//  Spi_Init();
//}

//u8 SX126X_ReadWriteByte(u8 Data)
//{
//  return Spi_WRByte(Data);
//}

//void SX126xWaitOnBusy( void )
//{
//  while(LORA_BUSY_VAL == 1 );
//}

