#ifndef _SX126X_GPIO_H
#define _SX126X_GPIO_H

#include "Cpu.h"

#define LORA_NSS_H      GPIO_SetBit(GPIO3, P0)
#define LORA_NSS_L      GPIO_ResetBit(GPIO3, P0)

#define LORA_SW1_H     GPIO_SetBit(GPIO4, P0)
#define LORA_SW1_L     GPIO_ResetBit(GPIO4, P0)

#define LORA_SW2_H     GPIO_SetBit(GPIO4, P1)
#define LORA_SW2_L     GPIO_ResetBit(GPIO4, P1)

#define LORA_RST_H      GPIO_SetBit(GPIO4, P3)
#define LORA_RST_L      GPIO_ResetBit(GPIO4, P3)

#define LORA_DIO_VAL   GPIO_ReadInputBit(GPIO2, P0)

#define LORA_BUSY_VAL   GPIO_ReadInputBit(GPIO4, P2)

void Lora_Gpio_Init(void);
u8 SX126X_ReadWriteByte(u8 Data);
void SX126xWaitOnBusy( void );

#endif
