#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32l0xx_hal.h"
#define FLASH_USER_START_ADDR 0x0800C000  // FLASH起始地址，可根据实际调整
#define PARAM_COUNT 8  // 总共要写入的参数个数
#define CACHE_SIZE  0x80  // STM32L051R8T6的FLASH页大小，单位：字节，需根据手册确认

// 待写入的参数结构体
typedef struct {
    uint32_t param1;
    uint32_t param2;
    uint32_t param3;
    uint32_t param4;
    float param5;
    float param6;
    float param7;
    float param8;
} CalibrationParams;




typedef struct
{
    uint8_t Flash_ID;
    uint8_t Version;//版本号
	  uint8_t T_number;
	  uint8_t Z_number;
	  uint8_t biaoding_vlaue1[2];
	  uint8_t biaoding_vlaue2[2];
	  uint8_t biaoding_vlaue3[2];
	  uint8_t biaoding_vlaue4[2];
	  

}save;

extern save Flash_save;
void Start_read_EEPROM(void);
void Flash_text(void);
void factory_reset(void);
void GetNewsend_addr(void);
void Save_data(void);
void Quick_BOR_Test(void);
#endif

