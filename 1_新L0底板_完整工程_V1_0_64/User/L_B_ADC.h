#ifndef	__L_B_ADC_H__
#define __L_B_ADC_H__


#include "main.h"

//// 宏定义总采集次数和去头尾次数(偶数)
//#define TOTAL_SAMPLES   5          // 总采集次数
//#define TRIM_COUNT      1          // 前后各去除的次数
//#define CHANNEL_COUNT   2          //通道数


//typedef struct {
//	uint16_t setting_count;
//	uint16_t old_value_count;
//	uint16_t value_count;
//	uint16_t state;
//}ADCcnt;

//typedef struct {
//    uint16_t buf[TOTAL_SAMPLES];
//} ChannelData;

uint8_t read_adc_setting(uint16_t number);
void ADC_data_check(void);
uint16_t ADC_DATA_pro(void);
void Vdd_Light_value(void);
uint16_t  get_vdd_adc_value(void);
uint16_t get_vdd_adc_value_sleep(void);
#endif


