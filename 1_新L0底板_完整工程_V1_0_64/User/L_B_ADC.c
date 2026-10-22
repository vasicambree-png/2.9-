#include "L_B_ADC.h"
#include "adc.h"
#include "usart.h"
#include "iwdg.h"       /* ★★ 看门狗审查补丁：下面要喂狗 */
extern ADC_HandleTypeDef hadc;
uint16_t adc_buffer[2]={0};
float Light_value=0;
uint16_t Vdd_value=330;
uint8_t DMA_ADC_Start_flag=0;
extern uint16_t VREFCAL ;

/* ==========================================================================
 * ★★ 看门狗审查补丁（V1_0_2 第三轮）：ADC 这条路径原来能把看门狗喂不上
 * --------------------------------------------------------------------------
 * 原实现（get_vdd_adc_value_sleep）：
 *     MX_ADC_Init();               // 内含 HAL_ADC_Init + ADC 校准
 *     HAL_Delay(50);               // 50ms 不喂狗
 *     for (8 次) HAL_ADC_PollForConversion(&hadc, 50);   // 单次上限 50ms -> 最坏 400ms
 * 合计最坏约 490ms **全程不喂狗**。而且它被两条路都调用：
 *     · Time_1s()            —— 每 10 次（10 秒）一次；
 *     · Time_check_task()    —— 每 60 秒一次的 VDD 读取（LoraPro.c）。
 * 这两条有可能**落在同一次主循环里**，那就是约 980ms 不喂狗；
 * 而 IWDG 只有约 2.08s（LSI 波动时下限约 1.995s）—— 余量不到 2 倍。
 * 收紧三处：
 *   ① 单次轮询上限 50ms -> VDD_ADC_POLL_MS(5ms)：一次转换只要约 22us，
 *      5ms 已是 200 倍余量，没必要给 50ms；
 *   ② 每次轮询后喂一次狗；
 *   ③ 轮询失败立刻跳出（不再把 8 次超时都花完），并保留上次电压值，
 *      避免样本太少时下面 sum/6 出错。
 * ========================================================================== */
#define VDD_ADC_POLL_MS   5U

uint16_t ADC_DATA_pro(void)
{
	  float adc_num2=0;
	 	adc_num2=( adc_buffer[0]* 3300.0) / 0xFFF0;
	  adc_num2=(float)(3*(float)VREFCAL/(float)adc_buffer[0]);
    Vdd_value=adc_num2*100;
	printf("Vdd_value:%d;\r\n",Vdd_value);
	return Vdd_value;
}

void Vdd_Light_value(void)
{
}

void ADC_data_check(void)
{
}

uint16_t vdd_buf[9]={0};

uint16_t  get_vdd_adc_value(void)
{
    uint32_t sum = 0;
    uint16_t max = 0, min = 0xFFFF;
    uint16_t val;

    HAL_ADC_Start(&hadc);
    for (int i = 0; i < 8; i++) {
        HAL_ADC_PollForConversion(&hadc, 50);
        val = HAL_ADC_GetValue(&hadc);
        sum += val;
        if (val > max) max = val;
        if (val < min) min = val;
    }
    HAL_ADC_Stop(&hadc);

    sum = sum - max - min;
    uint16_t avg = sum / 6;

    float adc_num2 = (3.0f * VREFCAL) / avg;
    Vdd_value = (uint16_t)(adc_num2 * 100);

    printf("adc == %d; Vdd_value:%d;\r\n", avg, Vdd_value);
    return Vdd_value;
}

uint16_t get_vdd_adc_value_sleep(void)
{
    uint32_t sum = 0;
    uint16_t max = 0, min = 0xFFFF;
    uint16_t val;
    uint8_t  n = 0U;                 /* ★ 有效采样个数 */

    MX_ADC_Init();
	  HAL_Delay(50);
    re_iwdg();                       /* ★ 上面这 50ms 不喂狗，补一口 */

    HAL_ADC_Start(&hadc);
    for (int i = 0; i < 8; i++) {
        /* ★ 轮询上限 50ms -> 5ms，并且失败就跳出，不把 8 次超时全花完 */
        if (HAL_ADC_PollForConversion(&hadc, VDD_ADC_POLL_MS) != HAL_OK) {
            break;
        }
        re_iwdg();                   /* ★ 每轮喂一口 */
        val = HAL_ADC_GetValue(&hadc);
        sum += val;
        if (val > max) max = val;
        if (val < min) min = val;
        n++;
    }
    HAL_ADC_Stop(&hadc);
    HAL_ADC_DeInit(&hadc);
		HAL_ADC_MspDeInit(&hadc);

    /* ★ 采样太少（ADC 异常）：保留上一次的 Vdd_value，直接返回。
       既避免 sum/6 用到不足 6 个样本，也避免把 VREFCAL/avg 除出个野值
       —— Vdd_value 会通过 0x01 帧显示到屏上，野值比"旧值"更糟。 */
    if (n < 8U)
    {
        return Vdd_value;
    }

    sum = sum - max - min;
    uint16_t avg = sum / 6;

    float adc_num2 = (3.0f * VREFCAL) / avg;
    Vdd_value = (uint16_t)(adc_num2 * 100);

    return Vdd_value;
}
