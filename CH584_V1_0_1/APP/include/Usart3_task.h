#ifndef USART3_TASK_H
#define USART3_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "yuying_TFT.h"
uint8_t handle_command(uint8_t command, uint8_t *data, uint16_t data_len) ;
void Usart3_Init(void);
void app_uart_process(void);
void usart_task(void);
void screen_reset_display_init(uint8_t reset_enabled, uint8_t reset_cause);
void send_response_frame(uint8_t response_cmd, uint8_t *data, uint16_t data_len );
/* cmd 0x03 传感器数据读取应答端（契约：协议_0x03_传感器数据读取_规格冻结.md）
 * cmd 0x06「一轮采集完成主动上报」载荷与 0x03 完全同构，故组包函数带命令字参数 */
uint16_t build_sensor_data_response(uint8_t cmd);
void send_sensor_data_response(void);
void send_round_report(void);
void  ble_reve_data(uint8_t *data_buf,uint16_t data_len);
uint8_t parse_received_frame(uint8_t *rx_buffer, uint16_t data_len, data_LIST *pData);
void hex_p(uint8_t *data,uint16_t data_len);
void uart_send_process(void);
void end_send_data(uint8_t cnt);
#ifdef __cplusplus
}
#endif

#endif
