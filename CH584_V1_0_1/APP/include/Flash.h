#ifndef __FLASH_H_
#define __FLASH_H_

#include "CH58x_common.h"

typedef struct{
    uint8_t MAC[6];
}ble_scan;

typedef struct
{
    uint8_t Flash_ID;
    uint8_t sacn_num;
    ble_scan ble_mac[22];

}save;
extern save Flash_save;
void Start_read_Flash(void);
void write_flash(void);
void factory_data_reset(void);
void  write_sacn_ble_flas(uint8_t *data_buf,uint16_t data_len);
void  write_sacn_ble(void);
void Start_read_Flash1(void);
void Frist_all_data(uint8_t  bd_num);
void flash_all_zero(void);

/* ======================================================================
 *  绑定记录持久化（任务 C/D/E）—— 新实现，与上面那套旧函数无关、互不复用
 *
 *  存储：Data-Flash 偏移 0 起的块 0（4096 B），绝不动 0x7000（BLE SNV）。
 *  镜像：magic 'CHBD' + 版本 + 条数 + uint16 校验和 + 40 字节定长记录 × N
 *        （总长 8 + 40*32 = 1288 字节）
 *  擦除：EEPROM_ERASE(0, 4096) —— 长度必须是 4096 的倍数，否则芯片 while(1)
 *  缓冲：__attribute__((aligned(4))) 静态 RAM 数组
 * ==================================================================== */

/* 0x05 保存 / 0x04 解绑的应答状态码（与协议契约_0x04解绑_0x05保存_规格冻结.md 一致） */
#define BIND_STORE_STATUS_OK      0x00u   /* 成功（0x05: 已写入并回读校验通过） */
#define BIND_STORE_STATUS_EMPTY   0x01u   /* 当前没有绑定，无内容可存 */
#define BIND_STORE_STATUS_FAIL    0x02u   /* 擦/写失败或回读校验不一致 */

/* 把 RAM 绑定表持久化到 Data-Flash，含回读校验。返回上面三个状态码之一 */
uint8_t binding_store_save(void);
/* 开机从 Data-Flash 恢复绑定表 + 通道占用；校验失败安全回退为无绑定。0=恢复了至少一条 / -1=无有效记录 */
int     binding_store_load(void);
/* 清除 Flash 中已保存的绑定记录（整块擦除）。返回 BIND_STORE_STATUS_OK / _FAIL */
uint8_t binding_store_clear(void);

/* Independent Data-Flash block 0x1000. 0=readback-verified/valid, 1=failure.
 * Main-loop/startup only. Invalid records load as OFF; unchanged saves skip I/O. */
uint8_t power_state_store_load(uint8_t *state);
uint8_t power_state_store_save(uint8_t state);

#endif
