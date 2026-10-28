#include "Flash.h"
#include "string.h"
#include "HAL.h"
#include "observer.h"
#include "Usart3_task.h"
#include "observer.h"
#define ID 0x7A
extern volatile uint8_t SW_SCAN_STATE;
save Flash_save;
extern uint8_t All_dat_flag;
// ��֤����һ����
__HIGH_CODE
uint8_t Verify_FLASH(uint8_t *write_data, uint8_t *read_data, uint32_t data_len) {
    for (uint32_t i = 0; i < data_len; i++) {
        if (write_data[i] != read_data[i]) {
            return 0; // ���ݲ�һ��
        }
    }
    return 1; // ����һ��
}

extern device_info_t  device_list[MAX_DEVICES];

void factory_data_reset(void)
{
      uint8_t Bind_device1_name[28]={0};
      uint8_t read_flash_buf[200]={0};
      int i=0;
      uint8_t  s;

      tmos_memset(read_flash_buf,0,200);
      read_flash_buf[0]=ID;




//     tmos_memcpy(read_flash_buf,&Flash_save,sizeof(Flash_save));
      s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE);

      s = EEPROM_WRITE(0, read_flash_buf, 200);
      tmos_memset(read_flash_buf,0,200);

      EEPROM_READ(0, read_flash_buf, 200);
      tmos_memcpy(&Flash_save,read_flash_buf,sizeof(Flash_save));

}
__HIGH_CODE
void Start_read_Flash(void)
{
    uint8_t read_flash_buf[200]={0};
     uint8_t  s;
     uint8_t  mac[6]={0};
     int i=0,j=0;
     uint8_t flag=0;
     uint8_t  count=0;
    tmos_memset(read_flash_buf,0,200);
    s= EEPROM_READ(0, read_flash_buf, 200);

//    hex_p(read_flash_buf,100);
    if(read_flash_buf[0]!=ID)//
    {
        tmos_memset(read_flash_buf,0,200);
        read_flash_buf[0]=ID;

        s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //����
        s = EEPROM_WRITE(0, read_flash_buf, 200); //д
        tmos_memset(read_flash_buf, 0, 200);
        EEPROM_READ(0, read_flash_buf, 200); //�ٶ�
//        PRINT(" sizeof(Flash_save)== %d \r\n", sizeof(Flash_save));
       // tmos_memcpy(&Flash_save,read_flash_buf,sizeof(Flash_save));
        All_dat_flag=1;
    }
    else
    {
      tmos_memset(read_flash_buf,0,200);
      EEPROM_READ(0, read_flash_buf, 200);//��
      //tmos_memcpy(&Flash_save,read_flash_buf,sizeof(Flash_save));
//      PRINT(" sizeof(Flash_save)== %d \r\n", sizeof(Flash_save));
//      PRINT("  Flash_save.Sleep_mode== %d \r\n", Flash_save.Sleep_mode);
      for ( i = 0;  i < 20;  i++)
      {

//          tmos_memcpy(mac, &read_flash_buf[2+i*6], 6);
//          flag=0;
//          for(j=0;j<6;j++)
//          {
//           if(mac[j]!=0)
//           {
//               flag=1;
//           }
//          }
//          if(flag==1)
//          {
//             tmos_memcpy(device_list[i].mac, mac, 6);
//             device_list[i].valid = 1;
//             count++;
//             PRINT("������豸[%d/%d]��MAC: ", i, count);
//             hex_p(mac, 6);
//          }
    }





      SW_SCAN_STATE=3;
//      PRINT("  Flash_save.brightness_level1== %d \r\n", Flash_save.brightness_level1);
    }
    //����ʱ�����ֵ


}
uint8_t  ble_count=0;
 uint8_t already_printed = 0;  // ��ֹ�ظ���ӡ

__HIGH_CODE
void Frist_all_data(uint8_t  bd_num)
{
//    uint8_t cnt=0;
//    cnt = bd_num;
//    if (already_printed||cnt==0) {
//           return;  // �Ѿ���ӡ�������ټ��
//       }
//      uint8_t all_ready = 1;
//       for (int i = 0; i < cnt; i++) {
//           if (device_list[i].valid) {
//               // Ҫ�����ƺ����������ݾ����յ�
//               if (device_list[i].data_re_flag==0)
//               {
//                   all_ready = 0;
//                   break;
//               }
//           }
//       }
//
//       if (all_ready) {
//           end_send_data(bd_num); //һ���Է�������
////           if()
////           GAPRole_CentralCancelDiscovery(); //ֹͣɨ��
//           PRINT("All  successful\r\n");
//           already_printed = 1;
//       }
}



__HIGH_CODE
void Start_read_Flash1(void)
{
    uint8_t read_flash_buf[400]={0};
    uint8_t  s;
    uint8_t  mac[6]={0};
    int i=0,j=0;
    uint8_t flag=0;

    tmos_memset(read_flash_buf,0,400);
    s= EEPROM_READ(0, read_flash_buf, 400);

//    hex_p(read_flash_buf,100);
    if(read_flash_buf[0]!=ID)//����ǵ�һ��дFLASH
    {
        tmos_memset(read_flash_buf,0,400);
        read_flash_buf[0]=ID;

        s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //����
        s = EEPROM_WRITE(0, read_flash_buf, 400); //д
        tmos_memset(read_flash_buf, 0, 400);
        EEPROM_READ(0, read_flash_buf, 400); //�ٶ�
        PRINT(" sizeof(Flash_save)== %d \r\n", sizeof(Flash_save));
       // tmos_memcpy(&Flash_save,read_flash_buf,sizeof(Flash_save));
        All_dat_flag=1;
    }
    else
    {
      tmos_memset(read_flash_buf,0,400);
      EEPROM_READ(0, read_flash_buf, 400);//��

      for (i = 0; i < 40; i++)
      {
          tmos_memcpy(mac, &read_flash_buf[2 + i * 6], 6);

          uint8_t all_zero = 1;   //
          uint8_t all_ff   = 1;   //

          for (j = 0; j < 6; j++) {
              if (mac[j] != 0)
              {
                  all_zero = 0;   //
              }
              if (mac[j] != 0xFF)
              {
                  all_ff   = 0;   //
              }
          }
          //
          if (!(all_zero || all_ff))
          {
              tmos_memcpy(device_list[i].mac, mac, 6);
            //  device_list[i].valid = 1;
              ble_count++;
//              hash_insert(i);
//              PRINT("���������豸[%d/%d]��MAC: ", i, ble_count);
//              hex_p(mac, 6);
          }
      }
      PRINT("MAC_num == %d \r\n", ble_count);




      SW_SCAN_STATE=3;
//      PRINT("  Flash_save.brightness_level1== %d \r\n", Flash_save.brightness_level1);
    }
}
__HIGH_CODE
void write_flash(void)
{
  uint16_t tpms_high=0,tpms_low=0;
//      PRINT("sleep_mode == %d\r\n",sleep_mode);
      uint8_t write_flash_buf[400]={0};
      uint8_t s=0;
      tmos_memset(write_flash_buf,0,200);

      s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //��

      tmos_memcpy(write_flash_buf,&Flash_save,sizeof(Flash_save));
      s = EEPROM_WRITE(0, write_flash_buf, 200);
      PRINT("Write_flash_one...\r\n");
      if( s!=0)
      {
        PRINT("Write_flash_two...\r\n");

        s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //��
        s = EEPROM_WRITE(0, write_flash_buf, 200);
      }

      PRINT("Write_flash_END!!!!!!!!!...\r\n");
}


__HIGH_CODE
void  write_sacn_ble_flas(uint8_t *data_buf,uint16_t data_len)
{
   uint8_t read_buf[200]={0};
   uint8_t write_flash_buf[200]={0};
   uint8_t index=0;
   uint8_t s=0;
   uint8_t flag=0;
   uint8_t i=0,j;
   uint8_t mac[6];
  if(data_len<=200)
  {
    index=1;
    tmos_memset(write_flash_buf, 0, 200);
    write_flash_buf[0]=ID;
    write_flash_buf[1]=data_buf[0];

    tmos_memcpy(&write_flash_buf[2],&data_buf[1],data_len-1);
    s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //��
    s = EEPROM_WRITE(0, write_flash_buf, 200);

    PRINT("Write_flash_one...\r\n");
    if( s!=0)
    {
        PRINT("Write_flash_two...\r\n");

        s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //��
        s = EEPROM_WRITE(0, write_flash_buf, 200);
        EEPROM_READ(0, read_buf, 200);
        if(Verify_FLASH(write_flash_buf,read_buf,200)==1)
        {

          //  init_device_list();
            for( i=0;i<20;i++)
            {
               tmos_memcpy(mac,&read_buf[2+i*6],6);
               tmos_memcpy(device_list[i].mac, mac, 6);
           //    device_list[i].mfg_received = 0;
             //  device_list[i].mfg_len = 0;
               flag=0;
               for( j=0;j<6;j++)
               {
                  if(mac[j]!=0)
                  {
                      flag=1;
                  }
               }
               if(flag==1)
               {
                 // device_list[i].valid = 1;
               }
           }
        }
    }
    else
    {
       EEPROM_READ(0, read_buf, 200);
       if(Verify_FLASH(write_flash_buf,read_buf,200)==1)
       {
         //  init_device_list();
           for( i=0;i<20;i++)
           {
               tmos_memcpy(mac,&read_buf[2+i*6],6);
               tmos_memcpy(device_list[i].mac, mac, 6);

               flag=0;
               for( j=0;j<6;j++)
               {
                  if(mac[j]!=0)
                  {
                      flag=1;
                  }
               }
               if(flag==1)
               {
                //  device_list[i].valid = 1;
               }
           }
       }
    }

    PRINT("Write_flash_END!!!!!!!!!...\r\n");
  }
}
__HIGH_CODE
void  write_sacn_ble(void)
{

       uint8_t read_buf[400]={0};
       uint8_t write_flash_buf[400]={0};
       uint8_t index=0;
       uint8_t s=0;
       uint8_t flag=0;
       uint8_t i=0,j;
       uint8_t mac[6];

       index=1;
       tmos_memset(write_flash_buf, 0, 200);
       write_flash_buf[0]=ID;
       write_flash_buf[1]=40;
        for( i=0;i<40;i++)
        {
          tmos_memcpy(mac,device_list[i].mac, 6);
          tmos_memcpy(&write_flash_buf[2+i*6],mac,6);
        }
        s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //��
        s = EEPROM_WRITE(0, write_flash_buf, 400);

        PRINT("Write_flash_one...\r\n");
        if( s!=0)
        {
            PRINT("Write_flash_two...\r\n");

            s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //��
            s = EEPROM_WRITE(0, write_flash_buf, 400);
            EEPROM_READ(0, read_buf, 400);
            if(Verify_FLASH(write_flash_buf,read_buf,400)==1)
            {
                PRINT("Write_flash_two success\r\n");
            }
        }
        else
        {
            EEPROM_READ(0, read_buf, 400);
            if(Verify_FLASH(write_flash_buf,read_buf,400)==1)
            {
                PRINT("Write_flash_one success\r\n");
            }
        }

        PRINT("Write_flash_END!!!!!!!!!...\r\n");

}

void flash_all_zero(void)
{
    uint8_t s;
    uint8_t write_buf[400]={0};
    tmos_memset(write_buf,0,400);
    write_buf[0]=ID;

     s = EEPROM_ERASE(0, EEPROM_BLOCK_SIZE); //����
     s = EEPROM_WRITE(0, write_buf, 400); //д
     tmos_memset(write_buf, 0, 400);
     EEPROM_READ(0, write_buf, 400); //�ٶ�
   //  PRINT(" sizeof(Flash_save)== %d \r\n", sizeof(Flash_save));
    // tmos_memcpy(&Flash_save,read_flash_buf,sizeof(Flash_save));
     All_dat_flag=1;
   PRINT("flash_all_zero\r\n");
}

/* ======================================================================
 *        绑定记录持久化（0x05 保存 / 0x04 解绑 / 开机自动加载）
 *        任务 C / D / E —— 新写、干净、自包含的实现
 *
 *  ★ 本节与文件上面那套旧函数（write_sacn_ble / write_flash /
 *    write_sacn_ble_flas / Start_read_Flash / Start_read_Flash1 /
 *    factory_data_reset / Frist_all_data / flash_all_zero）**没有任何关系**，
 *    也**一个都不复用**。原因（见 _docs 审核报告 M5 / B4）：
 *      - 旧代码把 device_list[i]（MAX_DEVICES 只有 2）按 i < 40 循环读写，
 *        是**越界读写**；
 *      - 旧代码的存储布局（ID + 裸 6 字节 MAC 数组）与现在的绑定表
 *        （MAC + 名字 + 类型 + 主机号 + 起始通道 + 通道数）完全不匹配；
 *      - 旧代码在全工程里调用者数为 0。
 *
 *  硬性约束（写错会导致芯片死机）：
 *    1) EEPROM_ERASE(addr,len) 的 len **必须是 EEPROM_BLOCK_SIZE(4096) 的倍数**：
 *       ISP585.h:139-149 的 EEPROM_ERASE() 在 Length%4096 != 0 时直接 while(1)；
 *       本节用 BIND_STORE_ERASE_LEN = EEPROM_BLOCK_SIZE 整块擦除。
 *    2) EEPROM_WRITE / EEPROM_READ 的缓冲**必须在 RAM 且 4 字节对齐**
 *       （ISP585.h:68/125/155 "Must in RAM and be aligned to 4 bytes"）；
 *       本节的两个缓冲都是 __attribute__((aligned(4))) 静态数组。
 *    3) 擦写函数标 __HIGH_CODE（工程既有用法；EEPROM_ERASE 是 always_inline
 *       宏，会一并落进 RAM 代码段）。
 *    4) Data-Flash 地址用**偏移 0 起**：本节用块 0 = 偏移 0x0000..0x0FFF，
 *       **绝不碰 0x7000**——BLE SNV 占 0x7000~0x7100
 *       （HAL/include/CONFIG.h: BLE_SNV_ADDR = 0x77000 - FLASH_ROM_MAX_SIZE = 0x7000,
 *         BLE_SNV_BLOCK = 256, BLE_SNV_NUM = 1，由 HAL/MCU.c:134 交给 BLE 库）。
 *    5) 镜像含 magic + 版本 + 条数 + 定长记录 + 校验和；条数上限 MAX_BINDING_NUM(32)。
 *
 *  记录格式（镜像总长 = 8 + 40*32 = 1288 字节）：
 *    偏移 0   4  magic  'C' 'H' 'B' 'D'  (0x43 0x48 0x42 0x44)
 *    偏移 4   1  版本  0x01
 *    偏移 5   1  条数  count（有效镜像为 1..32）
 *    偏移 6   2  校验和 sum16（对整幅镜像求和、跳过本字段自身；低字节在前）
 *    偏移 8.. 记录区，第 i 条起始于 8 + i*40：
 *         +0   6   MAC
 *         +6  30   名字（不足补 0，含 '\0'）
 *         +36  1   类型 Sensor_Tpye
 *         +37  1   主机号 host_num
 *         +38  1   起始通道 frist_ch_num
 *         +39  1   通道数 ch_num
 *      记录区剩余部分保持全 0（不是 0xFF），使同一份绑定算出的校验和稳定。
 * ==================================================================== */

#define BIND_STORE_BASE        0x0000u                  /* Data-Flash 偏移，块 0 */
#define BIND_STORE_ERASE_LEN   EEPROM_BLOCK_SIZE        /* 4096：必须是 4096 的倍数 */
#define BIND_STORE_MAGIC0      0x43u                    /* 'C' */
#define BIND_STORE_MAGIC1      0x48u                    /* 'H' */
#define BIND_STORE_MAGIC2      0x42u                    /* 'B' */
#define BIND_STORE_MAGIC3      0x44u                    /* 'D' */
#define BIND_STORE_VERSION     0x01u
#define BIND_STORE_HDR_LEN     8u
#define BIND_REC_SIZE          40u
#define BIND_REC_MAC_OFF       0u
#define BIND_REC_NAME_OFF      6u
#define BIND_REC_TYPE_OFF      36u
#define BIND_REC_HOST_OFF      37u
#define BIND_REC_START_OFF     38u
#define BIND_REC_CHNUM_OFF     39u
#define BIND_STORE_IMAGE_LEN   (BIND_STORE_HDR_LEN + (BIND_REC_SIZE * MAX_BINDING_NUM))

/* 写缓冲 / 回读缓冲：静态 + 4 字节对齐（EEPROM_WRITE/READ 的硬性要求） */
__attribute__((aligned(4))) static uint8_t s_bind_img[BIND_STORE_IMAGE_LEN];
__attribute__((aligned(4))) static uint8_t s_bind_rd [BIND_STORE_IMAGE_LEN];

/**
 * 对整幅镜像求 16 位累加校验和（跳过偏移 6/7 的校验和字段本身）。
 */
static uint16_t bind_image_sum16(const uint8_t *p)
{
    uint16_t s = 0;
    uint16_t i;

    for (i = 0; i < BIND_STORE_IMAGE_LEN; i++) {
        if (i == 6u || i == 7u) continue;      /* 跳过校验和字段 */
        s = (uint16_t)(s + p[i]);
    }
    return s;
}

/**
 * ★ 任务 C / 0x05：把当前 RAM 绑定表持久化到 Data-Flash，并回读校验。
 *
 * @return BIND_STORE_STATUS_OK(0x00) 成功且回读一致
 *         BIND_STORE_STATUS_EMPTY(0x01) 当前没有绑定，无内容可存
 *         BIND_STORE_STATUS_FAIL(0x02) 擦/写失败或回读校验不一致
 */
__HIGH_CODE
uint8_t binding_store_save(void)
{
    uint16_t i, off, sum;
    uint8_t  cnt, ret;

    if (g_binding_count == 0) {
        PRINT("[STORE] save: no binding, nothing to save\r\n");
        return BIND_STORE_STATUS_EMPTY;
    }
    if (g_binding_count > MAX_BINDING_NUM) {          /* 理论上不可能 */
        PRINT("[STORE] save: count %d > %d, refuse\r\n",
              g_binding_count, MAX_BINDING_NUM);
        return BIND_STORE_STATUS_FAIL;
    }

    cnt = g_binding_count;

    /* ---------- 1. 组镜像 ---------- */
    tmos_memset(s_bind_img, 0, BIND_STORE_IMAGE_LEN);
    s_bind_img[0] = BIND_STORE_MAGIC0;
    s_bind_img[1] = BIND_STORE_MAGIC1;
    s_bind_img[2] = BIND_STORE_MAGIC2;
    s_bind_img[3] = BIND_STORE_MAGIC3;
    s_bind_img[4] = BIND_STORE_VERSION;
    s_bind_img[5] = cnt;
    /* 偏移 6/7 的校验和最后填 */

    off = BIND_STORE_HDR_LEN;
    for (i = 0; i < cnt; i++) {
        const scan_binding *b = &g_binding_list[i];
        uint8_t name_len = (uint8_t)tmos_strlen((char *)b->name);

        if (name_len > (MAX_NAME_LEN - 1)) name_len = (MAX_NAME_LEN - 1);

        tmos_memcpy(&s_bind_img[off + BIND_REC_MAC_OFF],  b->mac, 6);
        tmos_memcpy(&s_bind_img[off + BIND_REC_NAME_OFF], b->name, name_len);
        s_bind_img[off + BIND_REC_TYPE_OFF]  = (uint8_t)b->Type;
        s_bind_img[off + BIND_REC_HOST_OFF]  = b->host_num;
        s_bind_img[off + BIND_REC_START_OFF] = b->frist_ch_num;
        s_bind_img[off + BIND_REC_CHNUM_OFF] = b->ch_num;

        off = (uint16_t)(off + BIND_REC_SIZE);
    }

    sum = bind_image_sum16(s_bind_img);
    s_bind_img[6] = (uint8_t)(sum & 0xFF);            /* 低字节在前 */
    s_bind_img[7] = (uint8_t)(sum >> 8);

    /* ---------- 2. 整块擦除（长度 = 4096，满足 EEPROM_ERASE 约束） ---------- */
    ret = (uint8_t)EEPROM_ERASE(BIND_STORE_BASE, BIND_STORE_ERASE_LEN);
    if (ret != 0) {
        PRINT("[STORE] erase fail: %d\r\n", ret);
        return BIND_STORE_STATUS_FAIL;
    }

    /* ---------- 3. 写入（失败重试一次） ---------- */
    ret = (uint8_t)EEPROM_WRITE(BIND_STORE_BASE, s_bind_img, BIND_STORE_IMAGE_LEN);
    if (ret != 0) {
        PRINT("[STORE] write fail: %d, retry once\r\n", ret);
        ret = (uint8_t)EEPROM_ERASE(BIND_STORE_BASE, BIND_STORE_ERASE_LEN);
        if (ret == 0)
            ret = (uint8_t)EEPROM_WRITE(BIND_STORE_BASE, s_bind_img,
                                        BIND_STORE_IMAGE_LEN);
        if (ret != 0) {
            PRINT("[STORE] write fail again: %d\r\n", ret);
            return BIND_STORE_STATUS_FAIL;
        }
    }

    /* ---------- 4. 回读校验（逐字节比对） ---------- */
    tmos_memset(s_bind_rd, 0, BIND_STORE_IMAGE_LEN);
    ret = (uint8_t)EEPROM_READ(BIND_STORE_BASE, s_bind_rd, BIND_STORE_IMAGE_LEN);
    if (ret != 0) {
        PRINT("[STORE] readback fail: %d\r\n", ret);
        return BIND_STORE_STATUS_FAIL;
    }
    if (Verify_FLASH(s_bind_img, s_bind_rd, BIND_STORE_IMAGE_LEN) != 1) {
        PRINT("[STORE] readback mismatch -> save failed\r\n");
        return BIND_STORE_STATUS_FAIL;
    }

    PRINT("[STORE] save ok: %d record(s), %d bytes @0x%04X (verified)\r\n",
          cnt, BIND_STORE_IMAGE_LEN, BIND_STORE_BASE);
    return BIND_STORE_STATUS_OK;
}

/**
 * ★ 任务 D：开机从 Data-Flash 恢复绑定表。
 *
 * 流程：读镜像 → 校验 magic/版本/条数/校验和 → 逐条做字段与通道冲突校验
 *       → observer_restore_binding()（追加绑定 + 占用通道）。
 * 任何一处校验失败都**安全回退为"无绑定"**（RAM 已预先清空），
 * 绝不带着垃圾数据运行。
 *
 * @return 0 = 至少恢复成功一条；-1 = 无有效记录 / 校验失败（回退为无绑定）
 */
__HIGH_CODE
int binding_store_load(void)
{
    uint16_t sum_stored, sum_calc;
    uint8_t  i, cnt, ret;

    /* 先无条件清空 RAM 绑定状态：后面任何失败都停在"无绑定"这个安全状态 */
    (void)observer_clear_all_bindings();

    tmos_memset(s_bind_rd, 0, BIND_STORE_IMAGE_LEN);
    ret = (uint8_t)EEPROM_READ(BIND_STORE_BASE, s_bind_rd, BIND_STORE_IMAGE_LEN);
    if (ret != 0) {
        PRINT("[LOAD] flash read fail: %d -> no binding\r\n", ret);
        return -1;
    }

    /* ---------- magic / 版本 ---------- */
    if (s_bind_rd[0] != BIND_STORE_MAGIC0 || s_bind_rd[1] != BIND_STORE_MAGIC1 ||
        s_bind_rd[2] != BIND_STORE_MAGIC2 || s_bind_rd[3] != BIND_STORE_MAGIC3) {
        PRINT("[LOAD] no valid record (magic %02X%02X%02X%02X) -> no binding\r\n",
              s_bind_rd[0], s_bind_rd[1], s_bind_rd[2], s_bind_rd[3]);
        return -1;
    }
    if (s_bind_rd[4] != BIND_STORE_VERSION) {
        PRINT("[LOAD] version %d != %d -> no binding\r\n",
              s_bind_rd[4], BIND_STORE_VERSION);
        return -1;
    }

    /* ---------- 条数 ---------- */
    cnt = s_bind_rd[5];
    if (cnt == 0) {
        PRINT("[LOAD] record count = 0 -> no binding\r\n");
        return -1;
    }
    if (cnt > MAX_BINDING_NUM) {
        PRINT("[LOAD] count %d > %d -> refuse, no binding\r\n",
              cnt, MAX_BINDING_NUM);
        return -1;
    }

    /* ---------- 校验和 ---------- */
    sum_stored = (uint16_t)(s_bind_rd[6] | ((uint16_t)s_bind_rd[7] << 8));
    sum_calc   = bind_image_sum16(s_bind_rd);
    if (sum_stored != sum_calc) {
        PRINT("[LOAD] checksum mismatch: stored=0x%04X calc=0x%04X -> no binding\r\n",
              sum_stored, sum_calc);
        return -1;
    }

    /* ---------- 逐条恢复 ---------- */
    for (i = 0; i < cnt; i++) {
        uint16_t off = (uint16_t)(BIND_STORE_HDR_LEN + (uint16_t)i * BIND_REC_SIZE);
        const uint8_t *mac  = &s_bind_rd[off + BIND_REC_MAC_OFF];
        const uint8_t *nm   = &s_bind_rd[off + BIND_REC_NAME_OFF];
        uint8_t  name_len = 0;
        uint8_t  all_zero = 1, all_ff = 1, k;

        Sensor_Tpye type  = (Sensor_Tpye)s_bind_rd[off + BIND_REC_TYPE_OFF];
        uint8_t     host  = s_bind_rd[off + BIND_REC_HOST_OFF];
        uint8_t     start = s_bind_rd[off + BIND_REC_START_OFF];
        uint8_t     cnum  = s_bind_rd[off + BIND_REC_CHNUM_OFF];

        for (k = 0; k < 6; k++) {
            if (mac[k] != 0x00) all_zero = 0;
            if (mac[k] != 0xFF) all_ff   = 0;
        }
        if (all_zero || all_ff) {
            PRINT("[LOAD] rec[%d] MAC invalid -> skip\r\n", i);
            continue;
        }

        /* 名字长度：在 30 字节字段内找 '\0'，最多 29 个字符 */
        while (name_len < (MAX_NAME_LEN - 1) && nm[name_len] != '\0') name_len++;

        if (observer_restore_binding(mac, (const char *)nm, name_len,
                                     type, host, start, cnum) != 0) {
            PRINT("[LOAD] rec[%d] rejected (bad field or channel conflict) -> skip\r\n", i);
        }
    }

    PRINT("[LOAD] restored %d/%d binding(s), used %d/%d channel(s)\r\n",
          g_binding_count, cnt, count_used_channels(), MAX_CH_NUM);

    return (g_binding_count > 0) ? 0 : -1;
}

/**
 * ★ 任务 B（Flash 侧）：清除已保存的绑定记录（0x04 解绑全部时调用）。
 *
 * 整块擦除 → 全 0xFF → 下次开机 magic 校验必然失败 → 回退为"无绑定"。
 * 这样解绑才真正生效（否则重启后记录又被加载回来）。
 *
 * @return BIND_STORE_STATUS_OK(0x00) / BIND_STORE_STATUS_FAIL(0x02)
 */
__HIGH_CODE
uint8_t binding_store_clear(void)
{
    uint8_t ret = (uint8_t)EEPROM_ERASE(BIND_STORE_BASE, BIND_STORE_ERASE_LEN);

    if (ret != 0) {
        PRINT("[STORE] clear (erase) fail: %d\r\n", ret);
        return BIND_STORE_STATUS_FAIL;
    }
    PRINT("[STORE] flash binding records erased (@0x%04X, %d bytes)\r\n",
          BIND_STORE_BASE, BIND_STORE_ERASE_LEN);
    return BIND_STORE_STATUS_OK;
}

/* POWER_STATE_STORE_BEGIN
 * Independent erase block: binding block 0 and BLE SNV block 0x7000 stay intact.
 * The record is 16 bytes: magic, version/state, its complement, commit tag.
 * Do not treat a torn/invalid record as ON. No write occurs during boot restore.
 */
#define POWER_STORE_BASE       0x1000u
#define POWER_STORE_ERASE_LEN  EEPROM_BLOCK_SIZE
#define POWER_STORE_MAGIC      0x57504843u
#define POWER_STORE_VERSION    0x00010000u
#define POWER_STORE_COMMIT     0x4B4F5750u

#if (POWER_STORE_BASE % EEPROM_BLOCK_SIZE) != 0 || \
    (POWER_STORE_BASE < (BIND_STORE_BASE + BIND_STORE_ERASE_LEN)) || \
    ((POWER_STORE_BASE + POWER_STORE_ERASE_LEN) > BLE_SNV_ADDR) || \
    ((POWER_STORE_BASE + POWER_STORE_ERASE_LEN) > EEPROM_MAX_SIZE)
#error "Power state storage overlaps another Data-Flash region"
#endif

__attribute__((aligned(4))) static uint32_t s_power_img[4];
__attribute__((aligned(4))) static uint32_t s_power_rd[4];
static uint8_t s_power_cached_valid;
static uint8_t s_power_cached_state;

static uint8_t power_state_record_valid(const uint32_t *record)
{
    return record[0] == POWER_STORE_MAGIC &&
           (record[1] == POWER_STORE_VERSION || record[1] == (POWER_STORE_VERSION | 1u)) &&
           record[2] == ~record[1] && record[3] == POWER_STORE_COMMIT;
}

__HIGH_CODE
uint8_t power_state_store_load(uint8_t *state)
{
    s_power_cached_valid = 0;
    if (state == NULL) return 1u;
    *state = 0;
    if (EEPROM_READ(POWER_STORE_BASE, s_power_rd, sizeof(s_power_rd)) != 0u ||
        !power_state_record_valid(s_power_rd))
    {
        PRINT("[BLE-PWR] no valid stored state; wait for controller\r\n");
        return 1u;
    }
    s_power_cached_state = (uint8_t)(s_power_rd[1] & 1u);
    s_power_cached_valid = 1;
    *state = s_power_cached_state;
    PRINT("[BLE-PWR] restored state=%u\r\n", (unsigned int)*state);
    return 0u;
}

__HIGH_CODE
uint8_t power_state_store_save(uint8_t state)
{
    uint8_t i;
    if (state > 1u) return 1u;
    if (s_power_cached_valid && s_power_cached_state == state) return 0u;

    s_power_cached_valid = 0;
    s_power_img[0] = POWER_STORE_MAGIC;
    s_power_img[1] = POWER_STORE_VERSION | state;
    s_power_img[2] = ~s_power_img[1];
    s_power_img[3] = POWER_STORE_COMMIT;
    if (EEPROM_ERASE(POWER_STORE_BASE, POWER_STORE_ERASE_LEN) != 0u ||
        EEPROM_WRITE(POWER_STORE_BASE, s_power_img, sizeof(s_power_img)) != 0u ||
        EEPROM_READ(POWER_STORE_BASE, s_power_rd, sizeof(s_power_rd)) != 0u)
        return 1u;
    for (i = 0; i < 4u; i++)
        if (s_power_rd[i] != s_power_img[i]) return 1u;
    if (!power_state_record_valid(s_power_rd)) return 1u;

    s_power_cached_state = state;
    s_power_cached_valid = 1;
    PRINT("[BLE-PWR] stored state=%u verified\r\n", (unsigned int)state);
    return 0u;
}
/* POWER_STATE_STORE_END */
