#include "flash.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "New_Key.h"
#include "config.h"
#include "main.h"
#include "Sx126x_Driver.h"
#include "LoraPro.h"
#include "iwdg.h"       /* re_iwdg()：数据 EEPROM 写入循环里要喂狗（见下面注释） */
#include "ui.h"         /* ★ 第六轮：Save_data() 用 UI_show_save_result() 发保存结果提示帧 */

#define EEPROM_START_ADDR  0x08080000  // data EEPROM start
#define EEPROM_END_ADDR    0x080807FF  // data EEPROM end (2KB)

extern uint8_t New_version;
save Flash_save;
extern Flash_Set Setup;
extern S_LoRaConfig G_LoRaConfig;
extern Flash_Set backups_Setup;

void Erase_EEPROM(void) {
    HAL_FLASHEx_DATAEEPROM_Unlock();
    HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_START_ADDR);
    HAL_FLASHEx_DATAEEPROM_Lock();
}

/* ==========================================================================
 * ★★ 第二轮修正：数据 EEPROM 写入要看住门狗预算
 * --------------------------------------------------------------------------
 * 事实（读 HAL 源码得到，不是猜的）：
 *   · stm32l0xx_hal_flash.h:40   #define FLASH_TIMEOUT_VALUE  (50000U)   // 50 秒！
 *   · stm32l0xx_hal_flash.c:646  FLASH_WaitForLastOperation() 用
 *     HAL_GetTick() 轮询 FLASH_SR 的 BSY 位，超时上限就是这个 50 s，
 *     **循环里没有任何喂狗**。
 * 所以：
 *   · 正常情况：一个字约 3.3 ms，400 字节 = 100 个字 ≈ 330 ms 一整套
 *     （加一次页擦除），远小于 IWDG 的 ≈2.08 s —— 安全。
 *   · 异常情况（NVM 卡住）：HAL 那个 50 s 超时根本轮不到，因为 IWDG 会在
 *     ≈2.08 s 先把芯片复位。对"存储器硬件故障"来说，复位正是期望的兜底行为。
 * 这里加的 re_iwdg() 是为了把**正常情况**的安全余量再拉大：每写一个字喂一次狗，
 * 即使某个字比预期慢很多，也不会累计到看门狗超时。
 * ========================================================================== */
void Write_EEPROM(uint8_t *data, uint32_t data_len) {
    HAL_FLASHEx_DATAEEPROM_Unlock();

    uint32_t addr = EEPROM_START_ADDR;
    uint32_t remaining = data_len;
    HAL_StatusTypeDef status = HAL_OK;

    while (remaining > 0 && addr < EEPROM_END_ADDR) {
        uint32_t write_len = (remaining >= 4) ? 4 : remaining;
        uint32_t value = 0;

        for (uint8_t i = 0; i < write_len; i++) {
            value |= (uint32_t)data[i] << (i * 8);
        }

        status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, addr, value);
        if (status != HAL_OK) {
            status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, addr, value);
        }

        re_iwdg();      /* ★ 每写一个 32bit 字喂一次狗 */

        addr += 4;
        data += 4;
        remaining -= write_len;
    }

    HAL_FLASHEx_DATAEEPROM_Lock();
}

void Read_EEPROM(uint8_t *data, uint32_t data_len) {
    uint32_t addr = EEPROM_START_ADDR;
    uint32_t remaining = data_len;

    while (remaining > 0 && addr < EEPROM_END_ADDR) {
        uint32_t read_len = (remaining >= 4) ? 4 : remaining;
        uint32_t value = *(__IO uint32_t*)addr;

        for (uint8_t i = 0; i < read_len; i++) {
            data[i] = (value >> (i * 8)) & 0xFF;
        }

        addr += 4;
        data += read_len;
        remaining -= read_len;
    }
}

uint8_t Verify_EEPROM(uint8_t *write_data, uint8_t *read_data, uint32_t data_len) {
    for (uint32_t i = 0; i < data_len; i++) {
        if (write_data[i] != read_data[i]) {
            return 0;
        }
    }
    return 1;
}

/* ==========================================================================
 * ★★ V1_0_2 修正（现场"按键按多了就卡死复位"的直接原因）：这里四个 400 字节
 *    的缓冲区原来全是**栈上局部变量**，而本工程的栈只有 1 KB
 *    （MDK-ARM\startup_stm32l051xx.s: Stack_Size EQU 0x400；链接映射里
 *     STACK 段 = 1024 字节，Heap 段已被链接器移除）。
 *
 *    调用链（都在主循环上下文、单线程，所以改成 static 完全安全）：
 *       Save_data()            write_buf[400]
 *        └ Write_data_to_EEPROM() read_data[400]     <-- 已经 800 字节
 *           ├ Erase_EEPROM() / Write_EEPROM() / Read_EEPROM()
 *           └ printf()                                <-- 库函数还要上百字节
 *    再加上 Key_task -> sub2_select -> Save_data 这一串调用者本身，
 *    以及随时可能嵌套进来的中断栈帧，**峰值必然越过 1 KB** -> 栈被踩到
 *    别的变量/返回地址上 -> HardFault（处理函数是 while(1)，还不喂狗）
 *    -> IWDG 约 2 s 后复位。表现就是"菜单操作/按键多按几次就卡死重启"。
 *    改成 static 之后这几个 400 字节从栈搬到 .bss，栈峰值直接降下来。
 *
 *    注意：这几个函数**只在主循环上下文调用**（按键软定时器回调 /
 *    开机初始化），不重入，所以共享 static 缓冲区没有问题。
 * ========================================================================== */

/* ==========================================================================
 * ★★ V1_0_2 第六轮：返回值改成"是否真的写成功"，供 Save_data() 做页面提示
 * --------------------------------------------------------------------------
 * 原来这个函数是 void，只在串口打印 write_success/write_error：
 *   · 上层（Save_data）拿不到结果，也就没法给用户"保存成功/失败"的页面提示；
 *   · 而且**失败重试之后没有再校验一次** —— 重试也失败的话，
 *     函数照样静默返回，串口上只留下第一次的 write_error。
 * 现在：回读校验通过才返回 1；失败重试一次并把重试结果也校验掉，返回最终结果。
 * 调用方：Flash_text()（恢复出厂，忽略返回值）和 Save_data()（用它决定提示）。
 * ========================================================================== */
uint8_t Write_data_to_EEPROM(uint8_t *data, uint16_t data_len)
{
    static uint8_t read_data[400];      /* static：见上面的栈溢出说明 */
    uint8_t ok;

    memset(read_data, 0, sizeof(read_data));
    Erase_EEPROM();

    Write_EEPROM(data, data_len);

    Read_EEPROM(read_data, data_len);
    ok = (uint8_t)Verify_EEPROM(data, read_data, data_len);
    if (ok) {
        printf("write_success\r\n");
        return 1U;
    }

    /* 失败：擦干净重试一次，并且**重试后也要回读校验**（原实现漏了这一步） */
    printf("write_error, retry once\r\n");
    memset(read_data, 0, sizeof(read_data));
    Erase_EEPROM();
    Write_EEPROM(data, data_len);
    Read_EEPROM(read_data, data_len);
    ok = (uint8_t)Verify_EEPROM(data, read_data, data_len);

    printf(ok ? "write_success(retry)\r\n" : "write_error(retry)\r\n");
    return ok;
}

void factory_reset(void)
{
    memset(Ch_senser, 0, sizeof(Channel_C) * 21);

    Flash_text();
    Start_read_EEPROM();
}

void Flash_text(void)
{
    static uint8_t read_buf[400];       /* static：见上面的栈溢出说明 */
    memset(read_buf, 0, sizeof(read_buf));
    Write_data_to_EEPROM(read_buf, sizeof(read_buf));
}

void GetNewsend_addr(void)
{
  /* ★★★ V1_0_21：跳步 3 -> **60**（用户确认）。为什么不是 3：
   * 跳步 s 直接决定"每条数据要跳几次"和"每台要替别人收/发几包"：
   *   一轮总传输 = Σ(S/s) ≈ 7260/s  ->  s=3 时 2420 包（占用 72%），s=60 时 121 包（占用 ~4.5%）
   *   单台中继负载 = 60.5/s          ->  s=3 时 20.2 包，s=60 时 1.0 包（RX 进入次数降 20 倍）
   * 按 DS_SX1268 与实测功耗模型，s=60 + CAD 200ms + 前导 300ms 时单台约 34uA
   *   （现状 s=3 + CAD 100ms + 前导 192ms 约 222uA）。
   * ★ s=60 时链只有两级：61..120 -> 1..60 -> 分站(122)，跳数从 40 降到 2。
   * ★ 备用地址仍给 3 个且只差 1~2 台，与原来的 N-3/N-2/N-1 同构
   *   —— 相邻 1~2 台坏掉照样能往前传（冗余哲学不变）。
   * ★ 台号 <=60 的台：直接发分站（分站常听，用 12 符号短前导，单包仅 ~23ms），
   *   两个备用取"更靠近分站的邻居"，与原来 Addr<=3 时的 122/1/2 语义一致。
   * ★ 注意：动态地址改写（LoraPro.c）**不允许把 slot0 改成 122**，见那里的 V1_0_21 说明。 */
  if(Setup.Host_number > 3)
  {
    Setup.transmit_part[0]   = Setup.partition;
    Setup.transmit_part[1]   = Setup.partition;
    Setup.transmit_part[2]   = Setup.partition;
    Setup.transmit_number[0] = Setup.Host_number - 3;
    Setup.transmit_number[1] = Setup.Host_number - 2;
    Setup.transmit_number[2] = Setup.Host_number - 1;
  }
  else if(Setup.Host_number == 3)
  {
    Setup.transmit_part[0]   = Setup.partition;
    Setup.transmit_part[1]   = Setup.partition;
    Setup.transmit_part[2]   = Setup.partition;
    Setup.transmit_number[0] = STATION;   /* V1_0_57: 删除 TEST-ONLY 残留，恢复旧设备行为（1/2/3 号台只发分站 122，见 send_next_lora()） */
    Setup.transmit_number[1] = 1;
    Setup.transmit_number[2] = 2;
  }
  else if(Setup.Host_number == 2)
  {
    Setup.transmit_part[0]   = Setup.partition;
    Setup.transmit_part[1]   = Setup.partition;
    Setup.transmit_part[2]   = NONEPART;
    Setup.transmit_number[0] = STATION;
    Setup.transmit_number[1] = 1;
    Setup.transmit_number[2] = NONEADDR;
  }
  else
  {
    Setup.transmit_part[0]   = Setup.partition;
    Setup.transmit_part[1]   = NONEPART;
    Setup.transmit_part[2]   = NONEPART;
    Setup.transmit_number[0] = STATION;
    Setup.transmit_number[1] = NONEADDR;
    Setup.transmit_number[2] = NONEADDR;
  }
}

/* read EEPROM: header(0x36 + version) + Flash_Set (Setup) */
void Start_read_EEPROM(void)
{
    static uint8_t read_buf[400];       /* static：见 Write_data_to_EEPROM() 上方的说明 */
    memset(read_buf, 0, sizeof(read_buf));
    Read_EEPROM(read_buf, 400);
    hex_p(read_buf, 20);

    if(read_buf[0] != 0x36)
    {
        memset(read_buf, 0, 400);
        read_buf[0] = 0x36;
        read_buf[1] = New_version;

        Flash_Set default_set = {
            .ble_bangding_flag = 0,
            .brightness = 5,
            .On_Off_flag = 1,
            .Host_number = DEFAULT_HOST_NUM,
            .partition = DEFAULT_PARTITION,
            .transmitted_power = 10
        };
        memcpy(&Setup, &default_set, sizeof(Flash_Set));
        GetNewsend_addr();  /* 首次初始化/恢复出厂：先生成完整默认路由，再写入。 */

        memcpy(read_buf + 2, &Setup, sizeof(Flash_Set));
        Write_data_to_EEPROM(read_buf, sizeof(read_buf));
    }
    else
    {
        memcpy(&Setup, read_buf + 2, sizeof(Flash_Set));
    }

    G_LoRaConfig.PowerCfig = Setup.transmitted_power;
    backups_Setup.transmitted_power = Setup.transmitted_power;
    /* 有效 EEPROM 配置保留用户保存的上传地址；动态选路/故障恢复仍由 LoRa 处理。 */
		memcpy(&backups_Setup,&Setup,sizeof(Setup));
		printf("Setup addr : %d ;Setup host : %d \r\n",Setup.partition,Setup.Host_number);

		printf("[BOOT] L0diag V1_0_64 part=%d host=%d slot=%d/%d/%d\r\n",(int)Setup.partition,(int)Setup.Host_number,(int)Setup.transmit_number[0],(int)Setup.transmit_number[1],(int)Setup.transmit_number[2]);
		printf("backups_Setup addr : %d ;backups_Setup host : %d \r\n",backups_Setup.partition,backups_Setup.Host_number);
		
}

/* ==========================================================================
 * Save_data ：把当前 Setup 参数写进 STM32 自己的 Data-EEPROM（菜单"保存"）
 * --------------------------------------------------------------------------
 * ★★ V1_0_2 第六轮新增：保存结束后**让屏幕给出明确的成功/失败提示**。
 * 需求原文："如果是参数保存在 stm32 flash 部分的保存成功，需要 ui 显示
 *           保存成功的显示页面提示"。
 * 为什么必须由 STM32 发这一帧：参数存在**本机**的 Data-EEPROM 里
 * （Write_data_to_EEPROM 的回读校验结果），CH584M 那边只是块显示屏，
 * 它拿不到这个结果，所以只能由本机把结果告诉它。
 * 帧内容：0x01 menu_rank = 6（提示页），1 个参数 = UI_MSG_PARAM_SAVE_OK/FAIL。
 * CH584M 收到后把提示画在屏幕上，约 2 秒后自动返回原页面
 *  （它不会把 menu_rank=6 当页面状态，所以返回后画面仍然正确）。
 * 与蓝牙绑定的保存（0x05，存在 CH584M 自己的 Flash 里）互不相干 ——
 * 那一种由 CH584M 自己提示，见 CH584 工程 Usart3_task.c 的 case 0x05。
 * ========================================================================== */
void Save_data(void)
{
    uint8_t ok;

    printf("Save_data \r\n");

    static uint8_t write_buf[400];      /* static：见 Write_data_to_EEPROM() 上方的说明 */
    memset(write_buf, 0, sizeof(write_buf));
    write_buf[0] = 0x36;
    write_buf[1] = New_version;
    memcpy(write_buf + 2, &Setup, sizeof(Flash_Set));

    ok = Write_data_to_EEPROM(write_buf, sizeof(write_buf));

    /* ★ 页面提示：成功/失败都提示（失败也要让用户知道，不能静默） */
    UI_show_save_result(ok ? UI_MSG_PARAM_SAVE_OK : UI_MSG_PARAM_SAVE_FAIL);
}

void Quick_BOR_Test(void)
{
    uint8_t bor_level = 0xFF;
    printf(" BOR level is %d \r\n", bor_level);
    bor_level = 0xFF;
    printf(" BOR level is %d \r\n", bor_level);
}
