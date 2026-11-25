# 硬件工程：20通道传感器-蓝牙主机

| 文件 | 内容 |
|---|---|
| `20通道传感器-蓝牙主机2.0.0.SchDoc` / `.pdf` | 主板（底板）原理图 2.0.0：STM32L051R8T6 + SX1268 LoRa + MX_02 蓝牙模块 + RS3485 + HS9149 红外 + BS813A 触摸键 + 电源开关 |
| `20通道传感器-蓝牙主机.PcbDoc` | 主板 PCB，器件与网络和 2.0.0 原理图一致（STM32L051、SX1268、MX_02、HS9149、RS3485） |
| `屏幕驱动.schdoc` | 屏幕驱动板原理图：CH585M（BLE + SPI 驱动墨水/反射屏 FPC）；同一张图里还画了下一代一体化方案（STM32U031R8T6 + SX1268 + MX_02 + 485 等） |

原始文件按上传内容原样保存（SHA256 与上传文件一致）。

## 系统结构

```
BLE 传感器(最多20路广播) ──BLE──▶ 屏幕板 CH585M ──UART3 115200──▶ 主板 STM32L051 ──SX1268 LoRa──▶ 中继/分站
                                    │ SPI0 驱动 ST7305 屏                 │ 按键/触摸/红外/光唤醒/电源管理
```

- `CH584_V1_0_1/`：跑在屏幕板 CH585M 上。BLE Observer 扫描广播、维护绑定和 20 通道数据，u8g2 画屏，UART3 与主板交换 0x01/0x03/0x06 帧。
- `1_新L0底板_完整工程_V1_0_64/`：跑在主板 STM32L051 上。USART1 和屏幕板通信，取回 20 路数据 `Ch_senser[0..19]`，按周期经 SX1268 LoRa 上传（含中继/路由），负责按键、光唤醒、休眠、屏幕供电。

## 主板 STM32L051 引脚对照（原理图 2.0.0 ↔ 代码）

| 引脚 | 原理图网络 | 代码用途 | 结论 |
|---|---|---|---|
| PA4/PA5/PA6/PA7 | CSN/SCLK/MISO/MOSI → SX1268 | SPI1 + `LORA_NSS`（`Core/Inc/spi.h`） | 一致 |
| PB1 / PB2 | SX1268 CTL1 / CTL2（射频开关） | `LORA_SW1/SW2` | 一致 |
| PB10 | SX1268 BUSY | `LORA_BUSY_VAL` | 一致 |
| PB11 | RST-RF | `LORA_RST` | 一致 |
| PC4 / PC5 / PB0 | DIO1 / DIO2 / DIO3 | `Lora_Gpio_Init()` | 一致 |
| PB6 | TX-YJP → JX1/JX2 脚2（去屏幕板） | USART1_TX，屏幕协议 | 一致 |
| PB7 | USART1_RX → 只到 S1 脚3（TTL1 口） | USART1_RX，接收屏幕 0x03 应答 | **不一致**：JX1/JX2 脚3 在原理图和 PCB 上都是孤立网络（`NetJX1_3`），屏幕→主板方向没有接到 PB7 |
| PA15 | LCD_BUSY（JX 脚5） | 配置已注释，未使用 | 硬件有、代码不用 |
| PB4 | 液晶屏供电 Q8 → 3V3-YJP | `lcd_power_on()`，低电平开 | 一致 |
| PB3 | 光唤醒供电 Q6 → 3V3-REMOTE | `Light_on/off`，低电平开 | 一致 |
| PC0 | 光敏三极管 3DU33 分压 | `Light_check()` 按数字电平读取 | 一致（不走 ADC） |
| PC1 + PC3 | 电池分压 / 3V3_ADC 开关 | 未使用；电池电压改用 VREFINT 估算 VDD | 硬件有、代码不用 |
| PB9 | 触摸供电 Q3 → 3V3-CM | 初始化输出低（开） | 一致；`Sleep.c` 里的 `EXTI_wakeup1()` 会把 PB9 改成中断输入，与原理图用途不符，目前未被调用 |
| PB14 / PB15 / PC6 | CM-OUT-1/2/3（BS813A 触摸） | `Touch_KEY1..3`，下降沿中断 | 一致 |
| PA8 / PA9 / PA10 | KEY1/2/3（实体键 + HS9149 红外解码输出） | `KEY1..3`，上升沿中断，下拉 | 一致 |
| PA0 | GD-9149 → HS9149 供电 Q7 | 初始化输出低（开），休眠置高 | 一致 |
| PD2 | Lanya_kz → MX_02 蓝牙模块供电 Q11 | 输出高 = 关闭 | 板载 MX_02 被刻意关掉 |
| PC7~PC12 | MX_BRTS/LINK/CDS、LPUART_TX/RX、MX_RST | 设为模拟输入，LPUART1 未初始化 | 板载 MX_02 不使用，BLE 改由屏幕板 CH585 完成 |
| PA2 / PA3 | USART2 → RS3485 + DaYin1 打印口 + S1 脚6 | USART2 115200 做 printf 调试 | 共线：485 收发器与调试口并在一起 |
| PA1 | 485-/RE（/RE+/DE） | 未配置 | 485 方向脚浮空，485 未使用 |
| PB5 | 485 供电 Q10 | 未配置 | 栅极浮空，485 供电状态不确定 |
| PC13 / PA11 / PB13 / PB8 | 控制供电1/2、TTL1/TTL2 供电 | 未配置 | S1/S2 外接传感器口在固件中未启用 |

## 屏幕板 CH585M 引脚对照（屏幕驱动.schdoc ↔ `CH584_V1_0_1`）

| 引脚 | 原理图网络 | 代码用途 | 结论 |
|---|---|---|---|
| PA12 / PA13 / PA14 | SPI0_CS / SCK / MOSI → FPC 12/13/14 | `TFT_IO_init()` + `SPI0_MasterDefInit()` | 一致 |
| PA2 / PA3 | LCD_DC / LCD_RES → FPC 11/10 | `DC_CMD/DC_DATA`、`RST_LOW/HIGH` | 一致 |
| PA1 | 经 R31 接 LCD_BUSY（FPC 9，同时去 YJP1 脚5 → 主板 PA15） | 配成推挽输出并置高 | 需核对：BUSY 通常是屏输出的信号，MCU 主动驱动可能冲突 |
| PA4 / PA5 | RXD3 / TXD3 → YJP1 脚3/脚2 | UART3 115200，主板协议 | 一致 |
| PA8 / PA9 | RXD1 / TXD1 → LOG1 | UART1 调试打印（DEBUG=1） | 一致 |
| PB23 | RST# 按键 S2 | 代码注明不作为 GPIO 使用 | 一致 |
| PB0 / PB1 | 未连接 | 推挽输出低 | 无影响 |
| 芯片 | 原理图为 CH585M | 工程名 CH584，但 `CHIP_ID=ID_CH585`、启动文件 CH585 | 原理图确认实际芯片为 CH585M |

## 需要关注

1. 主板 2.0.0 的 PB7(USART1_RX) 没有接到屏幕座 JX 脚3，而固件依赖屏幕板回传 0x03/0x06 帧。实物如果能正常显示 20 路数据，说明板上有飞线或用的不是这一版；改版时应把 JX 脚3 接到 PB7。
2. `屏幕驱动.schdoc` 中的 STM32U031R8T6 方案与仓库 L0 固件引脚完全不同（例如 LoRa 改到 PC4=RST、PB2=BUSY、PB10/PB11=CTL2/CTL1，USART2 改接屏幕），仓库固件不能直接用于那块板。
3. 485、TTL 传感器口、电池分压 ADC 在硬件上都有，但固件当前全部未启用；所谓"20通道"数据全部来自 BLE 广播。
