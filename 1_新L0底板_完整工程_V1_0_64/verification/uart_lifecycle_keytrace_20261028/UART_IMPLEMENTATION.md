# USART1 生命周期与复位检查点

本次仅改USART1生命周期、对应IRQ入口和必要链接布局，不改变USART2、600秒采集周期、绑定表、空口协议或射频IRQ。

## 生命周期

`UART1_DeInitSafe()`负责先关闭USART1 NVIC、确保外设时钟有效，再禁止UE/旧中断、清错误与旧接收字节、调用HAL_UART_DeInit，最后清NVIC挂起。HAL自行调用MSP关闭时钟；禁止在它之前另调MSP DeInit。Sleep.c的入口由根任务替换为同一API。

`MX_USART1_UART_Init()`从入口就关闭USART1 IRQ，并在时钟有效时清旧硬件状态。USART1 MSP保留GPIO/DMA初始化，但不再提前打开IRQ。`sensor_rx_start()`在IRQ关闭期间清软件队列、终止旧接收、清ORE/FE/NE/PE/IDLE与旧RXNE，Receive_IT实际安装RxISR并进入BUSY_RX后才开IRQ。正常逐字节ISR入队与重武装保持原函数。

`UART1_IRQHandlerSafe()`处理RXNEIE+RXNE而RxISR为空的异常：先保存现场，再关闭该IRQ、清接收中断和错误/旧字节，交回主循环重新安装接收。ISR不打印、不访问Flash/EEPROM、不喂狗。正常路径继续调用HAL_UART_IRQHandler。

## 64字节保留RAM

ARMCC编译的`UART_NOINIT`节由新scatter明确放入0x20001FC0..0x20001FFF的UNINIT执行区。普通RAM上限0x20001FC0，Flash范围和startup中的1024字节栈/1536字节堆不变。不能只凭C属性宣称保留；必须在Keil实际map和zero-init表核对该区没有被启动清零。

记录先将魔数置零，在短临界区保存字段及异或校验，最后提交魔数并恢复原PRIMASK。启动函数在CheckResetReason清复位标志前读取，仅IWDG或软件复位、无POR/PDR标志、魔数/阶段/校验均有效时打印`[U1PRE]`。掉电、上电/欠压复位或损坏记录不能作为前周期证据。

这是**最后UART检查点**，不是自动判定的根因，也不是故障历史队列。正常后续初始化/接收重武装会覆盖较早的orphan检查点；未复位而已经恢复的异常不会永远保留。IRQ计数是该检查点捕获时的累计值。enr中USART1EN为0时isr/cr1/cr3/brr置零表示未读取。

| stage | 含义 |
|---|---|
| 1 | 本次启动记录已开始 |
| 2 | 初始化入口，时钟已开、旧状态尚未清除 |
| 3 | 旧硬件状态已清除 |
| 4 | USART1 MSP已完成、IRQ仍关闭 |
| 5 / 6 | 正在等TEACK / REACK |
| 7 / 8 | 初始化成功 / 失败 |
| 9 / 10 | 准备安装接收 / 接收已经安装、准备开IRQ |
| 11 / 12 | 安全关闭开始 / 已关闭 |
| 13 | RXNE中断触发但RxISR为空，清除前的现场 |
| 14 | 进入HardFault后的UART检查点 |

HardFault只保存阶段和UART寄存器/IPSR，没有保存故障PC/LR或完整栈，因此stage=14仍需进一步定位。记录校验用于识别不完整/随机RAM，不是安全认证。

## 已做验证

`test_uart_lifecycle.py`抽取当前生产函数，以MSVC /W4 /WX编译运行；硬件边界为mock。覆盖旧状态清理、50次开关循环、先装RxISR后开IRQ、orphan截断与主循环恢复、失败arm、TEACK/REACK/双时基冻结/回绕、首次初始化前关闭、PRIMASK恢复、保留记录校验及POR拒绝。USART2初始化、错误回调、正常字节ISR函数与改前逐函数核对一致。

结果位于uart-tests/。Keil整体构建、UNINIT map/zero表、实物复位后的RAM保留和射频持续业务由根任务分别验证；主机测试不证明物理芯片时钟门控及实物重启已解决。
