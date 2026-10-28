# 底板 ARM Compiler 5 / 6 兼容验证

## 修改

- `Core/Src/usart.c`：先识别 Arm Compiler，再识别 GNU，AC5/AC6 都使用 Arm C 库的 `fputc` 串口重定向；GNU 分支保持。
- AC5 使用原 `UART_NOINIT` 零初始化节，AC6 使用 `.bss.UART_NOINIT`；scatter 接受两者，在 `0x20001FC0` 留64字节 UNINIT。没有进入启动复制/清零表。
- `MDK-ARM/MX_02.uvprojx` 新增 `MX_02_AC6`，使用相同源文件、芯片、内存、MicroLIB、优化和 short enum/wchar 设置。ARMASM 继续编译原启动文件。两个目标输出隔离，原 AC5 目标保留。
- AC6 只关闭 GB18030 注释的 `-Winvalid-utf8` 提示，没有改写源码编码或关闭其他警告；字符串编码诊断仍报告。

## 当前证据

AC5 5.06 update7 build960：0错误、50警告；AC6 6.21：0错误、1628警告。两种构建均从源码全量重编。AC5原有警告未清理；AC6 主要警告类别和固件SHA256在 `results.json`。这些是现有代码诊断，不代表均可忽略。本轮未扩大为业务重构：例如 `LoraPro.c` 的 `!=122 || !=121` 恒真比较和一处中文诊断字符串编码警告保留在日志中。

AC5 Code=51252 RO=2652 RW=724 ZI=7044；AC6 Code=55556 RO=3956 RW=64 ZI=7808。RAM统计包含固定栈、堆和64字节保留区，但不证明运行时栈峰值。两者均符合64KB Flash、8KB RAM布局。

`verify_images.py` 验证HEX记录、Flash地址、AXF导出内容相同、初始SP与复位向量、printf来自usart.o、64字节记录及启动初始化表不覆盖保留区、反汇编无BKPT。AC5 HEX SHA256与上一版LK3一致：74b4342ea65eaa0c8dbef96ac7c554a4ddad018770370cb68767ad54e9726d92。

UART生产函数主机测试也通过，覆盖初始化/关闭/重挂、孤立IRQ、50轮生命周期、记录校验、POR拒绝及TEACK/REACK失败。测试使用硬件边界桩，不证明实物时序或无线收发。

## 复现

在Keil打开 `MDK-ARM/MX_02.uvprojx`，选 `MX_02` 或 `MX_02_AC6`，在Target设置中选择本机安装的对应编译器，执行Rebuild。版本字段是工程选择，不是硬门槛。AC6已验证6.21，其他版本尚未验证；安装中需有ARMASM。不要把原ARMASM启动文件交给Clang GNU汇编器。保留自定义scatter，不改回自动生成链接布局。

Keil命令行：`UV4.exe -r <工程绝对路径> -t MX_02_AC6 -o <日志绝对路径>`，AC5目标名改为MX_02。离线镜像复核：`python verify_images.py <fromelf.exe绝对路径>`，脚本读取两个目标输出和当前目录构建日志。

## 尚未验证

本轮未连接串口、探针或执行烧录。实物最短复测：选择明确的AC6 HEX，按已有下载流程烧录并校验；独立复位确认USART2启动日志和LK3标识；接屏幕确认菜单及息屏唤醒；覆盖至少3轮600秒采集上传并核对分站收到新数据。用软件/IWDG复位检查有效`[U1PRE]`记录，用完全断电确认旧记录被拒绝。长期运行、功耗、射频和栈峰值仍须现场测量。
