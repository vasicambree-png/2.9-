# CH584 编译器选择

本工程保留 WCH GCC12，同时提供 WCH GCC15 工程入口。没有增加仅允许某一编译器版本的检查；普通 ARM Keil 编译器不适用于此 RISC-V MCU。其他 WCH 工具链需要按实际芯片指令集、库和 ABI 验证，不能由本次两套构建推导为全部兼容。

## MounRiver Studio 工程入口

| 入口 | 默认工具链 | 编译目录 | 下载 HEX |
|---|---|---|---|
| `CH584M_TFT_HB.wvproj` | WCH GCC12 | `obj` | `obj/CH584M_TFT_HB.hex` |
| `CH584M_TFT_HB_GCC15.wvproj` | WCH GCC15 | `obj_gcc15` | `obj_gcc15/CH584M_TFT_HB_GCC15.hex` |

两个入口共享同一目录下全部生产源码、CH584 96KB RAM 链接脚本和 BLE/ISP 库；互相排除两个构建目录。GCC15 入口的调试器和下载文件也随之选择 GCC15，不会误用 GCC12 输出。打开实际收到的 `.wvproj`，核对正在编译的项目根目录，不要复用 `.mrs` 中指向旧机器目录的历史工作区。

独立 GCC15 入口已按现有 `.wvproj` 格式生成并核对参数；本次使用下面的命令行脚本完成真实目标构建。MRS 图形界面导入、下载和真实板运行尚未验证。

## 可选工具链的命令行构建

`tools/build_wch.py` 接收 `--compiler-bin`，不锁定 GCC12 或 GCC15 版本号。使用已安装、适用于 CH584 的 WCH 工具链目录即可尝试构建。以下命令在工程根目录执行，Python 3.10 及以上；工具链路径按本机安装位置替换：

```powershell
python tools/build_wch.py --compiler-bin "D:/11/MounRiver_Studio3/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12/bin" --out verification/my_gcc12_build
python tools/build_wch.py --compiler-bin "D:/11/MounRiver_Studio3/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15/bin" --project CH584M_TFT_HB_GCC15.wvproj --out verification/my_gcc15_build
```

每次指定新的输出目录。脚本从选定工程读取包含目录、宏、库、链接脚本和排除项，扫描当前源文件；全量编译，不复用旧对象。CH584 指令集与代码生成参数沿用当前工程，输出完整命令、编译器版本、输入哈希、ELF/HEX 和 `result.json`。只有构建及静态镜像检查全部完成，`verified` 才为 `true`；不执行下载、擦除、复位或串口操作。

脚本把源文件、对象、链接脚本和输出都作为工程相对路径传入工具。实测本机 WCH GCC15 15.2.0 的汇编器不能创建含中文的绝对路径对象文件，而目录确实存在；同一源码、相同编译参数改用相对路径可完成。这是本机的构建路径问题，不是朋友板上“烧录后无反应”的已证实原因。如果 MRS 的 GCC15 构建也出现 `can't create ... No such file or directory`，可用此脚本，或把完整工程放到英文目录再全量构建；不要拿旧 HEX 冒充新输出。

脚本输出的 HEX 固定为 `<--out>/CH584M_TFT_HB.hex`，与 IDE 两个入口各自的输出路径区别对待。手工选择本次实际生成的文件，CH584/5 程序地址为 `0x00000000`，不添加 STM32 的 `0x08000000` 偏移。按原下载流程保留绑定数据，不因为更换编译器扩大擦除范围。

## 2026-10-28 离线验证

| 工具链 | 全量源码 | 警告/错误 | Flash | RAM（含放 RAM 执行的代码） |
|---|---:|---|---:|---:|
| WCH GCC12 12.2.0 | 77 | 0/0 | 222964 / 458752 字节 | 67432 / 98304 字节 |
| WCH GCC15 15.2.0 | 77 | 0/0 | 221732 / 458752 字节 | 67844 / 98304 字节 |

证据在 `verification/gcc12_gcc15_compat_20261028/gcc12_final/` 和 `gcc15_final/`。两份 ELF 都是 RISC-V 32 位、ILP32/软浮点 ABI，入口为 0，跳转 `handle_reset`；保留 WCH XW 扩展、快速中断属性和 `mret` 返回。两份 HEX 的逐记录校验和、长度、EOF、地址不重叠及 448KB Flash 范围通过，输入哈希与本轮构建一致。生产应用、启动、驱动和蓝牙库未修改。

尚未烧录，屏幕、串口、蓝牙扫描、绑定保存及休眠/唤醒的真实板行为尚未验证。最短实物复测：在同一块板用明确选定的本轮 HEX 下载并校验，退出调试暂停；连接正常底板并触发有效页面，检查显示、菜单和蓝牙数据，再检查息屏/唤醒与已有绑定恢复。当前普通上电尚无有效页面时可保持暗屏，不以黑屏单独判定程序未运行。
