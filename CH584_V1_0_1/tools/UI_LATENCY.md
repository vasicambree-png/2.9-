# 最终版11：菜单响应优化合入说明

2026-10-14：本目录另已合入独立供电启动显示，当前正式及诊断镜像均已更新。该行为及验证见 [STANDALONE_BOOT.md](STANDALONE_BOOT.md)，下文的刷新加速说明继续适用。

本目录合入此前的刷新调度、快绘和诊断代码，保留本目录的启动图资源及 ST7305 首次初始化显示路径。没有恢复 APP 中的 UI_Boot_Display。

## 改动

- APP/Usart3_task.c：按页面/焦点等可见变化请求刷新，使用 TMOS 系统时间。导航刷新间隔 20ms，普通数据间隔 50ms，均从上次绘制及传输完成起算。合并连续请求，只绘制最新状态。该间隔不是完整按键响应时间的保证。
- APP/yuying_TFT.c：将位图和填充矩形改为按缓冲字节绘制；不适用的尺寸、旋转和裁剪条件回退原 u8g2 实现。诊断版分别测量等待、绘制和传输耗时。
- APP/include/ui_redraw_policy.h、ui_fast_draw.h、ui_latency_diag.h：上述实现与诊断开关。正式固件默认不输出新增日志。
- .cproject：保留现有配置，仅增加 tools、obj 源码扫描排除项，防止主机测试入口编进 MCU 固件。
- tools：保留原启动图工具，新增可复现的构建、采集及回归检查工具。

串口协议、BLE、Flash、屏幕驱动及启动资源没有修改。刷新触发时机及合并行为发生变化，仍需实板检查附近功能。

## 已执行验证

- 在本目录全量重编 77 个 MCU 对象，正式、诊断和修改前对照镜像生成成功；固件编译日志 0 error、0 warning。
- 2727 个实际 UI 组合：修改前后整个帧缓冲和绘图状态逐字节一致。
- 8620 个绘图对比：含裁剪、反色、透明/不透明模式、越界保护及回退条件，通过。
- 刷新策略测试：首帧、导航优先、请求合并、限频、任务延迟、取消和时钟回绕，通过。
- HEX 校验及目标源码差异核对通过。既有文件仅改动上述两个 APP 文件和 .cproject；启动驱动及资源 hash 未变。
- 主机 u8g2 测试存在原库注释编码/类型转换警告；上述固件 0 warning 不表示主机测试无警告。

备份、diff、编译日志、测试日志和 merge_verification.json 位于 obj/ui_latency_check。

## 固件与实板验收

- obj/CH584M_TFT_HB.hex、obj/ui_latency_check/optimized.hex：正式优化版。
- obj/ui_latency_check/diagnostic.hex：同一优化加耗时日志，启动标记 timing v4; fast bitmap/box。
- obj/ui_latency_check/baseline.hex：使用本目录修改前的 UART/TFT 快照构建的对照版。

此次没有烧录；实板响应耗时和其他硬件功能尚未验证。下载时使用现有下载流程并保留绑定数据，避免全片擦除。

建议烧录诊断版后验证：
1. 上电启动图显示一次，进入菜单后不会在运行重初始化时重放。
2. 亮屏菜单单次移动、快速连按、确认/返回及分页，检查焦点无异常和最终状态正确。
3. 灭屏等待后唤醒，以及保存成功/失败提示，检查画面恢复和提示优先级。
4. 传感器数值持续更新，同时操作菜单，检查数据及页面不停止刷新。
5. 蓝牙扫描/绑定、保存和断电恢复，检查既有通信与绑定数据。

CH584 日志为 COM10、115200 8N1；底板为 COM13。串口助手与采集脚本不要同时占用端口。诊断字段 wait_max、render_max、transfer_max、draw_max 单位为 625us，乘 0.625 得毫秒；它们是同一启动周期独立累计最大值，不能相加当成同一次按键。它们不覆盖底板识别按键之前的耗时。

## 重新构建

在本项目目录运行 tools/build_ui_latency.py，参数为 --compiler-bin 和 --make-bin；本机工具分别位于：

- D:/11/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12/bin
- D:/11/MounRiver_Studio2/resources/app/resources/win32/others/Build_Tools/Make/bin

页面检查运行 tools/verify_ui_fast_pages.py --vcvars，使用本机 Visual Studio 的 vcvars64.bat。绘图及调度检查命令保存在 obj/ui_latency_check/fast_test_build.cmd 和 test_build.cmd。

本目录不是 Git 仓库，未创建仓库或同步 GitHub。
