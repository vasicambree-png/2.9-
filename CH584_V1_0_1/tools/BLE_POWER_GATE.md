# 开关状态保存与数值显示

本版只由已生效的开关状态决定是否显示测量值；蓝牙扫描、绑定、广播解析、通道更新、
0x03 应答和 0x06 上报继续按原流程运行。开关切换不会取消扫描、丢弃队列、清空通道
或重置采集轮次。CH584 必须有电才能运行；底板原有供电、按键、息屏和点亮策略未修改。

## 显示规则

| 内容 | 开机 | 关机或尚无有效状态 |
|---|---|---|
| 主页两页共 20 个通道值 | 按原类型/单位显示实际值 | 数值显示 `--` |
| 主页顶部电压 | 按原格式显示电压 | 显示 `--` |
| 安装调试的设备电压/信号 | 按原格式显示电压/RSSI | 显示 `--` |
| 通道号、类型/单位、名称、地址、计数和设置项 | 保持原样 | 保持原样 |

绘图只读数据，不写回占位符、不将通道清零。关机期间接收到的新数据继续更新缓存；
恢复开机后可显示最新已接收的数据，没有有效数据的通道仍按原逻辑显示 `--`。

主页状态来自底板 `Setup.On_Off_flag`。设置页中的 `Menu_rank6.state` 是编辑草稿，
只有设置保存结果确认已应用后才改变生效状态，“不保存”不改变。首次没有有效记录
时默认隐藏测量值，蓝牙仍正常采集。开关状态不直接控制是否点亮面板。

## 掉电保存

继续使用独立 Data-Flash 块 `0x1000–0x1FFF` 中的 16 字节记录，含版本、状态、反码
和完成标识；不改绑定块 `0x0000–0x0FFF` 或 BLE SNV `0x7000` 块。状态真正变化才
尝试擦写并回读验证；重复帧不写，启动恢复同值不写。失败不反复擦写，也不影响
蓝牙采集；运行中状态仍生效。没有有效记录时默认隐藏，等待底板校正。

恢复的是屏幕上次已知并成功保存的状态，后续有效底板状态会校正。单块保存不保证
任意擦写瞬断都原子完成：可能恢复旧有效记录或因记录无效回退隐藏。保存只增加
短暂擦写和启动读取能量，保持记录不需持续供电。具体耗电和真实断电可靠性尚未实测。

## 验证

本轮证据在 `verification/ble_display_only_20261028/`：

- 直接执行生产状态函数、BLE 回调、广播入队/消费及看门狗，确认 ON/OFF 都能采集，
  切换不丢待处理数据、不清轮次，失败保存不反复写。
- 用真实 UI_Control/u8g2 绘图，开机下无数据、部分数据、最大值和真实零值的 16 帧
  与修改前逐字节相同。关机下 12 帧在测量值大幅变化时完全相同，并检查占位符数量。
  绘图前后控制器数据、通道及绑定表一致；保存 PNG 预览用于人工查看。
- 生产 Flash 函数模型覆盖重复值零 I/O、加载免写、记录损坏、中断写及读擦写失败，
  检查绑定和 SNV 区域未变化。原有启动/息屏/串口显示回归保留。

测试命令在工程根目录执行，工具路径按实际安装位置调整：

```powershell
python tools/verify_ble_power_gate.py --vcvars "D:/visual studio/visual studio2026/VC/Auxiliary/Build/vcvars64.bat"
python tools/verify_display_state.py --vcvars "D:/visual studio/visual studio2026/VC/Auxiliary/Build/vcvars64.bat" --baseline verification/ble_display_only_20261028/before/APP/yuying_TFT.c
python tools/verify_power_state_store.py --vcvars "D:/visual studio/visual studio2026/VC/Auxiliary/Build/vcvars64.bat" --out verification/ble_display_only_20261028/store_host
python tools/verify_standalone_boot.py --vcvars "D:/visual studio/visual studio2026/VC/Auxiliary/Build/vcvars64.bat"
```

本轮不再继续黑屏诊断：撤下尚未同步的唤醒诊断源改动，并移除新增的周期 BLE-PWR
状态快照。历史诊断产物保留在本地，不作本次下载文件。

GCC12/GCC15 各全量编译 77 个源文件，均为 0 警告、0 错误。

新固件为 `obj/CH584M_TFT_HB_DISPLAY_STATE.hex`。按原下载配置保留 Data-Flash；
实板尚未验证。最短复测：保存不开机，确认数值和电压均为 `--`；改变传感器输入后
再保存开机，检查显示最新数值；分别重启，核对保存状态恢复及原按键/息屏行为。
