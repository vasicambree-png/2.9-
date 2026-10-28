# 通信状态控制蓝牙采集

仅修改 CH584 屏幕侧。底板 `User/ui.c` 的主页 `UI_main.state` 来自当前生效的
`Setup.On_Off_flag`：1（开机）正常扫描采集，0（不开机）停止扫描及广播处理。
设置页 `Menu_rank6.state` 来自 `backups_Setup`，属于编辑草稿，未保存或选择
“不保存”不改变采集状态。

收到通信设置的保存结果提示后，立即应用屏幕缓存的设置状态，无需返回主页。
底板 `New_Key.c` 在调用 `Save_data()` 前已经更新运行中的 `Setup`，所以参数保存
成功和 EEPROM 保存失败两种提示都同步运行状态；后续合法主页帧仍以实际状态校正。
其他菜单的保存提示、软关机提示、息屏和普通页面切换不控制此开关。
上电尚未收到有效状态前保留原有默认启用行为，不从设置页的零初始化值推断关机。
既有保存提示不携带菜单来源；若之前的导航或编辑帧丢失，缓存可能暂时不一致，
后续合法主页状态会重新校正。本次不扩展底板协议。

关闭时先禁止广播入队，清空未处理队列和未上报轮次，再取消扫描；初始化完成、
扫描结束回调及现有看门狗均遵守采集开关，取消扫描不会被完成回调重新开启。
恢复开机立即重新启动扫描；若 BLE 初始化尚未完成，则由初始化完成回调启动。
重复状态帧不重复启停。绑定表、Flash 保存、已采集的通道值、串口 0x03 应答、
显示及按键流程保留。

## 离线验证

从工程根目录运行，工具路径按实际安装位置调整：

```powershell
python tools/verify_ble_power_gate.py --vcvars "D:/visual studio/visual studio2026/VC/Auxiliary/Build/vcvars64.bat"
python tools/verify_standalone_boot.py --vcvars "D:/visual studio/visual studio2026/VC/Auxiliary/Build/vcvars64.bat"
python tools/build_wch.py --compiler-bin "D:/11/MounRiver_Studio3/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12/bin" --out verification/ble_power_gate_repeat/gcc12
python tools/build_wch.py --compiler-bin "D:/11/MounRiver_Studio3/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15/bin" --project CH584M_TFT_HB_GCC15.wvproj --out verification/ble_power_gate_repeat/gcc15
```

构建输出必须使用新的目录。本轮证据位于 `verification/ble_power_gate_20261028/`。
GCC12/GCC15 分别全量编译 77 个源文件，均为 0 警告、0 错误，生成 ELF/HEX；
门控测试直接抽取生产启停函数、BLE 回调、入队/消费函数、看门狗及保存结果分支，
模拟 BLE SDK 边界，覆盖初始化前关闭、迟到回调、残留队列、70 次启停及队列回绕、
重复状态、看门狗和保存结果范围。原有启动、息屏、软关机提示、串口 0x03 显示隔离
回归通过。显示回放的采集后端为桩，不能证明实物采集或实际射频启停。

## 实板待验

尚未烧录，蓝牙实际启停、实屏和底板联动尚未验证。最短复测：下载本轮屏幕 HEX 并
校验，开机状态观察已绑定设备数据更新；将通信状态设为不开机并保存，改变传感器
输入确认蓝牙数据停止更新，等待超过 10 秒确认看门狗未恢复采集；重新设为开机并
保存，确认数据恢复。另检查修改后“不保存”、页面切换、息屏/唤醒和绑定保存。
