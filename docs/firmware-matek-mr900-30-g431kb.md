# Matek mR900-30 G431KB 统一透明串口固件使用文档

适用固件：`firmware-matek-mr900-30-g431kb`

本固件用于两端同一硬件的透明串口数传，不区分 TX/RX，模块上电后自动配对并工作。

---

**1. 硬件端口与波特率**

端口角色
1. UART1 (PA9/PA10)：数传数据口。
2. USB CDC：数传数据口。USB CDC 的 DTR=1 时优先走 USB，否则走 UART1。
3. UART2 (PA2/PA3)：配置口（CLI）。

默认波特率
1. UART2 (CLI)：`115200 8N1` 固定。
2. UART1/USB CDC（数传）：默认 `57600 8N1`，可用 CLI 参数 `RX_SER_BAUD` 修改。

RX_SER_BAUD 取值索引
1. `0 = 9600`
2. `1 = 19200`
3. `2 = 38400`
4. `3 = 57600`（默认）
5. `4 = 115200`
6. `5 = 230400`

---

**2. 默认无线参数**

当前固件默认值（首次刷写后第一次启动生效）
1. 频段：915 MHz FCC
2. 模式：19 Hz
3. 功率：1 W（30 dBm）
4. Ortho：off

说明
1. 这些默认值只在“刷入新编译固件后首次启动”时生效。
2. 若再次刷同一个 `.hex`，不会重置；需要重新编译才能再次强制恢复默认。

---

**3. 编译流程**

前置条件
1. 已安装 GNU Arm 工具链：`/opt/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi`
2. 在工程根目录执行命令。

步骤
1. 复制 ST 驱动（首次或清理后需要）
```bash
python3 tools/run_copy_st_drivers.py -t firmware-matek-mr900-30-g431kb -s
```

2. 编译固件（会自动生成 `.hex/.bin/.dfu`）
```bash
ARM_GNU_TOOLCHAIN=/opt/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi \
  python3 tools/run_make_firmwares.py -t firmware-matek-mr900-30-g431kb -np
```

输出文件路径
1. `tools/build/firmware-matek-mr900-30-g431kb/firmware-matek-mr900-30-g431kb-*.hex`
2. `tools/build/firmware-matek-mr900-30-g431kb/firmware-matek-mr900-30-g431kb-*.bin`
3. `tools/build/firmware-matek-mr900-30-g431kb/firmware-matek-mr900-30-g431kb-*.dfu`
4. 同样会复制到 `tools/build/firmware/` 目录。

---

**4. 烧录流程**

方式 A：ST-LINK
1. 打开 STM32CubeProgrammer。
2. 选择 ST-LINK 连接。
3. 选择 `.hex` 或 `.bin` 烧录到 `0x08000000`。
4. 复位运行。

方式 B：USB DFU
1. 让模块进入系统 Bootloader（可在 CLI 输入 `systemboot`，或使用硬件 BOOT 方式）。
2. 在 STM32CubeProgrammer 里选择 USB DFU 连接。
3. 选择 `.dfu` 文件烧录。

方式 C：Windows 10 自动化顶针烧录（ST-LINK + CLI）
说明
1. 适合批量烧录：顶针接触 SWD 后自动烧录，移开后等待下一块。
2. 依赖 STM32CubeProgrammer CLI。

步骤
1. 安装 STM32CubeProgrammer。
2. 打开 PowerShell。
3. 执行脚本（`.bin` 或 `.hex` 都支持）：
```powershell
# 单次烧录
powershell -ExecutionPolicy Bypass -File tools\flash_win_stlink.ps1 -Firmware "C:\path\to\firmware.bin" -Once

# 连续烧录（顶针批量）
powershell -ExecutionPolicy Bypass -File tools\flash_win_stlink.ps1 -Firmware "C:\path\to\firmware.bin"
```

---

**5. 使用流程**

1. 两端模块刷同一固件后上电。
2. 自动发现对端并建立链路（无需区分 TX/RX）。
3. 串口数据接入 UART1 或 USB CDC。
4. 需要修改配置时使用 UART2 进入 CLI。

**6. 配对流程（随机配对短语）**

说明
1. 配对会生成随机 `BIND_PHRASE`，用于区分多对设备、降低同场干扰。
2. 两端需要在 15 秒内同时进入配对模式。
3. 配对成功后模块会自动重启并保存新的 `BIND_PHRASE`。

进入配对模式的方法
1. 按住模块按键约 2 秒，松开。
2. 或在 UART2 CLI 中执行 `bind`。

配对指示
1. 进入配对后红绿灯会快速交替闪烁（绑定模式指示）。

---

**7. CLI 使用与配置**

命令概览
1. `help` / `?`：帮助
2. `v`：版本信息
3. `pl` / `pl c`：列出参数
4. `p NAME`：读取参数
5. `p NAME = ?`：查看允许值
6. `p NAME = value`：设置参数
7. `pstore`：保存到 Flash
8. `reload`：重新加载（放弃未保存修改）
9. `bind`：进入配对模式（两端都需要执行）
10. `stats`：链路统计
11. `listfreqs`：FHSS 频点
12. `systemboot`：进入系统 Bootloader

参数名规则
1. 显示名空格改下划线，大小写不敏感。
2. LIST 类型必须用索引数字设置。
3. Bind Phrase 为 6 字符，字符集 `[a-z0-9#-._]`。

可用参数（透明数传相关）
1. `BIND_PHRASE`：配对短语
2. `MODE`：模式（19 Hz/31 Hz/50 Hz 等）
3. `RF_BAND`：频段（915/868 等）
4. `RF_ORTHO`：Ortho
5. `RX_POWER`：发射功率
6. `RX_DIVERSITY`：天线分集
7. `RX_SER_BAUD`：数传口波特率

示例
```text
p RF_BAND = ?
p RF_BAND = 1
p MODE = ?
p MODE = 2
p RX_POWER = ?
p RX_POWER = 4
p RX_SER_BAUD = 4
pstore
```

说明
1. 两端模块必须使用一致的 `BIND_PHRASE / MODE / RF_BAND`。
2. 本固件已隐藏/锁死所有非透明传输相关参数。

---

**8. 常见问题**

1. 重新刷固件后是否恢复默认？
是。每次重新编译生成的新固件，首次启动都会恢复默认。

2. 刷同一个 `.hex` 是否恢复默认？
不会。需要重新编译生成新固件。

3. USB CDC 为什么不工作？
需要主机打开串口并把 DTR 拉高，USB CDC 才会被优先使用。
