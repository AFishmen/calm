# CALM — Closed-Loop Temperature Controller for Zynq SoC

**CALM** (Controller for Active Limiting of Magnetic-drift) is a temperature control system designed to run on the **Xilinx Zynq** platform. It implements a PID-based closed-loop heater controller with a software PWM driver, a SPI-based dual-channel MAX31865 RTD-to-digital converter interface for PT100 sensors, and a TCP command server for remote monitoring and tuning.

## Features

- **Dual-Channel Independent PID Control** — Two independent PID loops (one per sensor), each with its own target, enable, gains, and PWM output
- **Software PWM Generation** — 1-second period, 100-step (10 ms) resolution, two independent PWM outputs (MIO08 / MIO50)
- **SPI Dual-Channel Temperature Sensing** — Bit-bang SPI interface for two MAX31865 chips, reading PT100 (RTD) temperature sensors with Callendar-Van Dusen conversion
- **Manual Override Mode** — Bypass PID and set heater duty cycle directly (0.0–1.0)
- **Remote Control via TCP** — Command-line interface over TCP port 8080
- **Thread-safe Design** — Mutex-protected state shared between control and network threads
- **Hardware Memory Mapping** — Direct `/dev/mem` access to Zynq SLCR and GPIO registers

## Project Structure

```
.
└── src/
    ├── main.cpp          # Entry point: starts controller & TCP server
    ├── Makefile          # Build system (g++ / gcc, links -lm -pthread)
    ├── calm_hw.h         # Hardware abstraction header (MIO pin definitions, MAX31865 constants)
    ├── calm_hw.c         # Zynq memory-mapped GPIO/SLCR driver, SPI bit-bang, MAX31865 interface
    ├── calm_ctrl.hpp     # Controller class declaration (dual-channel temperature)
    ├── calm_ctrl.cpp     # PID control loop, PWM generation, SPI temperature reading
    ├── calm_net.hpp      # TCP server class declaration
    └── calm_net.cpp      # TCP command handler (STATUS with dual temps, SET_*, etc.)
```

## Hardware Platform

The code is tailored for the **Xilinx Zynq-7000** series (e.g., Z-7010, Z-7020) running Linux (PetaLinux or similar). Memory-mapped I/O is used to control:

### MIO Pin Assignments

| MIO Pin | Signal       | Function                        |
|---------|-------------|---------------------------------|
| 8       | PID1 PWM    | Channel 1 heater output (MIO08) |
| 50      | PID2 PWM    | Channel 2 heater output (MIO50) |
| 9       | SPI_CS2     | MAX31865 #2 chip select (CS2)   |
| 10      | SPI1_MOSI   | SPI master-out-slave-in         |
| 11      | SPI1_MISO   | SPI master-in-slave-out         |
| 12      | SPI1_SCK    | SPI clock (mode 3: CPOL=1, CPHA=1) |
| 13      | SPI_CS1     | MAX31865 #1 chip select (CS1)   |

### Sensors

- **MAX31865** RTD-to-digital converters (2×) for PT100 platinum resistance thermometers
- 2-wire PT100 configuration (FORCE+ tied to RTDIN+, FORCE- tied to RTDIN-)
- Reference resistor: **430 Ω** (default `PT100_REF_RESISTOR` in `calm_hw.h`)
- Temperature conversion uses the **Callendar-Van Dusen** equation
- 50 Hz notch filter enabled for mains noise rejection

## Build & Run

### Prerequisites

- ARM/Linux cross-compilation toolchain (or native `g++`/`gcc` on the Zynq target)
- Root access on the target (required for `/dev/mem`)
- Math library (`-lm`) linked for floating-point operations

### Build

```bash
cd src
make
```

This produces the `calm_server` binary.

### Run

```bash
./calm_server
```

The server starts listening on **TCP port 8080**.

## Network Protocol

Connect via any TCP client (e.g., `netcat`, `telnet`, or a custom script):

```bash
nc <target-ip> 8080
```

If you need to test locally, use the following command:
```bash
nc 127.0.0.1 8080
```

### Commands

| Command                         | Description                        | Example Response         |
|---------------------------------|------------------------------------|--------------------------|
| `STATUS`                        | Get both channels status (JSON)    | `{"ch1":{...},"ch2":{...}}` |
| `SET_EN <1\|2> <0\|1>`         | Enable / disable channel          | `OK`                     |
| `SET_TARGET <1\|2> <temp>`      | Set channel target temperature     | `OK`                     |
| `SET_DUTY <1\|2> <duty>`        | Manual duty override per channel   | `OK`                     |
| `SET_PID <1\|2> <P> <I> <D>`    | Tune PID gains per channel         | `OK`                     |
| *anything else*                 | Unknown command                    | `ERROR: Unknown Command` |

## Controller Internals

### Control Thread

- Runs at a **10 ms** tick rate (100 ticks = 1 second full cycle)
- Every **1 second** (tick 0):
  - Reads temperature from both MAX31865 channels via SPI bit-bang
  - Computes independent PID output for each enabled channel
  - Each channel has its own target temperature, PID gains, and manual_mode flag
  - Prints debug info to stdout (both channels)
- Every tick:
  - Compares current tick index against `duty × 100` per channel
  - Drives PWM1 (MIO08) and PWM2 (MIO50) independently

### SPI Bit-Bang Protocol

The SPI interface is implemented using manual GPIO toggling (bit-bang):

- **Mode 3**: CPOL=1 (SCK idle high), CPHA=1 (data sampled on rising edge, changed on falling edge)
- **MSB first** byte transfer
- CS lines are driven active-low before each register access and released after

See `spi_transfer_byte()` in `calm_hw.c` for the detailed timing implementation.

### MAX31865 Read Sequence

1. Pull CS low
2. Send read register address (bit 7 = 0)
3. Read response byte(s)
4. Pull CS high

RTD resistance is calculated as:  
`R_RTD = 14-bit_data × R_REF / 16384`

### Temperature Conversion (Callendar-Van Dusen)

For temperatures ≥ 0°C:  
`R(t) = R₀ × (1 + A×t + B×t²)`  
where R₀ = 100 Ω, A = 3.9083×10⁻³, B = -5.775×10⁻⁷

### PID Algorithm

```
error = target_temp - current_temp
integral = clamp(integral + error, -10.0, +10.0)   // anti-windup
derivative = error - last_error
output = Kp × error + Ki × integral + Kd × derivative
output = clamp(output, 0.0, 1.0)                   // to valid duty range
```

Default gains: `Kp = 1.0`, `Ki = 0.1`, `Kd = 0.05`

## License

This project is provided as-is for educational and evaluation purposes.

---

# CALM — 基于 Zynq SoC 的闭环温度控制器

**CALM**（磁漂移主动限制控制器）是一个温度控制系统，设计用于在 **Xilinx Zynq** 平台上运行。它实现了一个基于 PID 的闭环加热器控制器，包含软件 PWM 驱动器、基于 SPI 的双通道 MAX31865 RTD 数字转换器接口（适配 PT100 温度传感器），以及用于远程监控和调参的 TCP 命令服务器。

## 功能特性

- **双通道独立 PID 控制** — 两路独立 PID 回路（每路一个传感器），各自拥有独立的目标温度、使能开关、PID 增益和 PWM 输出
- **软件 PWM 生成** — 1 秒周期，100 步（10 ms）分辨率，两路独立 PWM 输出（MIO08 / MIO50）
- **SPI 双通道温度采集** — 位脉冲 SPI 接口，连接两片 MAX31865 芯片，读取 PT100（RTD）温度传感器，使用 Callendar-Van Dusen 方程转换温度
- **手动覆盖模式** — 绕过 PID，直接设置加热器占空比（0.0–1.0）
- **通过 TCP 远程控制** — 通过 TCP 端口 8080 提供命令行接口
- **线程安全设计** — 控制线程和网络线程之间共享的状态通过互斥锁保护
- **硬件内存映射** — 直接通过 `/dev/mem` 访问 Zynq SLCR 和 GPIO 寄存器

## 项目结构

```
.
└── src/
    ├── main.cpp          # 入口点：启动控制器和 TCP 服务器
    ├── Makefile          # 构建系统（g++ / gcc，链接 -lm -pthread）
    ├── calm_hw.h         # 硬件抽象头文件（MIO 引脚定义、MAX31865 常量）
    ├── calm_hw.c         # Zynq 内存映射 GPIO/SLCR 驱动、SPI 位脉冲、MAX31865 接口
    ├── calm_ctrl.hpp     # 控制器类声明（双通道温度）
    ├── calm_ctrl.cpp     # PID 控制循环、PWM 生成、SPI 温度读取
    ├── calm_net.hpp      # TCP 服务器类声明
    └── calm_net.cpp      # TCP 命令处理器（STATUS 含双路温度、SET_* 等）
```

## 硬件平台

该代码专为 **Xilinx Zynq-7000** 系列（例如 Z-7010、Z-7020）设计，运行 Linux（PetaLinux 或类似系统）。通过内存映射 I/O 来控制：

### MIO 引脚分配

| MIO 引脚 | 信号      | 功能                        |
|---------|----------|-----------------------------|
| 8       | PID1 PWM    | 通道 1 加热器输出 (MIO08)       |
| 50      | PID2 PWM    | 通道 2 加热器输出 (MIO50)       |
| 9       | SPI_CS2     | MAX31865 #2 片选 (CS2)       |
| 10      | SPI1_MOSI   | SPI 主机输出从机输入            |
| 11      | SPI1_MISO   | SPI 主机输入从机输出            |
| 12      | SPI1_SCK    | SPI 时钟（模式 3：CPOL=1, CPHA=1） |
| 13      | SPI_CS1     | MAX31865 #1 片选 (CS1)       |

### 传感器

- **MAX31865** RTD 数字转换器（×2），用于 PT100 铂电阻温度计
- 2 线制 PT100 配置（FORCE+ 连接 RTDIN+，FORCE- 连接 RTDIN-）
- 参考电阻：**430 Ω**（默认值 `PT100_REF_RESISTOR`，定义在 `calm_hw.h` 中）
- 温度转换使用 **Callendar-Van Dusen** 方程
- 启用 50 Hz 陷波滤波器以抑制电网噪声

## 构建与运行

### 前置条件

- ARM/Linux 交叉编译工具链（或在 Zynq 目标上使用原生 `g++`/`gcc`）
- 目标设备上的 root 访问权限（访问 `/dev/mem` 所需）
- 需要链接数学库（`-lm`）用于浮点运算

### 构建

```bash
cd src
make
```

这将生成 `calm_server` 二进制文件。

### 运行

```bash
./calm_server
```

服务器开始在 **TCP 端口 8080** 上监听。

## 网络协议

通过任意 TCP 客户端连接（例如 `netcat`、`telnet` 或自定义脚本）：

```bash
nc <target-ip> 8080
```

如果需要本地测试，请使用以下命令：
```bash
nc 127.0.0.1 8080
```

### 命令

| 命令                           | 描述                              | 示例响应                  |
|---------------------------------|------------------------------------|--------------------------|
| `STATUS`                        | 获取两路通道状态（JSON）           | `{"ch1":{...},"ch2":{...}}` |
| `SET_EN <1\|2> <0\|1>`         | 启用/禁用指定通道                  | `OK`                     |
| `SET_TARGET <1\|2> <temp>`      | 设置通道目标温度                   | `OK`                     |
| `SET_DUTY <1\|2> <duty>`        | 通道手动占空比覆盖（0.0–1.0）       | `OK`                     |
| `SET_PID <1\|2> <P> <I> <D>`    | 调节通道 PID 增益系数              | `OK`                     |
| *其他任何输入*                   | 未知命令                           | `ERROR: Unknown Command` |

## 控制器内部原理

### 控制线程

- 以 **10 ms** 为节拍速率运行（100 个节拍 = 1 秒完整周期）
- 每 **1 秒**（节拍 0）：
  - 通过 SPI 位脉冲从两个 MAX31865 通道读取温度
  - 为每路已启用的通道独立计算 PID 输出
  - 每个通道有独立的目标温度、PID 增益和手动模式标志
  - 将调试信息打印到标准输出（两路通道）
- 每个节拍：
  - 分别比较当前节拍索引与各通道 `duty × 100`
  - 独立驱动 PWM1 (MIO08) 和 PWM2 (MIO50)

### SPI 位脉冲协议

SPI 接口通过手动 GPIO 翻转（位脉冲）实现：

- **模式 3**：CPOL=1（SCK 空闲高电平），CPHA=1（数据在上升沿采样，下降沿改变）
- **MSB 先** 进行字节传输
- 每次寄存器访问前拉低 CS 线（低有效），访问结束后释放

详见 `calm_hw.c` 中的 `spi_transfer_byte()` 函数实现。

### MAX31865 读取流程

1. 拉低 CS
2. 发送读寄存器地址（bit 7 = 0）
3. 读取返回数据字节
4. 拉高 CS

RTD 电阻计算公式：  
`R_RTD = 14位数据 × R_REF / 16384`

### 温度转换（Callendar-Van Dusen 方程）

温度 ≥ 0°C 时：  
`R(t) = R₀ × (1 + A×t + B×t²)`  
其中 R₀ = 100 Ω，A = 3.9083×10⁻³，B = -5.775×10⁻⁷

### PID 算法

```
error = target_temp - current_temp
integral = clamp(integral + error, -10.0, +10.0)   // 抗饱和
derivative = error - last_error
output = Kp × error + Ki × integral + Kd × derivative
output = clamp(output, 0.0, 1.0)                   // 限制到有效的占空比范围
```

默认增益系数：`Kp = 1.0`、`Ki = 0.1`、`Kd = 0.05`

## 许可证

本项目按原样提供，用于教育和评估目的。
