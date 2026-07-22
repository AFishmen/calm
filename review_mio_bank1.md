## 结论

你的判断是对的：**MIO50 的 GPIO 内存访问目前存在确定性错误**。不过需要区分两套“bank”概念：

* 从封装/供电域看，MIO50 位于通常所说的 **Bank 501**，对应 UG585 中的 MIO voltage bank 1，即 MIO16～53。
* 从 PS GPIO 控制器看，MIO0～31 属于 GPIO Bank 0，MIO32～53 属于 GPIO Bank 1。

因此：

[
\mathrm{MIO50}\rightarrow \mathrm{GPIO\ Bank1,\ bit}=50-32=18
]

AMD 的资料明确给出了这两种不同的分组方式。([AMD 文档][1])

此外，代码中还有几个比 MIO50 更危险的问题，尤其是 **MAX31865 故障位判断错误**。当前版本不建议直接用于无人值守加热控制。

---

# 1. MIO50 的 GPIO bank 访问确实错误

你的代码只定义了一套 Bank 0 寄存器偏移：

```c
#define DIRM_OFFSET     0x204
#define OUTEN_OFFSET    0x208
#define DATA_OFFSET     0x40
#define DATA_RO_OFFSET  0x60
```

随后直接使用：

```c
*dirm |= (1u << pin);
```

对于 MIO50，实际执行的是：

```c
1u << 50
```

但 `1u` 是32位无符号整数，左移50位属于 **C语言未定义行为**。同时，访问的仍然是 GPIO Bank 0 的寄存器。 

正确对应关系是：

| 功能      | GPIO Bank 0 | GPIO Bank 1 |
| ------- | ----------: | ----------: |
| DATA    |      `0x40` |      `0x44` |
| DATA_RO |      `0x60` |      `0x64` |
| DIRM    |     `0x204` |     `0x244` |
| OEN     |     `0x208` |     `0x248` |

AMD 官方驱动定义了 DATA 寄存器每个 bank 间隔 `0x04`，DIRM/OEN 等控制寄存器每个 bank 间隔 `0x40`。Bank 1 的 DATA 和 DIRM 地址分别是 `0xE000A044` 和 `0xE000A244`。([GitHub][2])

### 但 SLCR 的 MIO_PIN 映射本身是正确的

这里：

```c
#define MIO_PIN_BASE 0x700
#define MIO_PIN(n)   (MIO_PIN_BASE + ((n) * 4))
```

对于 MIO50：

```text
0x700 + 50 × 4 = 0x7C8
```

即物理地址：

```text
0xF8000000 + 0x7C8 = 0xF80007C8
```

仍位于你映射的 `0x1000` 字节 SLCR 区域内。因此：

* `SLCR_MAP_SIZE = 0x1000` 没问题；
* `MIO_PIN(50)` 没问题；
* **错误发生在 GPIO 控制器的 bank 选择和 bit 编号上。**

---

## 建议直接替换为 bank-aware 实现

下面这组实现同时支持 MIO0～53，并用 `MASK_DATA` 避免对整个 DATA 寄存器做读改写：

```c
#define GPIO_DATA_RO_OFF(bank)  (0x060U + 0x004U * (bank))
#define GPIO_DIRM_OFF(bank)     (0x204U + 0x040U * (bank))
#define GPIO_OEN_OFF(bank)      (0x208U + 0x040U * (bank))

static volatile uint32_t *gpio_reg(uint32_t offset)
{
    return (volatile uint32_t *)((uint8_t *)g_gpio_map + offset);
}

static int mio_decode(
    uint32_t pin,
    uint32_t *bank,
    uint32_t *bit)
{
    if (pin > 53U || bank == NULL || bit == NULL) {
        return -1;
    }

    *bank = pin / 32U;
    *bit  = pin % 32U;
    return 0;
}

void calm_io_set_dir(uint32_t pin, int is_output)
{
    uint32_t bank;
    uint32_t bit;

    if (g_gpio_map == MAP_FAILED ||
        mio_decode(pin, &bank, &bit) != 0) {
        return;
    }

    volatile uint32_t *dirm = gpio_reg(GPIO_DIRM_OFF(bank));
    volatile uint32_t *oen  = gpio_reg(GPIO_OEN_OFF(bank));

    const uint32_t mask = 1U << bit;

    if (is_output) {
        *dirm |= mask;
        *oen  |= mask;
    } else {
        *oen  &= ~mask;
        *dirm &= ~mask;
    }
}

void calm_io_write(uint32_t pin, int value)
{
    uint32_t bank;
    uint32_t bit;

    if (g_gpio_map == MAP_FAILED ||
        mio_decode(pin, &bank, &bit) != 0) {
        return;
    }

    /*
     * 每个 bank 有两个 MASK_DATA 寄存器：
     * bank0: 0x00 / 0x04
     * bank1: 0x08 / 0x0C
     *
     * 高16位为 mask，mask 位为0表示更新对应输出。
     */
    const uint32_t half = bit / 16U;
    const uint32_t pos  = bit % 16U;
    const uint32_t offset = bank * 0x08U + half * 0x04U;

    const uint32_t update_mask =
        0xFFFFU & ~(1U << pos);

    const uint32_t data =
        value ? (1U << pos) : 0U;

    *gpio_reg(offset) = (update_mask << 16U) | data;
}

int calm_io_read(uint32_t pin)
{
    uint32_t bank;
    uint32_t bit;

    if (g_gpio_map == MAP_FAILED ||
        mio_decode(pin, &bank, &bit) != 0) {
        return 0;
    }

    return ((*gpio_reg(GPIO_DATA_RO_OFF(bank))) &
            (1U << bit)) != 0U;
}
```

AMD 也明确建议使用 `MASK_DATA`，因为它不需要先读取整个 DATA 寄存器再写回，可以避免修改同一 bank 中的其他 GPIO。([AMD 文档][3])

---

# 2. `MIO_GPIO_CFG = 0x1600` 需要谨慎

你的代码把所有管脚统一写成：

```c
#define MIO_GPIO_CFG 0x00001600U
```

并且 MIO8、MIO9～13、MIO50 全部使用同一个配置。

`0x1600` 大致表示：

* GPIO复用；
* IO type 为 LVCMOS 2.5/3.3 V；
* 内部上拉打开；
* `TRI_ENABLE=0`，不强制三态。

这里最重要的问题不是 Bank 0/Bank 1 的寄存器地址，而是：

> **MIO50 所在 Bank501 的实际 VCCO 电压必须与 `IO_Type` 配置一致。**

如果 Bank501 实际是1.8 V，则不应使用 `0x1600`，而应使用相应的 LVCMOS18 配置，例如：

```c
#define MIO_GPIO_OUT_33V  0x00001600U
#define MIO_GPIO_IN_33V   0x00001601U

#define MIO_GPIO_OUT_18V  0x00001200U
#define MIO_GPIO_IN_18V   0x00001201U
```

AMD 特别警告，MIO 输入允许电压取决于 `IO_Type` 等配置，不匹配可能损坏输入缓冲器。([AMD 文档][4])

建议至少将输入和输出分开：

```c
/* 示例：假设相关 MIO bank 确实供电为3.3V */
*mio_mosi = MIO_GPIO_OUT_33V;
*mio_sck  = MIO_GPIO_OUT_33V;
*mio_cs1  = MIO_GPIO_OUT_33V;
*mio_cs2  = MIO_GPIO_OUT_33V;
*mio_pwm1 = MIO_GPIO_OUT_33V;
*mio_pwm2 = MIO_GPIO_OUT_33V;

*mio_miso = MIO_GPIO_IN_33V;
```

MIO8 只能作为输出使用，而你的用途正好是 PWM 输出，这一点没有问题。([AMD 文档][5])

---

# 3. MAX31865 的故障位判断是错误的

这是当前代码中另一个确定性问题。

代码写的是：

```c
if (msb & 0x80) {
    // fault
}
```

并且组合ADC时：

```c
uint16_t adc_full =
    ((uint16_t)(msb & 0x7F) << 8) | lsb;
adc_full >>= 1;
```



但 MAX31865 的实际格式是：

* RTD MSB 的 `D7` 是正常的 ADC 最高有效位，即 ADC bit 14；
* 故障标志位位于 RTD LSB 的 `D0`。

因此正确判断应为：

```c
if (lsb & 0x01U) {
    /* RTD fault */
}
```

正确组合应为：

```c
uint16_t adc =
    (((uint16_t)msb << 8) | lsb) >> 1;
```

ADI 数据手册明确指出，RTD MSB D7 权重为 (2^{14})，而 RTD LSB D0 才是 Fault 位。

在你当前PT100常温测量范围内，最高ADC位通常不会置位，所以“屏蔽 MSB D7”的数值错误可能暂时不明显；但是：

* 真正的故障位没有被正确检测；
* PT100短路、断路或接线异常时，控制器可能继续使用错误温度；
* 错误的低温结果可能直接让PID输出100%加热。

这是加热控制中必须做成 **故障闭锁** 的部分：检测到传感器故障后，应立即将对应 PWM 拉低，而不是清除故障后继续返回本次错误采样。

---

## MSB和LSB应在一次CS事务中读取

当前代码分别调用两次：

```c
msb = max31865_read_reg(..., 0x01);
lsb = max31865_read_reg(..., 0x02);
```

两次调用之间 CS 会拉高。如果连续转换恰好在两次读取之间更新，可能得到来自不同转换周期的 MSB 和 LSB。

建议使用一次连续读取：

```c
int max31865_read_rtd(
    uint32_t cs_pin,
    float *resistance)
{
    if (resistance == NULL) {
        return -1;
    }

    calm_io_write(cs_pin, 0);

    spi_transfer_byte(MAX31865_REG_RTD_MSB);
    uint8_t msb = spi_transfer_byte(0x00);
    uint8_t lsb = spi_transfer_byte(0x00);

    calm_io_write(cs_pin, 1);

    if ((lsb & 0x01U) != 0U) {
        uint8_t fault =
            max31865_read_reg(
                cs_pin,
                MAX31865_REG_FAULT_STAT);

        fprintf(stderr,
            "[MAX31865] CS%u fault: 0x%02X\n",
            cs_pin,
            fault);

        return -1;
    }

    uint16_t adc =
        (((uint16_t)msb << 8) | lsb) >> 1;

    *resistance =
        ((float)adc * PT100_REF_RESISTOR) /
        32768.0f;

    return 0;
}
```

控制线程中应采用类似逻辑：

```c
float resistance;

if (max31865_read_rtd(cs_pin, &resistance) != 0) {
    /* 传感器故障：禁止加热 */
    channel.enabled = false;
    channel.current_duty = 0.0f;
    calm_io_write(pwm_pin, 0);
}
```

---

# 4. MAX31865 初始化回读会出现假 `MISMATCH`

初始化配置包含：

```c
MAX31865_CONFIG_FAULT_CLR
```

即 D1：

```c
uint8_t config =
    VBIAS | AUTO | 50HZ | FAULT_CLR;
```

随后又比较：

```c
readback == config
```



但 Fault Clear D1 是 **自清零位**。写入1后，芯片会自动清回0。因此正常回读时，这一位很可能已经为0，导致程序错误报告：

```text
MISMATCH!
```

ADI 数据手册明确说明 D1 自动清零。

应改为：

```c
uint8_t expected =
    config & ~MAX31865_CONFIG_FAULT_CLR;

if (readback != expected) {
    /* 真正的配置错误 */
}
```

---

# 5. `delay_us(65000)` 不适合等待转换完成

你的 `delay_us()` 是根据“CPU大约667 MHz，每微秒100次循环”估算的：

```c
volatile unsigned int count = us * 100;
while (count--) {
    asm volatile("nop");
}
```

这在 Linux 下不可靠：

* CPU可能动态调频；
* 编译器和指令流水线会改变每轮时间；
* 线程可能被调度出去；
* 100次循环不一定正好是1 µs；
* `delay_us(65000)` 可能实际远小于65 ms。

MAX31865 在50 Hz滤波模式下一次转换约需62.5 ms，还需要考虑输入RC网络稳定时间。

对于65 ms这种长延时，直接使用：

```c
usleep(70000);
```

或者：

```c
struct timespec ts = {
    .tv_sec = 0,
    .tv_nsec = 70L * 1000L * 1000L
};
nanosleep(&ts, NULL);
```

更理想的是接出 `DRDY` 并读取其状态。

SPI单个位延时则最好使用：

* PS硬件 SPI 控制器；
* Linux `spidev`；
* 或基于单调时钟的延时。

---

# 6. 当前 PWM 周期不是10 ms，而是1秒

控制线程每10 ms执行一次：

```c
tick = (tick + 1) % 100;
```

PWM判断为：

```c
tick < act
```

所以完整PWM周期是：

[
100\times10\ \mathrm{ms}=1\ \mathrm{s}
]

PID也只在：

```c
if (tick == 0)
```

时计算，因此 PID更新周期同样约为1秒。

也就是说，结合你之前希望的“10 ms PWM周期”，当前实现的是：

* 时间片：10 ms；
* PWM周期：1 s；
* PWM频率：1 Hz；
* 占空比分辨率：1%；
* PID周期：约1 s。

如果使用机械继电器或低频SSR，1秒周期可能合理；如果使用MOSFET并确实希望100 Hz PWM，则这段逻辑不符合要求。

---

## PID还缺少实际时间间隔

当前积分和微分是：

```c
pid.integral += err;
float deriv = err - pid.last_error;
```



严格写法应为：

```c
pid.integral += err * dt;
float deriv = (err - pid.last_error) / dt;
```

否则：

* PID参数强依赖固定1秒更新周期；
* Linux调度造成的周期波动不会被补偿；
* 后续将PID周期改为其他值时，参数含义会完全变化。

另外还应在以下情况重置积分与微分状态：

* 通道从 disabled 变为 enabled；
* 从手动模式切回PID模式；
* 修改目标温度；
* 传感器故障恢复。

否则可能发生重新启用后占空比突跳。

---

# 7. 重新启用时可能先输出旧占空比

`set_enable1(true)` 只修改：

```c
_ch1.enabled = true;
```

并没有清空旧占空比。

如果通道之前的 `current_duty` 是100%，重新启用发生在两个 `tick==0` 之间，那么在下一次PID计算之前，旧的100%占空比可能被继续使用，最长接近1秒。

建议关闭时直接执行：

```c
_ch.enabled = false;
_ch.current_duty = 0.0f;
_ch.pid.integral = 0.0f;
_ch.pid.last_error = 0.0f;
```

重新启用后，只有拿到一次有效温度并完成PID计算，才允许PWM输出。

---

# 8. 硬件初始化失败没有阻止程序运行

构造函数中：

```cpp
CalmController::CalmController() : _running(false) {
    calm_hw_init();
}
```

完全忽略了 `calm_hw_init()` 的返回值。随后 `main()` 无条件启动控制线程和网络服务器。 

如果：

* `/dev/mem` 打开失败；
* SLCR mmap失败；
* GPIO mmap失败；

服务器仍会响应 `OK`，但实际硬件可能完全没有动作。

构造函数应检查并抛出异常，或者保存初始化状态：

```cpp
CalmController::CalmController()
    : _running(false)
{
    if (calm_hw_init() != 0) {
        throw std::runtime_error(
            "CALM hardware initialization failed");
    }
}
```

同时 `calm_hw_init()` 的部分失败路径应关闭已经打开的资源。当前 mmap 失败后直接 `return -1`，没有清理，而且 `g_memfd` 已经大于等于0，下一次调用会错误地直接返回成功。

---

# 9. 网络层存在几个明显问题

## `SET_EN` 参数缺失时使用未初始化变量

当前代码：

```cpp
int en;
iss >> en;

if (ch == 1) {
    _ctrl.set_enable1(en > 0);
}
```

如果发送：

```text
SET_EN 1
```

`iss >> en` 会失败，`en` 保持未初始化，继续读取它属于未定义行为。

应改为：

```cpp
int en;

if (!(iss >> en)) {
    return "ERROR: Missing enable value";
}

if (en != 0 && en != 1) {
    return "ERROR: Enable must be 0 or 1";
}
```

## 把一次 `read()` 当成一条完整命令

TCP是字节流，可能出现：

* 一条命令被拆成两次 `read()`；
* 多条命令一次性进入一个 `read()`；
* 当前代码只处理第一个换行符之前的内容，后面的命令会被丢弃。

现有逻辑确实直接将每次 `read()` 结果构造成一条命令。

应维护一个接收缓存，持续按 `\n` 拆包。

## 其他网络问题

当前服务器还存在：

* `socket()`、`bind()`、`listen()`、`write()` 返回值均未检查；
* `write()` 不保证一次发送全部数据；
* 客户端异常断开可能产生 `SIGPIPE`；
* 一个客户端长时间保持连接会阻塞其他客户端；
* 绑定 `INADDR_ANY` 且没有认证，网络中任何可访问8080端口的设备都能修改目标温度和占空比。

对于实际加热器，至少应：

* 限制绑定地址或使用防火墙；
* 加入简单认证；
* 加客户端超时；
* 对目标温度、PID参数、占空比进行范围和 `isfinite()` 检查。

---

# 10. 程序目前按上传文件实际编译会失败

我按你给的 Makefile 实际编译，首先出现：

```text
calm_ctrl.cpp:127:13:
error: 'printf' was not declared in this scope
```

`calm_ctrl.cpp` 使用了 `printf()`，但只包含了：

```cpp
#include <chrono>
#include <algorithm>
```

 

增加：

```cpp
#include <cstdio>
```

后可以继续编译。`calm_net.cpp` 同样建议显式加入：

```cpp
#include <cstdio>
#include <cerrno>
#include <cstring>
```

另外，上传文件名是：

```text
calm_hw(2).c
calm_hw(2).h
```

但 Makefile 和源代码引用的是：

```text
calm_hw.c
calm_hw.h
```

如果只是上传时自动增加的重名后缀，可以忽略；否则需要重命名。

---

# 11. 进程退出时的加热器安全

正常调用析构函数时，`stop()` 会等待线程结束并将两路PWM拉低，这部分设计是正确的。

但服务器是无限循环，通常由 `SIGINT`、`SIGTERM` 或进程崩溃结束。默认信号终止并不保证执行C++析构函数，因此 GPIO 可能停留在之前的高电平。

建议至少加入：

* `SIGINT`/`SIGTERM` 退出处理；
* systemd 的停止流程；
* 加热器使能端硬件下拉；
* 外部看门狗或最大连续导通限制；
* 独立的过温断路器。

软件故障闭锁应遵循：

```text
传感器故障
网络超时
温度越界
温升速率异常
控制线程异常
程序准备退出
        ↓
PWM立即拉低并锁定
```

---

## 修改优先级

建议按下面顺序处理：

1. **修复 MIO50 的 GPIO Bank 1 映射和 bit18访问。**
2. **修复 MAX31865 故障位与ADC组合逻辑。**
3. **传感器异常时强制PWM为0并故障闭锁。**
4. **确认 Bank500/501 实际VCCO，修正 `0x1600` 的电平类型。**
5. **明确到底需要1秒还是10 ms的PWM周期。**
6. **加入PID的实际 `dt`、重新启用清零和温度范围检查。**
7. **修复网络拆包、参数检查、认证和退出安全。**
8. **补充 `<cstdio>` 并处理硬件初始化失败。**

其中前四项属于上电连接加热器之前必须修正的问题。

[1]: https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/PS-MIO-I/Os "https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/PS-MIO-I/Os"
[2]: https://raw.githubusercontent.com/Xilinx/embeddedsw/master/XilinxProcessorIPLib/drivers/gpiops/src/xgpiops_hw.h "https://raw.githubusercontent.com/Xilinx/embeddedsw/master/XilinxProcessorIPLib/drivers/gpiops/src/xgpiops_hw.h"
[3]: https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/GPIO-Control-of-Device-Pins "https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/GPIO-Control-of-Device-Pins"
[4]: https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/Register-MIO_PIN_00-Details "https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/Register-MIO_PIN_00-Details"
[5]: https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/MIO-at-a-Glance-Table "https://docs.amd.com/r/en-US/ug585-zynq-7000-SoC-TRM/MIO-at-a-Glance-Table"
