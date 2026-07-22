// calm_hw.c
#define _GNU_SOURCE
#include "calm_hw.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>

/* Zynq 物理地址常量 */
#define SLCR_BASE       0xF8000000UL
#define SLCR_MAP_SIZE   0x1000
#define SLCR_UNLOCK_OFF 0x08
#define SLCR_LOCK_OFF   0x04
#define SLCR_UNLOCK_KEY 0x0000DF0D
#define SLCR_LOCK_KEY   0x0000767B
#define MIO_PIN_BASE    0x700
#define MIO_PIN(n)      (MIO_PIN_BASE + ((n) * 4))
#define MIO_GPIO_CFG    0x00001600U 

#define GPIO_BASE       0xE000A000UL
#define GPIO_MAP_SIZE   0x1000
#define DIRM_OFFSET     0x204
#define OUTEN_OFFSET    0x208
#define DATA_OFFSET     0x40
#define DATA_RO_OFFSET  0x60

static int g_memfd = -1;
static void *g_slcr_map = MAP_FAILED;
static void *g_gpio_map = MAP_FAILED;

static void *map_phys_region(int fd, off_t phys_base, size_t map_size) {
    void *m = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_base);
    return (m == MAP_FAILED) ? MAP_FAILED : m;
}

int calm_hw_init(void) {
    if (g_memfd >= 0) return 0;

    g_memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_memfd < 0) {
        perror("Failed to open /dev/mem");
        return -1;
    }
    
    g_slcr_map = map_phys_region(g_memfd, SLCR_BASE, SLCR_MAP_SIZE);
    g_gpio_map = map_phys_region(g_memfd, GPIO_BASE, GPIO_MAP_SIZE);
    
    if (g_slcr_map == MAP_FAILED || g_gpio_map == MAP_FAILED) return -1;
    
    printf("SUCCESS: Hardware memory mapped perfectly.\n");
    
    /* 解锁 SLCR 并配置管脚复用 */
    volatile uint32_t *slcr_unlock = (volatile uint32_t *)((uint8_t*)g_slcr_map + SLCR_UNLOCK_OFF);
    volatile uint32_t *slcr_lock   = (volatile uint32_t *)((uint8_t*)g_slcr_map + SLCR_LOCK_OFF);
    
    *slcr_unlock = SLCR_UNLOCK_KEY;
    
    /* 配置所有使用到的 MIO 管脚为 GPIO:
     * MIO08 → PID1 加热器 PWM, MIO50 → PID2 加热器 PWM,
     * MIO09 → SPI_CS2, MIO10 → SPI_MOSI,
     * MIO11 → SPI_MISO, MIO12 → SPI_SCK, MIO13 → SPI_CS1 */
    uint32_t pins[] = {MIO_HEATER_PWM, MIO_HEATER_PWM2, MIO_SPI_CS2,
                       MIO_SPI_MOSI, MIO_SPI_MISO, MIO_SPI_SCK, MIO_SPI_CS1};
    for (int i = 0; i < 7; ++i) {
        volatile uint32_t *mio_reg = (volatile uint32_t *)((uint8_t*)g_slcr_map + MIO_PIN(pins[i]));
        *mio_reg = MIO_GPIO_CFG;
    }
    
    *slcr_lock = SLCR_LOCK_KEY;

    /* 初始化 SPI 管脚方向 */
    calm_io_set_dir(MIO_SPI_MOSI, 1);  // 输出
    calm_io_set_dir(MIO_SPI_SCK,  1);  // 输出
    calm_io_set_dir(MIO_SPI_MISO, 0);  // 输入
    calm_io_set_dir(MIO_SPI_CS1,  1);  // 输出 (高电平无效)
    calm_io_set_dir(MIO_SPI_CS2,  1);  // 输出 (高电平无效)

    /* 初始化 SPI 管脚空闲状态 (Mode 3: SCK 空闲 = 高电平) */
    calm_io_write(MIO_SPI_SCK,  1);    // SCK 高电平 (CPOL=1)
    calm_io_write(MIO_SPI_MOSI, 0);    // MOSI 低电平
    calm_io_write(MIO_SPI_CS1,  1);    // CS1 高电平 (无效)
    calm_io_write(MIO_SPI_CS2,  1);    // CS2 高电平 (无效)

    /* 默认初始化两路加热器 PWM 为输出并拉低 */
    calm_io_set_dir(MIO_HEATER_PWM, 1);
    calm_io_write(MIO_HEATER_PWM, 0);
    calm_io_set_dir(MIO_HEATER_PWM2, 1);
    calm_io_write(MIO_HEATER_PWM2, 0);

    /* 初始化两路 MAX31865 */
    max31865_init(MIO_SPI_CS1);
    max31865_init(MIO_SPI_CS2);

    return 0;
}

void calm_io_set_dir(uint32_t pin, int is_output) {
    if (g_gpio_map == MAP_FAILED) return;
    volatile uint32_t *dirm  = (volatile uint32_t *)((uint8_t*)g_gpio_map + DIRM_OFFSET);
    volatile uint32_t *outen = (volatile uint32_t *)((uint8_t*)g_gpio_map + OUTEN_OFFSET);
    
    if (is_output) {
        *dirm  |= (1u << pin);
        *outen |= (1u << pin);
    } else {
        *dirm  &= ~(1u << pin);
        *outen &= ~(1u << pin);
    }
}

void calm_io_write(uint32_t pin, int value) {
    if (g_gpio_map == MAP_FAILED) return;
    volatile uint32_t *data_reg = (volatile uint32_t *)((uint8_t*)g_gpio_map + DATA_OFFSET);
    if (value) *data_reg |= (1u << pin);
    else       *data_reg &= ~(1u << pin);
}

int calm_io_read(uint32_t pin) {
    if (g_gpio_map == MAP_FAILED) return 0;
    volatile uint32_t *data_ro = (volatile uint32_t *)((uint8_t*)g_gpio_map + DATA_RO_OFFSET);
    return (*data_ro & (1u << pin)) ? 1 : 0;
}

void calm_hw_cleanup(void) {
    if (g_gpio_map != MAP_FAILED) munmap(g_gpio_map, GPIO_MAP_SIZE);
    if (g_slcr_map != MAP_FAILED) munmap(g_slcr_map, SLCR_MAP_SIZE);
    if (g_memfd >= 0) close(g_memfd);
}

/* 微秒级忙等延时 (适配 Zynq ~667MHz ARM Cortex-A9)
 * 实际延时会因编译器优化 / CPU 频率有所偏差,
 * 但用于 SPI 位脉冲的建立 / 保持时间已足够 */
static void delay_us(int us) {
    /* Zynq-7010 约 667 MHz, 每种循环迭代 ~3 cycles ≈ 4.5 ns/iter
     * 取保守值: 1 µs ≈ 100 次迭代 */
    volatile unsigned int count = us * 100;
    while (count--) {
        asm volatile("nop");
    }
}

/* ================================================================
 * SPI 位脉冲实现 (模式3: CPOL=1, CPHA=1)
 * 
 * 适配 MAX31865 数据手册要求的 SPI Mode 3:
 *   - SCK 空闲 = 高电平 (CPOL=1)
 *   - 数据在 SCK 上升沿采样, 在 SCK 下降沿改变 (CPHA=1)
 * 
 * 每 bit 时序流程:
 *   SCK=1 (空闲) → SCK=0 (下降沿, 主机设 MOSI) →
 *   延时 1µs → SCK=1 (上升沿, 从机输出 MISO, 主机采样) →
 *   延时 1µs → 读 MISO
 * ================================================================ */
uint8_t spi_transfer_byte(uint8_t tx_byte) {
    uint8_t rx_byte = 0;
    
    for (int i = 7; i >= 0; i--) {
        /* SCK 下降沿: 主机设定 MOSI 数据位 (MSB 先) */
        calm_io_write(MIO_SPI_SCK, 0);
        calm_io_write(MIO_SPI_MOSI, (tx_byte >> i) & 0x01);
        delay_us(1);
        
        /* SCK 上升沿: 从机采样 MOSI, 同时从机驱动 MISO */
        calm_io_write(MIO_SPI_SCK, 1);
        delay_us(1);
        
        /* 上升沿后主机采样 MISO */
        if (calm_io_read(MIO_SPI_MISO)) {
            rx_byte |= (1 << i);
        }
    }
    
    /* 通信结束, SCK 回到空闲高电平 */
    calm_io_write(MIO_SPI_SCK, 1);
    
    return rx_byte;
}

/* ================================================================
 * MAX31865 操作函数
 * ================================================================ */

/* 读取 MAX31865 寄存器
 * 帧格式: [地址字节(读: bit7=0)] [数据字节]
 * 地址字节的高位是写/读标志: 0x00-0x7F = 读, 0x80-0xFF = 写 */
uint8_t max31865_read_reg(uint32_t cs_pin, uint8_t reg_addr) {
    uint8_t result;
    
    /* 拉低片选, 开始通信 */
    calm_io_write(cs_pin, 0);
    
    /* 发送读命令 (bit7=0 表示读) */
    spi_transfer_byte(reg_addr & 0x7F);
    
    /* 接收返回值 */
    result = spi_transfer_byte(0x00);
    
    /* 拉高片选, 结束通信 */
    calm_io_write(cs_pin, 1);
    
    return result;
}

/* 写入 MAX31865 寄存器 */
void max31865_write_reg(uint32_t cs_pin, uint8_t reg_addr, uint8_t value) {
    /* 拉低片选 */
    calm_io_write(cs_pin, 0);
    
    /* 发送写命令 (bit7=1 表示写) */
    spi_transfer_byte(reg_addr | 0x80);
    
    /* 发送要写入的数据 */
    spi_transfer_byte(value);
    
    /* 拉高片选 */
    calm_io_write(cs_pin, 1);
}

/* 初始化 MAX31865:
 * - 启用 Vbias
 * - 自动转换模式
 * - 50Hz 陷波 (国内电网频率)
 * - 2 线制 PT100 连接
 * - 清除故障状态 */
void max31865_init(uint32_t cs_pin) {
    int ch = (cs_pin == MIO_SPI_CS1) ? 1 : 2;
    uint8_t config = MAX31865_CONFIG_VBIAS 
                   | MAX31865_CONFIG_AUTO 
                   | MAX31865_CONFIG_50HZ
                   | MAX31865_CONFIG_FAULT_CLR;
    /* 注意: 不设置 MAX31865_CONFIG_3WIRE 位 = 2/4 线制模式
     * 对于 2 线制 PT100, 需要将 FORCE+ 连到 RTDIN+,
     * FORCE- 连到 RTDIN-, 短路片接在对应端子上 */
    
    max31865_write_reg(cs_pin, MAX31865_REG_CONFIG, config);
    
    /* 延时等待 Vbias 稳定 + 首个转换完成 (~60ms for 50Hz) */
    delay_us(65000);
    
    /* 回读确认配置写入成功 */
    uint8_t readback = max31865_read_reg(cs_pin, MAX31865_REG_CONFIG);
    printf("[MAX31865] CS%d init: wrote=0x%02X, readback=0x%02X%s\n",
           ch, config, readback,
           (readback == config) ? " (OK)" : " (MISMATCH!)");
    
    /* 读取一次故障状态, 确认传感器在线 */
    uint8_t fault = max31865_read_reg(cs_pin, MAX31865_REG_FAULT_STAT);
    if (fault) {
        printf("[MAX31865] CS%d fault after init: 0x%02X (clearing...)\n", ch, fault);
        max31865_write_reg(cs_pin, MAX31865_REG_CONFIG, config | MAX31865_CONFIG_FAULT_CLR);
    }
}

/* 从 MAX31865 读取 RTD 电阻值
 * 返回: RTD 电阻值 (欧姆)
 * 
 * MAX31865 的 ADC 为 15 位, RTD 寄存器高字节的 bit7 是故障标志位
 * 计算公式: R_RTD = (ADC_code >> 1) * R_REF / 32768
 * 其中 R_REF = 430Ω (参考电阻) */
float max31865_read_rtd(uint32_t cs_pin) {
    /* 读取 RTD MSB 和 LSB */
    uint8_t msb = max31865_read_reg(cs_pin, MAX31865_REG_RTD_MSB);
    uint8_t lsb = max31865_read_reg(cs_pin, MAX31865_REG_RTD_LSB);
    
    /* 检查故障标志
     * MAX31865 数据手册: RTD MSB 寄存器的 bit7 是故障标志位,
     * RTD LSB 寄存器的 bit0 (D0) 也是故障指示位.
     * 当任意故障发生时, MSB bit7 会被置位 */
    if (msb & 0x80) {
        /* 故障检测: 读取故障状态寄存器查看详情 */
        uint8_t fault = max31865_read_reg(cs_pin, MAX31865_REG_FAULT_STAT);
        printf("[MAX31865] Fault detected on CS%d! Fault reg: 0x%02X\n",
               (cs_pin == MIO_SPI_CS1) ? 1 : 2, fault);
        /* 清除故障后继续尝试 */
        max31865_write_reg(cs_pin, MAX31865_REG_CONFIG, 
            max31865_read_reg(cs_pin, MAX31865_REG_CONFIG) | MAX31865_CONFIG_FAULT_CLR);
    }
    
    /* ADC 数据合并: 
     * RTD MSB[6:0] = ADC[14:8] (7 bits)
     * RTD LSB[7:1] = ADC[7:1]  (7 bits)
     * MAX31865 数据手册: 15-bit ADC 存储格式为
     *   MSB[6:0] << 8 | LSB[7:0], D0 始终为 0
     *   R_RTD = ADC_code * R_REF / 32768 */
    uint16_t adc_full = ((uint16_t)(msb & 0x7F) << 8) | lsb;
    adc_full >>= 1;  /* 去掉 D0 故障标志位, 得到纯 15-bit ADC 值 */
    
    /* 转换为电阻值: R_RTD = ADC_code * R_REF / 2^15 */
    float resistance = ((float)adc_full * PT100_REF_RESISTOR) / 32768.0f;
    
    /* DEBUG: 打印原始寄存器值 (每 ~60s 打印一次, 避免刷屏) */
    static int debug_print_counter = 0;
    int ch = (cs_pin == MIO_SPI_CS1) ? 1 : 2;
    if (debug_print_counter % 60 == 0) {
        printf("[DEBUG CS%d] raw MSB=0x%02X LSB=0x%02X | adc=%u | R=%.2fΩ\n",
               ch, msb, lsb, adc_full, resistance);
    }
    if (ch == 1) debug_print_counter++;  /* 只在 CS1 上计数 */
    
    return resistance;
}

/* Callendar-Van Dusen 公式: 将 PT100 电阻值转换为摄氏温度
 * 使用简化的线性近似 (在 0~100°C 范围内精度 ±0.5°C):
 *   R(t) = R0 * (1 + A*t + B*t^2)
 *   其中 R0 = 100Ω, A = 3.9083e-3, B = -5.775e-7
 * 
 * 对于 0°C 以上, 使用二次方程求解:
 *   t = (-R0*A + sqrt(R0^2*A^2 - 4*R0*B*(R0 - R))) / (2*R0*B)
 * 
 * 对于简化线性近似:
 *   t = (R - 100) / 0.385  (≈ 100 * 0.00385 = 0.385 Ω/°C)
 * 
 * 这里实现完整的 Callendar-Van Dusen 方程 */
float pt100_resistance_to_temp(float resistance) {
    const float R0 = 100.0f;
    const float A  = 3.9083e-3f;
    const float B  = -5.775e-7f;
    
    float R = resistance;
    
    if (R >= R0) {
        /* 0°C 以上: R = R0 * (1 + A*t + B*t^2)
         * 二次方程: R0*B*t^2 + R0*A*t + (R0 - R) = 0
         * t = (-R0*A + sqrt((R0*A)^2 - 4*R0*B*(R0 - R))) / (2*R0*B) */
        float discriminant = (R0 * A) * (R0 * A) - 4.0f * R0 * B * (R0 - R);
        if (discriminant < 0) discriminant = 0;
        float temp = (-R0 * A + sqrtf(discriminant)) / (2.0f * R0 * B);
        return temp;
    } else {
        /* 0°C 以下: R = R0 * (1 + A*t + B*t^2 + C*(t-100)*t^3)
         * 这里简化处理, 使用线性近似: t = (R - R0) / (R0 * A) */
        return (R - R0) / (R0 * A);
    }
}