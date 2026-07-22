// calm_hw.c
#define _GNU_SOURCE
#include "calm_hw.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <time.h>

/* Zynq-7000 物理地址常量 */
#define SLCR_BASE       0xF8000000UL
#define SLCR_MAP_SIZE   0x1000
#define SLCR_UNLOCK_OFF 0x08
#define SLCR_LOCK_OFF   0x04
#define SLCR_UNLOCK_KEY 0x0000DF0D
#define SLCR_LOCK_KEY   0x0000767B
#define MIO_PIN_BASE    0x700
#define MIO_PIN(n)      (MIO_PIN_BASE + ((n) * 4))

/* GPIO 控制器地址 (UG585 §14.2.2) */
#define GPIO_BASE       0xE000A000UL
#define GPIO_MAP_SIZE   0x1000

/* MIO 电平配置 (假设 Bank0/1 均供电 3.3V) */
#define MIO_GPIO_OUT_33V 0x00001600U  /* GPIO输出, LVCMOS33, 上拉开, 三态关 */
#define MIO_GPIO_IN_33V  0x00001601U  /* GPIO输入, 三态开 */

/* ---- Bank-Aware GPIO 寄存器偏移 (每个 bank 间隔 0x40) ---- */
#define GPIO_DATA_RO_OFF(bank)  (0x060U + 0x004U * (bank))
#define GPIO_DIRM_OFF(bank)     (0x204U + 0x040U * (bank))
#define GPIO_OEN_OFF(bank)      (0x208U + 0x040U * (bank))
/* MASK_DATA 写寄存器: bank0 = 0x00/0x04, bank1 = 0x08/0x0C */
#define GPIO_MASK_DATA_OFF(bank, half)  (bank * 0x08U + half * 0x04U)

static int g_memfd = -1;
static void *g_slcr_map = MAP_FAILED;
static void *g_gpio_map = MAP_FAILED;

static void *map_phys_region(int fd, off_t phys_base, size_t map_size) {
    void *m = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_base);
    return (m == MAP_FAILED) ? MAP_FAILED : m;
}

/* 获取 GPIO 寄存器指针 */
static volatile uint32_t *gpio_reg(uint32_t offset) {
    return (volatile uint32_t *)((uint8_t *)g_gpio_map + offset);
}

/* MIO 引脚解码为 (bank, bit) */
static int mio_decode(uint32_t pin, uint32_t *bank, uint32_t *bit) {
    if (pin > 53U || bank == NULL || bit == NULL) return -1;
    *bank = pin / 32U;
    *bit  = pin % 32U;
    return 0;
}

int calm_hw_init(void) {
    if (g_memfd >= 0) return 0;

    g_memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_memfd < 0) {
        perror("[CALM] Failed to open /dev/mem");
        return -1;
    }

    g_slcr_map = map_phys_region(g_memfd, SLCR_BASE, SLCR_MAP_SIZE);
    if (g_slcr_map == MAP_FAILED) {
        perror("[CALM] Failed to mmap SLCR");
        close(g_memfd); g_memfd = -1;
        return -1;
    }

    g_gpio_map = map_phys_region(g_memfd, GPIO_BASE, GPIO_MAP_SIZE);
    if (g_gpio_map == MAP_FAILED) {
        perror("[CALM] Failed to mmap GPIO");
        munmap(g_slcr_map, SLCR_MAP_SIZE);
        close(g_memfd);
        g_slcr_map = MAP_FAILED;
        g_memfd = -1;
        return -1;
    }

    printf("[CALM] Hardware memory mapped (SLCR + GPIO).\n");

    /* 解锁 SLCR 并配置 MIO 复用 */
    volatile uint32_t *slcr_unlock = (volatile uint32_t *)((uint8_t*)g_slcr_map + SLCR_UNLOCK_OFF);
    volatile uint32_t *slcr_lock   = (volatile uint32_t *)((uint8_t*)g_slcr_map + SLCR_LOCK_OFF);

    *slcr_unlock = SLCR_UNLOCK_KEY;

    /* MIO 复用寄存器直接写绝对偏移 (MIO8~13 在 Bank0, MIO50 在 Bank1, 无需 bank 变换) */
    volatile uint32_t *mio_regs[7];
    uint32_t mio_pins[] = {MIO_HEATER_PWM, MIO_SPI_CS2, MIO_SPI_MOSI,
                           MIO_SPI_MISO, MIO_SPI_SCK, MIO_SPI_CS1,
                           MIO_HEATER_PWM2};
    uint32_t mio_cfgs[]  = {MIO_GPIO_OUT_33V, MIO_GPIO_OUT_33V, MIO_GPIO_OUT_33V,
                            MIO_GPIO_IN_33V,  MIO_GPIO_OUT_33V, MIO_GPIO_OUT_33V,
                            MIO_GPIO_OUT_33V};

    for (int i = 0; i < 7; ++i) {
        mio_regs[i] = (volatile uint32_t *)((uint8_t*)g_slcr_map + MIO_PIN(mio_pins[i]));
        *mio_regs[i] = mio_cfgs[i];
    }

    *slcr_lock = SLCR_LOCK_KEY;

    /* 初始化 SPI 管脚方向 */
    calm_io_set_dir(MIO_SPI_MOSI, 1);
    calm_io_set_dir(MIO_SPI_SCK,  1);
    calm_io_set_dir(MIO_SPI_MISO, 0);
    calm_io_set_dir(MIO_SPI_CS1,  1);
    calm_io_set_dir(MIO_SPI_CS2,  1);

    /* SPI 空闲状态 (Mode 3: CPOL=1, CPHA=1) */
    calm_io_write(MIO_SPI_SCK,  1);
    calm_io_write(MIO_SPI_MOSI, 0);
    calm_io_write(MIO_SPI_CS1,  1);
    calm_io_write(MIO_SPI_CS2,  1);

    /* 初始化两路加热器 PWM 为输出并拉低 */
    calm_io_set_dir(MIO_HEATER_PWM, 1);
    calm_io_write(MIO_HEATER_PWM, 0);
    calm_io_set_dir(MIO_HEATER_PWM2, 1);
    calm_io_write(MIO_HEATER_PWM2, 0);

    /* 初始化两路 MAX31865 */
    max31865_init(MIO_SPI_CS1);
    max31865_init(MIO_SPI_CS2);

    return 0;
}

/* ---- Bank-Aware GPIO 操作 ---- */

void calm_io_set_dir(uint32_t pin, int is_output) {
    uint32_t bank, bit;
    if (g_gpio_map == MAP_FAILED || mio_decode(pin, &bank, &bit) != 0) return;

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

void calm_io_write(uint32_t pin, int value) {
    uint32_t bank, bit;
    if (g_gpio_map == MAP_FAILED || mio_decode(pin, &bank, &bit) != 0) return;

    /* MASK_DATA 写: 高16位=mask (0=更新), 低16位=data */
    const uint32_t half   = bit / 16U;
    const uint32_t pos    = bit % 16U;
    const uint32_t offset = GPIO_MASK_DATA_OFF(bank, half);

    const uint32_t update_mask = 0xFFFFU & ~(1U << pos);
    const uint32_t data        = value ? (1U << pos) : 0U;

    *gpio_reg(offset) = (update_mask << 16U) | data;
}

int calm_io_read(uint32_t pin) {
    uint32_t bank, bit;
    if (g_gpio_map == MAP_FAILED || mio_decode(pin, &bank, &bit) != 0) return 0;

    return ((*gpio_reg(GPIO_DATA_RO_OFF(bank))) & (1U << bit)) != 0U;
}

void calm_hw_cleanup(void) {
    if (g_gpio_map != MAP_FAILED) { munmap(g_gpio_map, GPIO_MAP_SIZE); g_gpio_map = MAP_FAILED; }
    if (g_slcr_map != MAP_FAILED) { munmap(g_slcr_map, SLCR_MAP_SIZE); g_slcr_map = MAP_FAILED; }
    if (g_memfd >= 0) { close(g_memfd); g_memfd = -1; }
}

/* ---- SPI 位脉冲 (Mode 3: CPOL=1, CPHA=1) ---- */

static void delay_us(int us) {
    volatile unsigned int count = us * 100;
    while (count--) { asm volatile("nop"); }
}

uint8_t spi_transfer_byte(uint8_t tx_byte) {
    uint8_t rx_byte = 0;

    for (int i = 7; i >= 0; i--) {
        calm_io_write(MIO_SPI_SCK, 0);
        calm_io_write(MIO_SPI_MOSI, (tx_byte >> i) & 0x01);
        delay_us(1);

        calm_io_write(MIO_SPI_SCK, 1);
        delay_us(1);

        if (calm_io_read(MIO_SPI_MISO))
            rx_byte |= (1 << i);
    }

    calm_io_write(MIO_SPI_SCK, 1);
    return rx_byte;
}

/* ---- MAX31865 操作 ---- */

uint8_t max31865_read_reg(uint32_t cs_pin, uint8_t reg_addr) {
    uint8_t result;
    calm_io_write(cs_pin, 0);
    spi_transfer_byte(reg_addr & 0x7F);
    result = spi_transfer_byte(0x00);
    calm_io_write(cs_pin, 1);
    return result;
}

void max31865_write_reg(uint32_t cs_pin, uint8_t reg_addr, uint8_t value) {
    calm_io_write(cs_pin, 0);
    spi_transfer_byte(reg_addr | 0x80);
    spi_transfer_byte(value);
    calm_io_write(cs_pin, 1);
}

void max31865_init(uint32_t cs_pin) {
    int ch = (cs_pin == MIO_SPI_CS1) ? 1 : 2;
    uint8_t config = MAX31865_CONFIG_VBIAS 
                   | MAX31865_CONFIG_AUTO 
                   | MAX31865_CONFIG_50HZ
                   | MAX31865_CONFIG_FAULT_CLR;

    max31865_write_reg(cs_pin, MAX31865_REG_CONFIG, config);

    /* Vbias 稳定 + 首个50Hz转换 ≈ 62.5ms, 用 usleep 更可靠 */
    usleep(70000);

    /* 回读: FAULT_CLR 是自清零位, 比较时屏蔽之 */
    uint8_t readback  = max31865_read_reg(cs_pin, MAX31865_REG_CONFIG);
    uint8_t expected  = config & ~MAX31865_CONFIG_FAULT_CLR;
    uint8_t readback_cmp = readback & ~MAX31865_CONFIG_FAULT_CLR;

    printf("[MAX31865] CS%d init: wrote=0x%02X, readback=0x%02X%s\n",
           ch, config, readback,
           (readback_cmp == expected) ? " (OK)" : " (MISMATCH!)");

    /* 检查初始故障 */
    uint8_t fault = max31865_read_reg(cs_pin, MAX31865_REG_FAULT_STAT);
    if (fault) {
        printf("[MAX31865] CS%d fault after init: 0x%02X (clearing...)\n", ch, fault);
        max31865_write_reg(cs_pin, MAX31865_REG_CONFIG,
            (readback & ~MAX31865_CONFIG_FAULT_CLR) | MAX31865_CONFIG_FAULT_CLR);
    }
}

/* 改良版 RTD 读取: 单次 CS 事务 + 正确故障位检测 + 故障闭锁语义
 * 返回: 0 = 成功, -1 = 传感器故障 (调用方应禁止加热) */
int max31865_read_rtd(uint32_t cs_pin, float *resistance) {
    if (resistance == NULL) return -1;

    /* 一次 CS 事务连续读取 MSB + LSB, 避免跨周期不一致 */
    calm_io_write(cs_pin, 0);
    spi_transfer_byte(MAX31865_REG_RTD_MSB & 0x7F);
    uint8_t msb = spi_transfer_byte(0x00);
    uint8_t lsb = spi_transfer_byte(0x00);
    calm_io_write(cs_pin, 1);

    /* 故障检测: 正确位置在 LSB bit0 (D0) — 参见 MAX31865 数据手册 */
    if (lsb & 0x01U) {
        uint8_t fault = max31865_read_reg(cs_pin, MAX31865_REG_FAULT_STAT);
        int ch = (cs_pin == MIO_SPI_CS1) ? 1 : 2;
        fprintf(stderr, "[MAX31865] CS%d sensor FAULT! Fault reg: 0x%02X\n", ch, fault);
        /* 尝试清除故障 */
        uint8_t cfg = max31865_read_reg(cs_pin, MAX31865_REG_CONFIG);
        max31865_write_reg(cs_pin, MAX31865_REG_CONFIG, cfg | MAX31865_CONFIG_FAULT_CLR);
        return -1;
    }

    /* ADC 提取: MSB D7=ADC[14], MSB D6-0=ADC[13:7], LSB D7-1=ADC[6:0], LSB D0=Fault
     * 15-bit ADC = ((MSB << 8) | LSB) >> 1 */
    uint16_t adc = (((uint16_t)msb << 8) | lsb) >> 1;

    *resistance = ((float)adc * PT100_REF_RESISTOR) / 32768.0f;

    /* 调试输出 (每60s一次) */
    static int dbg_cnt = 1;
    int ch = (cs_pin == MIO_SPI_CS1) ? 1 : 2;
    if (dbg_cnt % 60 == 0) {
        printf("[DEBUG CS%d] raw MSB=0x%02X LSB=0x%02X | adc=%u | R=%.2fΩ\n",
               ch, msb, lsb, adc, *resistance);
    }
    if (ch == 1) dbg_cnt++;

    return 0;
}

/* PT100 电阻值转摄氏温度 (Callendar-Van Dusen 方程) */
float pt100_resistance_to_temp(float resistance) {
    const float R0 = 100.0f;
    const float A  = 3.9083e-3f;
    const float B  = -5.775e-7f;

    float R = resistance;

    if (R >= R0) {
        /* 0°C 以上: R = R0 * (1 + A*t + B*t^2) */
        float discriminant = (R0 * A) * (R0 * A) - 4.0f * R0 * B * (R0 - R);
        if (discriminant < 0.0f) discriminant = 0.0f;
        return (-R0 * A + sqrtf(discriminant)) / (2.0f * R0 * B);
    } else {
        /* 0°C 以下: 简化线性近似 */
        return (R - R0) / (R0 * A);
    }
}
