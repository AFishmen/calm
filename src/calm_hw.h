// calm_hw.h
#ifndef CALM_HW_H
#define CALM_HW_H

#include <stdint.h>

/* CALM MIO 管脚分配 — 适配赛灵思 Zynq-7010 */
/* SPI1 位脉冲接口 (两路 MAX31865, PT100 温度传感器) */
#define MIO_SPI_MOSI  10   // PS_MIO10 → SPI1_MOSI
#define MIO_SPI_MISO  11   // PS_MIO11 → SPI1_MISO
#define MIO_SPI_SCK   12   // PS_MIO12 → SPI1_SCK
#define MIO_SPI_CS1   13   // PS_MIO13 → MAX31865 #1 片选
#define MIO_SPI_CS2   9    // PS_MIO09 → MAX31865 #2 片选

/* 加热器 PWM 输出 */
#define MIO_HEATER_PWM  8   // PS_MIO08  → PID1 加热器 PWM
#define MIO_HEATER_PWM2 50  // PS_MIO50  → PID2 加热器 PWM

/* MAX31865 SPI 配置常量 */
#define MAX31865_REG_CONFIG   0x00
#define MAX31865_REG_RTD_MSB  0x01
#define MAX31865_REG_RTD_LSB  0x02
#define MAX31865_REG_HFAULT_MSB 0x03
#define MAX31865_REG_HFAULT_LSB 0x04
#define MAX31865_REG_LFAULT_MSB 0x05
#define MAX31865_REG_LFAULT_LSB 0x06
#define MAX31865_REG_FAULT_STAT 0x07

/* MAX31865 配置寄存器位 */
#define MAX31865_CONFIG_VBIAS    (1 << 7)  // Vbias 使能
#define MAX31865_CONFIG_AUTO     (1 << 6)  // 自动转换模式
#define MAX31865_CONFIG_1SHOT    (1 << 5)  // 单次触发
#define MAX31865_CONFIG_3WIRE    (1 << 4)  // 3 线制
#define MAX31865_CONFIG_FAULT_CLR (1 << 1) // 清除故障
#define MAX31865_CONFIG_50HZ     (1 << 0)  // 50Hz 陷波

/* PT100 参考电阻值 (欧姆) */
#define PT100_REF_RESISTOR 430.0f

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 /dev/mem 映射并配置相应的 MIO 管脚为 GPIO */
int calm_hw_init(void);

/* 释放内存映射 */
void calm_hw_cleanup(void);

/* 控制 GPIO 方向: is_output = 1 (输出), 0 (输入) */
void calm_io_set_dir(uint32_t pin, int is_output);

/* 设置 GPIO 输出电平 */
void calm_io_write(uint32_t pin, int value);

/* 读取 GPIO 当前电平 */
int calm_io_read(uint32_t pin);

/* ---- SPI 位脉冲接口 (用于 MAX31865) ---- */

/* 发送一个 SPI 字节并同时接收一个字节 (模式1: CPOL=0, CPHA=1) */
uint8_t spi_transfer_byte(uint8_t tx_byte);

/* 读取 MAX31865 的单个寄存器 */
uint8_t max31865_read_reg(uint32_t cs_pin, uint8_t reg_addr);

/* 写入 MAX31865 的单个寄存器 */
void max31865_write_reg(uint32_t cs_pin, uint8_t reg_addr, uint8_t value);

/* 初始化 MAX31865 传感器 (设置配置寄存器) */
void max31865_init(uint32_t cs_pin);

/* 从 MAX31865 读取 RTD 电阻值 (欧姆)
 * 返回 0 = 成功, -1 = 传感器故障 */
int max31865_read_rtd(uint32_t cs_pin, float *resistance);

/* 将 PT100 电阻值转换为摄氏温度 (Callendar-Van Dusen 简化公式) */
float pt100_resistance_to_temp(float resistance);

#ifdef __cplusplus
}
#endif

#endif // CALM_HW_H
