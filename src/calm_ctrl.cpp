// calm_ctrl.cpp
#include "calm_ctrl.hpp"
#include "calm_hw.h"
#include <chrono>
#include <algorithm>

CalmController::CalmController() 
    : _running(false), _enabled(false), _target_temp(25.0f), 
      _current_temp(0.0f), _current_temp2(0.0f), _current_duty(0.0f),
      _kp(1.0f), _ki(0.1f), _kd(0.05f), _integral(0.0f), _last_error(0.0f) {
    calm_hw_init();
}

CalmController::~CalmController() {
    stop();
    calm_hw_cleanup();
}

void CalmController::start() {
    if (!_running) {
        _running = true;
        _worker = std::thread(&CalmController::control_thread_func, this);
    }
}

void CalmController::stop() {
    _running = false;
    if (_worker.joinable()) _worker.join();
    calm_io_write(MIO_HEATER_PWM, 0); // 断电保护
}

void CalmController::set_enable(bool enable) {
    std::lock_guard<std::mutex> lock(_mtx);
    _enabled = enable;
}

void CalmController::set_target_temp(float temp) {
    std::lock_guard<std::mutex> lock(_mtx);
    _target_temp = temp;
}

void CalmController::set_manual_duty(float duty) {
    std::lock_guard<std::mutex> lock(_mtx);
    _current_duty = std::max(0.0f, std::min(duty, 1.0f));
}

void CalmController::set_pid(float p, float i, float d) {
    std::lock_guard<std::mutex> lock(_mtx);
    _kp = p; _ki = i; _kd = d;
}

CalmController::Status CalmController::get_status() {
    std::lock_guard<std::mutex> lock(_mtx);
    return {_current_temp, _current_temp2, _target_temp, _current_duty, _enabled};
}

/* 读取 MAX31865 #1 温度 (CS1, MIO13, 传感器1)
 * 用于 PID 主控回路 */
float CalmController::read_spi_temperature() {
    float resistance = max31865_read_rtd(MIO_SPI_CS1);
    return pt100_resistance_to_temp(resistance);
}

/* 读取 MAX31865 #2 温度 (CS2, MIO09, 传感器2)
 * 用于辅助监测 */
float CalmController::read_spi_temperature_ch2() {
    float resistance = max31865_read_rtd(MIO_SPI_CS2);
    return pt100_resistance_to_temp(resistance);
}

float CalmController::compute_pid(float current_temp) {
    float err = _target_temp - current_temp;
    _integral += err;
    _integral = std::max(-10.0f, std::min(_integral, 10.0f)); // Anti-windup
    
    float deriv = err - _last_error;
    _last_error = err;

    float output = (_kp * err) + (_ki * _integral) + (_kd * deriv);
    return std::max(0.0f, std::min(output, 1.0f));
}

void CalmController::control_thread_func() {
    using namespace std::chrono;
    int tick = 0; // 0 ~ 99 (10ms tick, 总周期 1s)

    while (_running) {
        auto next_wakeup = steady_clock::now() + milliseconds(10);

        // 每 1 秒的第 0 个 tick 触发一次数据读取和 PID 运算
        if (tick == 0) {
            // 读取两路 MAX31865 传感器温度
            float temp1 = read_spi_temperature();
            float temp2 = read_spi_temperature_ch2();
            
            std::lock_guard<std::mutex> lock(_mtx);
            _current_temp = temp1;
            _current_temp2 = temp2;
            if (_enabled) {
                _current_duty = compute_pid(_current_temp);
            }
            printf("[CALM] Ch1: %.3f C | Ch2: %.3f C | Target: %.1f C | Duty: %.1f%% | Enabled: %d\n", 
                   _current_temp, _current_temp2, _target_temp, _current_duty * 100.0f, _enabled);
        }

        // 提取当前需要执行的 PWM 状态
        bool en; float duty;
        {
            std::lock_guard<std::mutex> lock(_mtx);
            en = _enabled;
            duty = _current_duty;
        }

        // 根据占空比决定当前 10ms 窗口的电平
        int active_ticks = static_cast<int>(duty * 100);
        if (en && tick < active_ticks) {
            calm_io_write(MIO_HEATER_PWM, 1);
        } else {
            calm_io_write(MIO_HEATER_PWM, 0);
        }

        tick = (tick + 1) % 100;
        std::this_thread::sleep_until(next_wakeup);
    }
}