// calm_ctrl.cpp
#include "calm_ctrl.hpp"
#include "calm_hw.h"
#include <chrono>
#include <algorithm>

CalmController::CalmController() : _running(false) {
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
    calm_io_write(MIO_HEATER_PWM, 0);
    calm_io_write(MIO_HEATER_PWM2, 0);
}

/* ============ 通道 1 (CS1 → MIO08 PWM) ============ */
void CalmController::set_enable1(bool enable) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch1.enabled = enable;
}
void CalmController::set_target_temp1(float temp) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch1.target_temp = temp;
    _ch1.manual_mode = false;
}
void CalmController::set_manual_duty1(float duty) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch1.current_duty = std::max(0.0f, std::min(duty, 1.0f));
    _ch1.manual_mode = true;
}
void CalmController::set_pid1(float p, float i, float d) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch1.pid.kp = p; _ch1.pid.ki = i; _ch1.pid.kd = d;
}

/* ============ 通道 2 (CS2 → MIO50 PWM) ============ */
void CalmController::set_enable2(bool enable) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch2.enabled = enable;
}
void CalmController::set_target_temp2(float temp) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch2.target_temp = temp;
    _ch2.manual_mode = false;
}
void CalmController::set_manual_duty2(float duty) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch2.current_duty = std::max(0.0f, std::min(duty, 1.0f));
    _ch2.manual_mode = true;
}
void CalmController::set_pid2(float p, float i, float d) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch2.pid.kp = p; _ch2.pid.ki = i; _ch2.pid.kd = d;
}

/* ============ Status 获取 ============ */
CalmController::Status CalmController::get_status() {
    std::lock_guard<std::mutex> lock(_mtx);
    return {
        _ch1.current_temp, _ch1.target_temp, _ch1.current_duty, _ch1.enabled,
        _ch2.current_temp, _ch2.target_temp, _ch2.current_duty, _ch2.enabled
    };
}

/* ============ 温度读取 ============ */
float CalmController::read_spi_temperature_ch1() {
    float resistance = max31865_read_rtd(MIO_SPI_CS1);
    return pt100_resistance_to_temp(resistance);
}
float CalmController::read_spi_temperature_ch2() {
    float resistance = max31865_read_rtd(MIO_SPI_CS2);
    return pt100_resistance_to_temp(resistance);
}

/* ============ 通用 PID 计算 ============ */
float CalmController::compute_pid(float current_temp, ChannelState& ch, PidState& pid) {
    float err = ch.target_temp - current_temp;
    pid.integral += err;
    pid.integral = std::max(-10.0f, std::min(pid.integral, 10.0f)); // Anti-windup

    float deriv = err - pid.last_error;
    pid.last_error = err;

    float output = (pid.kp * err) + (pid.ki * pid.integral) + (pid.kd * deriv);
    return std::max(0.0f, std::min(output, 1.0f));
}

/* ============ 主控制线程 ============ */
void CalmController::control_thread_func() {
    using namespace std::chrono;
    int tick = 0;

    while (_running) {
        auto next_wakeup = steady_clock::now() + milliseconds(10);

        if (tick == 0) {
            float temp1 = read_spi_temperature_ch1();
            float temp2 = read_spi_temperature_ch2();

            std::lock_guard<std::mutex> lock(_mtx);
            _ch1.current_temp = temp1;
            _ch2.current_temp = temp2;

            // 通道 1 PID
            if (_ch1.enabled && !_ch1.manual_mode) {
                _ch1.current_duty = compute_pid(temp1, _ch1, _ch1.pid);
            }
            // 通道 2 PID
            if (_ch2.enabled && !_ch2.manual_mode) {
                _ch2.current_duty = compute_pid(temp2, _ch2, _ch2.pid);
            }

            printf("[CALM] Ch1: %.3f C (T:%.1f D:%.1f%%) | Ch2: %.3f C (T:%.1f D:%.1f%%)\n",
                   _ch1.current_temp, _ch1.target_temp, _ch1.current_duty * 100.0f,
                   _ch2.current_temp, _ch2.target_temp, _ch2.current_duty * 100.0f);
        }

        // 提取快照
        bool en1, en2;
        float d1, d2;
        {
            std::lock_guard<std::mutex> lock(_mtx);
            en1 = _ch1.enabled;
            d1  = _ch1.current_duty;
            en2 = _ch2.enabled;
            d2  = _ch2.current_duty;
        }

        // PWM1 (MIO08)
        int act1 = static_cast<int>(d1 * 100);
        calm_io_write(MIO_HEATER_PWM, (en1 && tick < act1) ? 1 : 0);

        // PWM2 (MIO50)
        int act2 = static_cast<int>(d2 * 100);
        calm_io_write(MIO_HEATER_PWM2, (en2 && tick < act2) ? 1 : 0);

        tick = (tick + 1) % 100;
        std::this_thread::sleep_until(next_wakeup);
    }
}