// calm_ctrl.cpp
#include "calm_ctrl.hpp"
#include "calm_hw.h"
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <unistd.h>

CalmController::CalmController() : _running(false) {
    if (calm_hw_init() != 0) {
        throw std::runtime_error("CALM: hardware initialization failed");
    }
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
    if (!enable) {
        _ch1.current_duty = 0.0f;
        _ch1.pid.integral = 0.0f;
        _ch1.pid.last_error = 0.0f;
    }
}
void CalmController::set_target_temp1(float temp) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch1.target_temp = temp;
    _ch1.manual_mode = false;
    /* 切换目标温度时重置 PID 状态, 避免旧积分残留 */
    _ch1.pid.integral = 0.0f;
    _ch1.pid.last_error = 0.0f;
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
    if (!enable) {
        _ch2.current_duty = 0.0f;
        _ch2.pid.integral = 0.0f;
        _ch2.pid.last_error = 0.0f;
    }
}
void CalmController::set_target_temp2(float temp) {
    std::lock_guard<std::mutex> lock(_mtx);
    _ch2.target_temp = temp;
    _ch2.manual_mode = false;
    _ch2.pid.integral = 0.0f;
    _ch2.pid.last_error = 0.0f;
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

/* ============ 温度读取 (新 API: 故障时返回 NaN) ============ */
static float read_temp_from_ch(uint32_t cs_pin) {
    float resistance = 0.0f;
    if (max31865_read_rtd(cs_pin, &resistance) != 0) {
        /* 传感器故障 → 返回 NaN, 调用方应检测并锁停该通道 */
        return NAN;
    }
    return pt100_resistance_to_temp(resistance);
}

/* ============ 通用 PID 计算 (带实际 dt) ============ */
static float compute_pid_dt(float current_temp, ChannelState& ch, PidState& pid, float dt) {
    float err = ch.target_temp - current_temp;
    pid.integral += err * dt;
    pid.integral = std::max(-10.0f, std::min(pid.integral, 10.0f));

    float deriv = (dt > 0.001f) ? ((err - pid.last_error) / dt) : 0.0f;
    pid.last_error = err;

    float output = (pid.kp * err) + (pid.ki * pid.integral) + (pid.kd * deriv);
    return std::max(0.0f, std::min(output, 1.0f));
}

/* ============ 主控制线程 ============ */
void CalmController::control_thread_func() {
    using namespace std::chrono;
    int tick = 0;
    auto last_pid_time = steady_clock::now();

    while (_running) {
        auto next_wakeup = steady_clock::now() + milliseconds(10);

        if (tick == 0) {
            auto now = steady_clock::now();
            float dt = duration<float>(now - last_pid_time).count();
            if (dt < 0.1f) dt = 1.0f;  /* 兜底: 首次或频率异常时用 1s */
            last_pid_time = now;

            float temp1 = read_temp_from_ch(MIO_SPI_CS1);
            float temp2 = read_temp_from_ch(MIO_SPI_CS2);

            std::lock_guard<std::mutex> lock(_mtx);

            /* 通道 1: 传感器故障锁停 */
            if (std::isnan(temp1)) {
                _ch1.enabled = false;
                _ch1.current_duty = 0.0f;
                _ch1.current_temp = NAN;
            } else {
                _ch1.current_temp = temp1;
                if (_ch1.enabled && !_ch1.manual_mode) {
                    _ch1.current_duty = compute_pid_dt(temp1, _ch1, _ch1.pid, dt);
                }
            }

            /* 通道 2: 传感器故障锁停 */
            if (std::isnan(temp2)) {
                _ch2.enabled = false;
                _ch2.current_duty = 0.0f;
                _ch2.current_temp = NAN;
            } else {
                _ch2.current_temp = temp2;
                if (_ch2.enabled && !_ch2.manual_mode) {
                    _ch2.current_duty = compute_pid_dt(temp2, _ch2, _ch2.pid, dt);
                }
            }

            printf("[CALM] Ch1: %.3f C (T:%.1f D:%.1f%%) | Ch2: %.3f C (T:%.1f D:%.1f%%)\n",
                   _ch1.current_temp, _ch1.target_temp, _ch1.current_duty * 100.0f,
                   _ch2.current_temp, _ch2.target_temp, _ch2.current_duty * 100.0f);
        }

        /* 提取快照 */
        bool en1, en2;
        float d1, d2;
        {
            std::lock_guard<std::mutex> lock(_mtx);
            en1 = _ch1.enabled && !std::isnan(_ch1.current_temp);
            d1  = _ch1.current_duty;
            en2 = _ch2.enabled && !std::isnan(_ch2.current_temp);
            d2  = _ch2.current_duty;
        }

        /* PWM1 (MIO08) */
        int act1 = static_cast<int>(d1 * 100);
        calm_io_write(MIO_HEATER_PWM, (en1 && tick < act1) ? 1 : 0);

        /* PWM2 (MIO50) */
        int act2 = static_cast<int>(d2 * 100);
        calm_io_write(MIO_HEATER_PWM2, (en2 && tick < act2) ? 1 : 0);

        tick = (tick + 1) % 100;
        std::this_thread::sleep_until(next_wakeup);
    }
}