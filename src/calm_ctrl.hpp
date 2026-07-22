// calm_ctrl.hpp
#ifndef CALM_CTRL_HPP
#define CALM_CTRL_HPP

#include <thread>
#include <atomic>
#include <mutex>

/* 单路 PID 状态 */
struct PidState {
    float kp = 1.0f;
    float ki = 0.1f;
    float kd = 0.05f;
    float integral = 0.0f;
    float last_error = 0.0f;
};

/* 单路通道状态 */
struct ChannelState {
    bool  enabled = false;
    float target_temp = 25.0f;
    float current_temp = 0.0f;
    float current_duty = 0.0f;   // 手动覆盖或 PID 算出的占空比
    bool  manual_mode = false;   // true = 手动占空比覆盖 PID
    PidState pid;
};

class CalmController {
public:
    CalmController();
    ~CalmController();

    void start();
    void stop();

    /* ---- 通道 1 (CS1, MIO08 PWM) ---- */
    void set_enable1(bool enable);
    void set_target_temp1(float temp);
    void set_manual_duty1(float duty);
    void set_pid1(float p, float i, float d);

    /* ---- 通道 2 (CS2, MIO50 PWM) ---- */
    void set_enable2(bool enable);
    void set_target_temp2(float temp);
    void set_manual_duty2(float duty);
    void set_pid2(float p, float i, float d);

    /* 状态打包 */
    struct Status {
        // Ch1
        float temp1, target1, duty1;
        bool  en1;
        // Ch2
        float temp2, target2, duty2;
        bool  en2;
    };
    Status get_status();

private:
    void control_thread_func();
    float read_spi_temperature_ch1();
    float read_spi_temperature_ch2();
    float compute_pid(float current_temp, ChannelState& ch, PidState& pid);

    std::thread _worker;
    std::atomic<bool> _running;
    std::mutex _mtx;

    ChannelState _ch1;
    ChannelState _ch2;
};

#endif