// calm_ctrl.hpp
#ifndef CALM_CTRL_HPP
#define CALM_CTRL_HPP

#include <thread>
#include <atomic>
#include <mutex>

class CalmController {
public:
    CalmController();
    ~CalmController();

    void start();
    void stop();

    // 线程安全的设置接口
    void set_enable(bool enable);
    void set_target_temp(float temp);
    void set_manual_duty(float duty); // 覆盖 PID 手动设置占空比 (0.0~1.0)
    void set_pid(float p, float i, float d);

    // 状态数据打包结构体 (双通道温度)
    struct Status {
        float current_temp;   // 当前主控温度 (传感器1)
        float current_temp2;  // 第二路传感器温度
        float target_temp;
        float current_duty;
        bool enabled;
    };
    Status get_status();

private:
    void control_thread_func();
    float read_spi_temperature();     // SPI 读取 MAX31865 温度 (传感器1, CS1)
    float read_spi_temperature_ch2(); // SPI 读取 MAX31865 温度 (传感器2, CS2)
    float compute_pid(float current_temp);

    std::thread _worker;
    std::atomic<bool> _running;
    std::mutex _mtx;

    bool  _enabled;
    float _target_temp;
    float _current_temp;
    float _current_temp2;   // 第二路传感器温度
    float _current_duty;
    
    // PID 参数与状态
    float _kp, _ki, _kd;
    float _integral;
    float _last_error;
};

#endif
