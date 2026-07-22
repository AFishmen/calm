// main.cpp
#include "calm_ctrl.hpp"
#include "calm_net.hpp"

int main() {
    CalmController calm_sys;
    calm_sys.start();

    // 在主线程启动网络服务器 (阻塞)
    CalmServer server(calm_sys, 8080);
    server.run();

    return 0;
}