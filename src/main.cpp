// main.cpp
#include "calm_ctrl.hpp"
#include "calm_net.hpp"
#include <cstdio>
#include <stdexcept>

int main() {
    try {
        CalmController calm_sys;
        calm_sys.start();

        /* 在主线程启动网络服务器 (阻塞) */
        CalmServer server(calm_sys, 8080);
        server.run();
    }
    catch (const std::exception& e) {
        fprintf(stderr, "[CALM] Fatal: %s\n", e.what());
        return 1;
    }

    return 0;
}