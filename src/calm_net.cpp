// calm_net.cpp
#include "calm_net.hpp"
#include <iostream>
#include <sstream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

CalmServer::CalmServer(CalmController& ctrl, int port) 
    : _ctrl(ctrl), _port(port) {}

void CalmServer::run() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("[CALM] socket() failed");
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(_port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("[CALM] bind() failed");
        close(server_fd);
        return;
    }

    if (listen(server_fd, 3) < 0) {
        perror("[CALM] listen() failed");
        close(server_fd);
        return;
    }

    std::cout << "CALM Server listening on port " << _port << "..." << std::endl;

    while (true) {
        int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd >= 0) {
            handle_client(client_fd);
            close(client_fd);
        }
    }
}

void CalmServer::handle_client(int client_fd) {
    char buffer[256];
    printf("[Network] New host connected.\n");
    while (true) {
        ssize_t bytes = read(client_fd, buffer, sizeof(buffer) - 1);
        if (bytes <= 0){
            printf("[Network] Host disconnected.\n");
            break;
        }
        
        buffer[bytes] = '\0';
        std::string req(buffer);
        
        size_t pos = req.find('\n');
        if (pos != std::string::npos) req = req.substr(0, pos);
        printf("[Network] Received cmd: %s\n", req.c_str());
        std::string reply = process_command(req) + "\n";
        ssize_t written = write(client_fd, reply.c_str(), reply.length());
        (void)written;  /* 忽略 write 部分失败的边际情况 */
    }
}

std::string CalmServer::process_command(const std::string& cmd) {
    std::istringstream iss(cmd);
    std::string action;
    iss >> action;

    if (action == "STATUS") {
        auto st = _ctrl.get_status();
        char buf[384];
        snprintf(buf, sizeof(buf),
            "{\"ch1\":{\"temp\":%.3f,\"target\":%.1f,\"duty\":%.3f,\"enabled\":%d},"
            "\"ch2\":{\"temp\":%.3f,\"target\":%.1f,\"duty\":%.3f,\"enabled\":%d}}",
            st.temp1, st.target1, st.duty1, st.en1 ? 1 : 0,
            st.temp2, st.target2, st.duty2, st.en2 ? 1 : 0);
        return std::string(buf);
    }

    /* 以下命令需要通道参数: <CMD> <ch> <args...> */
    int ch;
    if (!(iss >> ch)) {
        return "ERROR: Missing channel. Format: <CMD> <1|2> <args...>";
    }
    if (ch < 1 || ch > 2) {
        return "ERROR: Channel must be 1 or 2";
    }

    if (action == "SET_EN") {
        int en;
        if (!(iss >> en)) return "ERROR: Missing enable value (0 or 1)";
        if (en != 0 && en != 1) return "ERROR: Enable must be 0 or 1";
        if (ch == 1) _ctrl.set_enable1(en > 0);
        else         _ctrl.set_enable2(en > 0);
        return "OK";
    }
    else if (action == "SET_TARGET") {
        float t; 
        if (!(iss >> t)) return "ERROR: Missing temperature. Format: SET_TARGET <1|2> <temp>";
        if (!std::isfinite(t) || t < -50.0f || t > 200.0f)
            return "ERROR: Temperature out of safe range (-50 .. 200)";
        if (ch == 1) _ctrl.set_target_temp1(t);
        else         _ctrl.set_target_temp2(t);
        return "OK";
    }
    else if (action == "SET_DUTY") {
        float d; 
        if (!(iss >> d)) return "ERROR: Missing duty. Format: SET_DUTY <1|2> <duty>";
        if (!std::isfinite(d) || d < 0.0f || d > 1.0f)
            return "ERROR: Duty must be in range 0.0 .. 1.0";
        if (ch == 1) _ctrl.set_manual_duty1(d);
        else         _ctrl.set_manual_duty2(d);
        return "OK";
    }
    else if (action == "SET_PID") {
        float p, i, d;
        if (!(iss >> p >> i >> d))
            return "ERROR: Missing parameters. Format: SET_PID <1|2> <P> <I> <D>";
        if (!std::isfinite(p) || !std::isfinite(i) || !std::isfinite(d))
            return "ERROR: PID gains must be finite numbers";
        if (ch == 1) _ctrl.set_pid1(p, i, d);
        else         _ctrl.set_pid2(p, i, d);
        return "OK";
    }
    
    return "ERROR: Unknown Command";
}