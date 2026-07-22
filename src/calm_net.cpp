// calm_net.cpp
#include "calm_net.hpp"
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

CalmServer::CalmServer(CalmController& ctrl, int port) 
    : _ctrl(ctrl), _port(port) {}

void CalmServer::run() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(_port);

    bind(server_fd, (struct sockaddr*)&address, sizeof(address));
    listen(server_fd, 3);
    
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
        
        // 简单处理包含换行符的命令
        size_t pos = req.find('\n');
        if (pos != std::string::npos) req = req.substr(0, pos);
        printf("[Network] Received cmd: %s\n", req.c_str());
        std::string reply = process_command(req) + "\n";
        write(client_fd, reply.c_str(), reply.length());
    }
}
std::string CalmServer::process_command(const std::string& cmd) {
    std::istringstream iss(cmd);
    std::string action;
    iss >> action;

    if (action == "STATUS") {
        auto st = _ctrl.get_status();
        char buf[256];
        snprintf(buf, sizeof(buf), "{\"temp\":%.3f, \"temp2\":%.3f, \"target\":%.3f, \"duty\":%.2f, \"enabled\":%d}",
                 st.current_temp, st.current_temp2, st.target_temp, st.current_duty, st.enabled ? 1 : 0);
        return std::string(buf);
    } 
    else if (action == "SET_EN") {
        int en; iss >> en;
        _ctrl.set_enable(en > 0);
        return "OK";
    }
    else if (action == "SET_TARGET") {
        float t; iss >> t;
        _ctrl.set_target_temp(t);
        return "OK";
    }
    else if (action == "SET_DUTY") {
        float d; iss >> d;
        _ctrl.set_manual_duty(d);
        return "OK";
    }
    // 【新增的 PID 设置分支】
    else if (action == "SET_PID") {
        float p, i, d;
        // 确保上位机发来了三个参数
        if (iss >> p >> i >> d) {
            _ctrl.set_pid(p, i, d);
            return "OK";
        } else {
            return "ERROR: Missing PID parameters. Format: SET_PID <P> <I> <D>";
        }
    }
    
    return "ERROR: Unknown Command";
}