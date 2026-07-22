// calm_net.hpp
#ifndef CALM_NET_HPP
#define CALM_NET_HPP

#include "calm_ctrl.hpp"
#include <string>

class CalmServer {
public:
    CalmServer(CalmController& ctrl, int port = 8080);
    void run();
private:
    void handle_client(int client_fd);
    std::string process_command(const std::string& cmd);
    CalmController& _ctrl;
    int _port;
};

#endif