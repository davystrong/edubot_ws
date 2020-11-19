#include "robot_socket.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <exception>
#include <string>
#include <iostream>
#include <thread>

namespace robot_socket
{
    RobotSocket::RobotSocket(std::string host, int port)
    {
        sockaddr_in serv_addr;
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            throw std::runtime_error("Socket creation error");
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, host.c_str(), &serv_addr.sin_addr) <= 0)
        {
            throw std::runtime_error("Invalid address/ Address not supported");
        }

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            throw std::runtime_error("Connection Failed");
        }
    }

    std::string RobotSocket::read()
    {
        int valread;
        char buffer[1024] = {0};
        valread = ::read(sock, buffer, 1024);
        return buffer;
    }

    

    void RobotSocket::write(std::string message)
    {
        if (sock == 0)
        {
            throw std::runtime_error("Socket has not been initialised");
        }
        ::send(sock, message.c_str(), message.length(), 0);
    }
} // namespace robot_socket