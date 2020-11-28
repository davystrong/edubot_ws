#include "robot_socket.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <exception>
#include <string>
#include <iostream>
#include <thread>
#define SOCKET_TIMEOUT_SECONDS 1

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

        struct timeval tv;
        //TODO: On Windows this has to be multiplied by 1000, I think
        tv.tv_sec = SOCKET_TIMEOUT_SECONDS;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            throw std::runtime_error("Connection Failed");
        }
    }

    RobotSocket::~RobotSocket()
    {
        if (read_flag)
        {
            std::cout << "Joining reader thread" << std::endl;
            read_flag = false;
            reader_thread.join();
            std::cout << "Thread joined" << std::endl;
        }
    }

    RobotSocket &RobotSocket::operator=(RobotSocket &&obj)
    {
        reader_thread = std::move(obj.reader_thread);
        reader_callback = std::move(obj.reader_callback);
        sock = obj.sock;
        read_flag = obj.read_flag;
        return *this;
    };

    std::string RobotSocket::read()
    {
        //TODO: This should buffer and only return until the first newline
        int valread;
        char buffer[1024] = {0};
        valread = ::read(sock, buffer, 1024);
        if (valread == -1)
        {
            throw robot_socket::TimeoutException();
        }
        else if (valread == 0)
        {
            throw robot_socket::DisconnectedException();
        }
        return buffer;
    }

    void RobotSocket::setReaderCb(std::function<void(std::string)> new_reader_callback)
    {
        reader_callback = new_reader_callback;
        if (!read_flag)
        {
            read_flag = true;
            reader_thread = std::thread([this] {
                while (read_flag)
                {
                    try
                    {
                        std::string message = this->read();
                        (this->reader_callback)(message);
                    }
                    catch (robot_socket::TimeoutException e)
                    {
                        //Timeout. Do nothing
                    }
                    catch (robot_socket::DisconnectedException e)
                    {
                        read_flag = false;
                    }
                }
            });
            // reader_thread.detach();
        }
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