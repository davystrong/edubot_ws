#ifndef _ROBOT_SOCKET
#define _ROBOT_SOCKET 1
#include <string>
#include <functional>
#include <thread>

namespace robot_socket
{
    class TimeoutException : std::exception
    {
        const char *what() const throw()
        {
            return "Socket timed out (returned -1)";
        }
    };

    class DisconnectedException : std::exception
    {
        const char *what() const throw()
        {
            return "Socket disconnected (returned 0)";
        }
    };

    class RobotSocket
    {
    public:
        RobotSocket() = default;
        ~RobotSocket();
        RobotSocket &operator=(RobotSocket &&obj);
        RobotSocket(std::string host, int port);
        void write(std::string message);

        ///Blocking read from socket
        std::string read();

        void setReaderCb(std::function<void(std::string)> newReaderCallback);

    protected:
        std::function<void(std::string)> reader_callback;
        int sock;
        bool read_flag = false;
        std::thread reader_thread;
    };
} // namespace robot_socket
#endif