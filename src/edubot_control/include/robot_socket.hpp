#ifndef _ROBOT_SOCKET
#define _ROBOT_SOCKET 1
#include <string>
#include <functional>

namespace robot_socket
{
    class RobotSocket
    {
    public:
        RobotSocket(){};
        RobotSocket(std::string host, int port);
        void write(std::string message);

        ///Blocking read from socket
        std::string read();

        void setReaderCb(std::function<void(std::string)> readerCallback);

    protected:
        std::function<void(std::string)> readerCallback;
        int sock;
    };
} // namespace robot_socket
#endif