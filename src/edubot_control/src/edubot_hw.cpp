#include <ros/ros.h>
#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include "edubot_hw.hpp"
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "robot_socket.hpp"
#include <sstream>
#include <math.h>
#define PULSES_PER_REV 1496.0
#define WHEEL_DIAMETER 0.065

namespace edubot_hardware_interface
{

    EdubotInterface::EdubotInterface()
    {
    }

    EdubotInterface::~EdubotInterface()
    {
    }

    bool EdubotInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        //get joint names and num of joint
        robot_hw_nh.getParam("/edubot/robot_hardware/joints", joint_name);
        num_joints = joint_name.size();

        robot_socket = robot_socket::RobotSocket("192.168.0.8", 8080);
        robot_socket.setReaderCb([this](std::string message) {
            std::size_t from = 0;
            std::size_t index = 0;
            auto to = message.find(',', from);
            double d_time = std::stof(message.substr(from, to));
            from = to + 1;

            to = message.find(',', from);
            double left_count = std::stod(message.substr(from, to));
            from = to + 1;

            to = message.find(',', from);
            double right_count = std::stod(message.substr(from, to));
            from = to + 1;

            double left_dist = WHEEL_DIAMETER * M_PI * left_count / PULSES_PER_REV;
            double right_dist = WHEEL_DIAMETER * M_PI * right_count / PULSES_PER_REV;

            temp_joint_velocity_state[0] = (temp_joint_position_state[0] - left_dist) / d_time;
            temp_joint_velocity_state[1] = (temp_joint_position_state[1] - right_dist) / d_time;
            temp_joint_position_state[0] = left_dist;
            temp_joint_position_state[1] = right_dist;
        });

        //resize vectors
        temp_joint_position_state.resize(num_joints);
        temp_joint_velocity_state.resize(num_joints);
        joint_position_state.resize(num_joints);
        joint_velocity_state.resize(num_joints);
        joint_effort_state.resize(num_joints);
        joint_effort_command.resize(num_joints);

        //Register handles
        for (int i = 0; i < num_joints; i++)
        {
            //State
            hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
            joint_state_interface.registerHandle(jointStateHandle);

            //Effort
            hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command[i]);
            effort_joint_interface.registerHandle(jointEffortHandle);
        }

        //Register interfaces
        registerInterface(&joint_state_interface);
        registerInterface(&effort_joint_interface);

        //return true for successful init or ComboRobotHW initialisation will fail
        return true;
    }

    void EdubotInterface::read(const ros::Time &time, const ros::Duration &period)
    {
        static auto start_time = time;
        for (int i = 0; i < num_joints; i++)
        {
            //TODO: Reading from temp vectors should be done in a mutex/lock
            std::cout << temp_joint_velocity_state[i] << std::endl;
            joint_velocity_state[i] = temp_joint_velocity_state[i];
            joint_position_state[i] = temp_joint_position_state[i];
        }
    }

    void EdubotInterface::write(const ros::Time &time, const ros::Duration &period)
    {
        if (num_joints > 0)
        {
            std::ostringstream sstream;
            sstream << "w";
            for (int i = 0; i < num_joints - 1; i++)
            {
                sstream << joint_effort_command[i] << ":";
            }
            sstream << joint_effort_command[num_joints - 1] << "\n";
            robot_socket.write(sstream.str().c_str());
        }
    }

} // namespace edubot_hardware_interface
PLUGINLIB_EXPORT_CLASS(edubot_hardware_interface::EdubotInterface, hardware_interface::RobotHW)