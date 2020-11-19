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
        // num_joints = 1;

        //resize vectors
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
            if ((time - start_time).toSec() > 10)
            {
                joint_velocity_state[i] = 1.0;
                joint_position_state[i] = 1.0;
                joint_effort_state[i] = 1.0;
            }
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