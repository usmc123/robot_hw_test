#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>


namespace robot_hw_test_ns
{
    class MyRobotInterface : public hardware_interface::RobotHW
    {

    public:
    MyRobotInterface();
    MyRobotInterface(ros::NodeHandle& nh);
    ~MyRobotInterface();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);
    private:
    ros::NodeHandle nh_;
    //interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;
    
    int num_joints;
    std::vector<std::string> joint_name;

    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    std::vector<double> joint_effort_command;

    };
}