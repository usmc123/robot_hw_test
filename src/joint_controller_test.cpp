#include "joint_controller_test.h"

namespace robot_hw_test_ns
{

    joint_controller_test::joint_controller_test(/* args */)
    {
    }
    
    joint_controller_test::~joint_controller_test()
    {
    }


    bool joint_controller_test::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh)
    {
        std::vector<std::string> joint_names;
        if (!nh.getParam("joint",joint_names))
        {
        ROS_ERROR(
        "Agv_Controller: Invalid or no joint_names parameters provided, aborting "
        "controller init!fuck!!!!!");
        return false;
        }
        auto* joint_interface=robot_hw->get<hardware_interface::EffortJointInterface>();

        for (uint8_t i = 0; i < joint_names.size(); i++)
        {
            joint_handles_.push_back(joint_interface->getHandle(joint_names[i]));
            ROS_ERROR_STREAM("controller info: my name is "<<joint_handles_[i].getName());
        }

        joint_state_.resize(joint_names.size());
        return true;    
    }


    void joint_controller_test::starting(const ros::Time&)
    {
        ROS_WARN("controller starting");
    }


    void joint_controller_test::update(const ros::Time& time_now, const ros::Duration& period)
    {
        double temp = std::cos(time_now.toSec());
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(temp);
            ROS_ERROR("recive from robot_hw feedback is %f",joint_handles_[i].getPosition());
        }
    }
}       //namespace
PLUGINLIB_EXPORT_CLASS(robot_hw_test_ns::joint_controller_test,
                       controller_interface::ControllerBase)
    
