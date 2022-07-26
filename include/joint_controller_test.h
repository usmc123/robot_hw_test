#include <ros/node_handle.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>


#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <pluginlib/class_list_macros.h>


namespace robot_hw_test_ns
{
    class joint_controller_test: public controller_interface::MultiInterfaceController
                                    <hardware_interface::EffortJointInterface>
    {
    private:

    std::vector<hardware_interface::JointHandle> joint_handles_;
    ros::NodeHandle nh;
    std::vector<double> joint_state_;


        /* data */
    public:
        joint_controller_test();
        ~joint_controller_test();

    bool init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh);
    void starting(const ros::Time& time_now);
    void update(const ros::Time& time_now, const ros::Duration& period);
    };
}