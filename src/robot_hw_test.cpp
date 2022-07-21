#include "robot_hw_test.h"
namespace robot_hw_test_ns
{
    MyRobotInterface::MyRobotInterface()
    {
        ;
    }
    MyRobotInterface::MyRobotInterface(ros::NodeHandle& nh)
    {

        ;
    }

    MyRobotInterface::~MyRobotInterface()
    {;}

   bool  MyRobotInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
    {
        bool ret = robot_hw_nh.getParam("/mylink/robot_hw_test/joints", joint_name);//从参数服务器中获取name

        ROS_ERROR("getParam ret= %d",ret);
        
        num_joints = joint_name.size();
        for (size_t i = 0; i < num_joints; i++)
        {
            ROS_ERROR("jointname=%s",joint_name[i].c_str());
        }

        joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_effort_command.resize(num_joints);


    for(int i=0; i<num_joints; i++)
    {
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i].c_str(), &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);
        ROS_INFO("joint_name[%d].c_str()=%s",i,jointStateHandle.getName().c_str());

        //Effort
        hardware_interface::JointHandle jointEffortHandle(joint_state_interface.getHandle(joint_name[i]), &joint_effort_command[i]);
        effort_joint_interface.registerHandle(jointEffortHandle);
    }

    registerInterface(&joint_state_interface);          //将类中的接口注册到ros中

    registerInterface(&effort_joint_interface);         
    return true;
    }

void MyRobotInterface::read(const ros::Time& time, const ros::Duration& period)
{
    static int t =0 ;
    if (t<3)
    {
        ROS_ERROR("read is run");
    }
    t++;
    

    double temp = std::sin(time.toSec());
    for(int i=0;i < num_joints;i++){
        joint_position_state[i]=temp+1;
        }
}

void MyRobotInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        for(int i=0;i < num_joints;i++)
        {
         ROS_WARN("recive from controller joint %d is %f",i,joint_effort_command[i]);
        
        }
   
    }
}
PLUGINLIB_EXPORT_CLASS(robot_hw_test_ns::MyRobotInterface, hardware_interface::RobotHW)