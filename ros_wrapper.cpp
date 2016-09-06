#include "ros_wrapper.hpp"

RosWrapper::RosWrapper(int argc, char * argv[], 
    const char * name, const char * publisher_name)
{
    ros::init(argc, argv, name);
    ros::NodeHandle n;
    m_publisher = n.advertise<std_msgs::Float32MultiArray>(publisher_name, 1000); 
}

RosWrapper::~RosWrapper()
{
    ros::shutdown();
}
