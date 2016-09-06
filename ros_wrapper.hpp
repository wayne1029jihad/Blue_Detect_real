#ifndef _ROS_WRAPPER_HPP_
#define _ROS_WRAPPER_HPP_

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include "stdio.h"   

class RosWrapper
{
public:
    RosWrapper(int argc, char * argv[], const char * name, const char * publisher_name);
    ~RosWrapper();

    template <typename T>
    void publish(T msg);
private:
    ros::Publisher m_publisher;
};


template <typename T>
void RosWrapper::publish(T msg)
{
    ros::Rate rate(10);
    m_publisher.publish(msg); 
    rate.sleep();
}

#endif
