

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>


ros::Publisher pantilt_pub; 




void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // ROS_ERROR_STREAM(__PRETTY_FUNCTION__);



    std_msgs::Float64MultiArray msg;
    msg.data.resize(2); 
    msg.data[0] = 1000*joy->axes[3];
    msg.data[1] = 1000*joy->axes[4];
    pantilt_pub.publish(msg);


}



int main(int argc, char **argv)
{





    ros::init(argc, argv, "joy2pantilt_node");

    ros::NodeHandle n;


    ros::Subscriber joy_sub = n.subscribe("joy", 1000, joyCallback);
    pantilt_pub = n.advertise<std_msgs::Float64MultiArray>("pan_tilt/position", 1000);




    ros::spin(); 
}