/* Copyright (c), Prof. Dr. Christian Pfitzner, All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
 */


#include <thi_pan_tilt_unit/PanTiltUnit.h>
// #include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

PanTiltUnit* panTiltUnit = nullptr; 
std::vector<double> pos = {0.0, 0.0};

struct VALUES
{
    float axis_pan  = 0.0;
    float axis_tilt = 0.0;
    float btn_reset = 0.0;  
    float btn_reset_buf = 0.0;
} joy_values;



void update_pan_tilt_unit()
{


    const auto vel_pan  = joy_values.axis_pan  * 15.0;
    const auto vel_tilt = joy_values.axis_tilt * 15.0;

    pos[1] = vel_pan  + pos[1]; 
    pos[0] = vel_tilt + pos[0];

    if(pos[0] >= 3650)
        pos[0] = 3650;
    else if(pos[0] <= 2180)
        pos[0] = 2180;
    
    if(pos[1] <= 2050)
        pos[1] = 2050;
    else if(pos[1] >= 4060)
        pos[1] = 4060;

    if(joy_values.btn_reset == 1.0 && joy_values.btn_reset_buf == 0.0)
    {
        if(pos[0] == 2760.0 && pos[1] == 3057.0)
        {
            pos[0] = 2200.0;
            pos[1] = 3057.0;
        }
        else if (pos[0] == 2200.0 && pos[1] == 3057.0)
        {
            pos[0] = 2760.0;
            pos[1] = 3057.0;
        }
        else
        {
            pos[0] = 2760.0;
            pos[1] = 3057.0;
        }

    }

    panTiltUnit->setPosition(pos);
    panTiltUnit->getPosition();   

    joy_values.btn_reset_buf = joy_values.btn_reset;

    // ROS_INFO("tilt: %f, pan: %f", pos[0], pos[1]);

}

void cb_axis_pan(const std_msgs::Float64::ConstPtr& msg)
{
    joy_values.axis_pan = msg->data;
}

void cb_axis_tilt(const std_msgs::Float64::ConstPtr& msg)
{
    joy_values.axis_tilt = msg->data;
}

void cb_btn_reset(const std_msgs::Float64::ConstPtr& msg)
{
    joy_values.btn_reset = msg->data;
}

// void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
// {

//     const auto vel_pan   = msg->axes[3]*50.0; //
//     const auto vel_tilt  = msg->axes[4]*50.0; 

//     // std::vector<double> pos = {std::fabs(msg->axes[3]*1023.0), std::fabs(msg->axes[4])*1023.0};

//     pos[0] = vel_pan  + pos[0]; 
//     pos[1] = vel_tilt + pos[1];


//     if(msg->buttons[5] == 1)
//     {
//         pos[0] = 0.0;
//         pos[1] = 0.0;
//     }
//     panTiltUnit->setPosition(pos);
//     panTiltUnit->getPosition(); 
// }


int main(int argc, char ** argv)
{

    ROS_INFO_STREAM("Starting ros node for pan tilt unit"); 


    // initialize ros node environment
    ros::init(argc, argv, "thi_pan_tilt_node");
    ros::NodeHandle n; 


    // ros::Subscriber joySub = n.subscribe("joy", 1000, joyCallback);
    ros::Subscriber sub_axis_pan = n.subscribe("axis_pan", 10, cb_axis_pan);
    ros::Subscriber sub_axis_tilt = n.subscribe("axis_tilt", 10, cb_axis_tilt);
    ros::Subscriber sub_btn_reset = n.subscribe("btn_reset", 10, cb_btn_reset);



    panTiltUnit = new PanTiltUnit(1,2);
    panTiltUnit->setLEDsOn(); 
    std::vector<double> pos = {0.0, 0.0}; 
    panTiltUnit->setPositionMode();
    panTiltUnit->enableTorque();

    // pos[0] = 3500.0;
    // pos[1] = 3082.0;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        update_pan_tilt_unit();

        loop_rate.sleep();
        ros::spinOnce();
    }    


    panTiltUnit->disableTorque(); 
    panTiltUnit->setLEDsOff(); 

    
    return 0; 
}