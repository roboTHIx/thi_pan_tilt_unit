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
#include <sensor_msgs/Joy.h>

PanTiltUnit* panTiltUnit = nullptr; 
std::vector<double> pos = {0.0, 0.0};


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

    const auto vel_pan   = msg->axes[3]*50.0; //
    const auto vel_tilt  = msg->axes[4]*50.0; 

    // std::vector<double> pos = {std::fabs(msg->axes[3]*1023.0), std::fabs(msg->axes[4])*1023.0};

    pos[0] = vel_pan  + pos[0]; 
    pos[1] = vel_tilt + pos[1];


    if(msg->buttons[5] == 1)
    {
        pos[0] = 0.0;
        pos[1] = 0.0;
    }
    panTiltUnit->setPosition(pos);
    panTiltUnit->getPosition(); 
}


int main(int argc, char ** argv)
{

    ROS_INFO_STREAM("Starting ros node for pan tilt unit"); 


    // initialize ros node environment
    ros::init(argc, argv, "thi_pan_tilt_node");
    ros::NodeHandle n; 


    ros::Subscriber joySub = n.subscribe("joy", 1000, joyCallback);


    panTiltUnit = new PanTiltUnit(1,2);
    panTiltUnit->setLEDsOn(); 
    std::vector<double> pos = {0.0, 0.0}; 
    panTiltUnit->setPositionMode();
    panTiltUnit->enableTorque(); 

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }    


    panTiltUnit->disableTorque(); 
    panTiltUnit->setLEDsOff(); 

    
    return 0; 
}