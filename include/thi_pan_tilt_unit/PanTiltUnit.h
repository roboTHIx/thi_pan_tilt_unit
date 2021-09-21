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


#include <ros/ros.h>
#include <vector>


#include "dynamixel_sdk/dynamixel_sdk.h"


#if !defined(PANTILTUNIT_H)
#define PANTILTUNIT_H



//! @brief Class for the PanTiltUnit
//! @author Christian Pfitzner
//! @date 2021-09-21 Created
//! @date 2021-09-21 Last modified
class PanTiltUnit
{
public:

    /** @brief Constructor
     * Default constructor of class panTiltUnit
     */ 
    PanTiltUnit(const unsigned int id1, 
                const unsigned int id2, 
                const unsigned int baudrate=57600, 
                const std::string device="/dev/ttyUSB0"); 


    /** 
     * Default destructor 
     */
    ~PanTiltUnit(void);

    // setter function
    /** 
     * Function to set position of pan tilt unit
     * @param pan_position: pan position in degree
     * @param tilt_position: tilt position in degree
     * @return: true if successful, false otherwise
     */
    int setPosition(const std::vector<double> &position);

    /**
     * Function to set velocity
     * @param pan_velocity: pan velocity in degree per second
     * @param tilt_velocity: tilt velocity in degree per second
     * @return: true if successful, false otherwise
     * @note need to be in velocity mode
     * @see setVelocityMode
     */
    int setVelocity(const std::vector<double> &velocity);

    /**
     * Function to set position limits
     * @param pan_min: pan minimum position in degree
     * @param pan_max: pan maximum position in degree
     * @param tilt_min: tilt minimum position in degree
     * @param tilt_max: tilt maximum position in degree
     * @return: true if successful, false otherwise
     */
    int setPositionLimits(const std::vector<double> &position_limits);

    /**
     * Function to set velocity limits
     * @param velocity_limits
     * @return true if successful, false otherwise
     */
    int setVelocityLimits(const std::vector<double> &velocity_limits); 

    /**
     * Function to set leds individually
     */
    int setLEDs(const std::vector<bool> &leds);

    /** 
     * Function to switch on all LEDS
     * @return true if successful, false otherwise
     */
    int setLEDsOn(void); //

    /**
     * Function to switch off all LEDS
     * @return true if successful, false otherwise
     */
    int setLEDsOff(void); //

    /** 
     * Function to enable all motors
     * @return true if successful, false otherwise
     */
    int enableTorque(void); 

    /**
     * Function to disable all motors
     * @return true if successful, false otherwise
     */
    int disableTorque(void); 

    /**
     * Function to activate position mode
     * @return true if successful, false otherwise
     */
    int setPositionMode(void); //

    /**
     * Function to activate velocity mode
     * @return true if successful, false otherwise
     */
    int setVelocityMode(void); //


    // getter function
    /**
     * Get the current position of the pan-tilt unit
     * @return the current position of the pan-tilt unit
     * @see setPositionMode
     */
    std::vector<double> getPosition() const; 

    /**
     * Function to get current velocity of the pan-tilt unit
     * @return the current velocity of the pan-tilt unit
     * @see setVelocityMode
     */
    std::vector<double> getVelocity() const;

    /**
     * Function to the current limits of position
     * @return the current limits of position
     */
    std::vector<double> getPositionLimits() const;

    /**
     * Function to the current limits of velocity
     * @return the current limits of velocity
     */
    std::vector<double> getVelocityLimits() const;




private:
    unsigned int _id_1; // _id_1 of the first dynamixel in the pair
    unsigned int _id_2; // _id_2 of the second dynamixel in the pair

    dynamixel::PortHandler   *portHandler; 
    dynamixel::PacketHandler *packetHandler; // 

    dynamixel::GroupSyncRead*  groupSyncRead; 
    dynamixel::GroupSyncWrite* groupSyncWrite; 


    dynamixel::GroupSyncWrite* groupSyncWriteVel; 
};


#endif // PT
