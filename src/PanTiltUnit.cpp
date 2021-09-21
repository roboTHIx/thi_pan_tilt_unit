


#include <thi_pan_tilt_unit/PanTiltUnit.h>
#include <iostream>


#include "dynamixel_sdk_examples/SyncGetPosition.h"
#include "dynamixel_sdk_examples/SyncSetPosition.h"


using namespace dynamixel;

constexpr unsigned int ADDR_DRIVE_MODE       = 11;
constexpr unsigned int ADDR_TORQUE_ENABLE    = 64; 
constexpr unsigned int ADDR_LED              = 65;  // LED  
constexpr unsigned int ADDR_PRESENT_POSITION = 132;
constexpr unsigned int ADDR_GOAL_POSITION    = 116;
constexpr unsigned int ADDR_GOAL_VEL         = 104;
constexpr unsigned int PROTOCOL_VERSION      = 2.0;              // Default Protocol version of DYNAMIXEL X series.



PanTiltUnit::PanTiltUnit(const unsigned int id1, 
                         const unsigned int id2, 
                         const unsigned int baudrate, 
                         const std::string device)
: _id_1(id1), 
  _id_2(id2)
{

    portHandler         = PortHandler::getPortHandler(device.c_str());
    packetHandler       = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    groupSyncRead       = new GroupSyncRead( portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
    groupSyncWrite      = new GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION,     4);

    // velocity control mode
    groupSyncWriteVel   = new GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VEL,          4); 

    uint8_t dxl_error   = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    if (!portHandler->openPort()) 
    {
        ROS_ERROR("Failed to open the port!");
        // return -1;
    }

    if (!portHandler->setBaudRate(baudrate)) {
        ROS_ERROR("Failed to set the baudrate!");
        // return -1;
    }



    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, _id_1, 11, &dxl_error);
    ROS_INFO_STREAM("mode: " << dxl_comm_result);


    ROS_INFO_STREAM("Port is open for the control of dynamixels and the pan tilt unit. ");

}

PanTiltUnit::~PanTiltUnit()
{

  portHandler->closePort();

  delete portHandler;
  delete packetHandler;

  delete groupSyncWrite; 
  delete groupSyncRead;

  delete groupSyncWriteVel;
}




// setter function
int PanTiltUnit::setPosition(const std::vector<double> &position)
{
  uint8_t dxl_error       = 0;
  int dxl_comm_result     = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  uint8_t param_goal_position1[4];
  uint8_t param_goal_position2[4];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position1 = (unsigned int)position[0]; // Convert int32 -> uint32
  param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(position1));
  param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(position1));
  param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(position1));
  param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(position1));

  uint32_t position2 = (unsigned int)position[1]; // Convert int32 -> uint32
  param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(position2));
  param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(position2));
  param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(position2));
  param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(position2));

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_addparam_result = groupSyncWrite->addParam((uint8_t)_id_1, param_goal_position1);
  if (dxl_addparam_result != true) 
  {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", _id_1);
  }

  dxl_addparam_result = groupSyncWrite->addParam((uint8_t)_id_2, param_goal_position2);
  if (dxl_addparam_result != true) 
  {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", _id_2);
  }

  dxl_comm_result = groupSyncWrite->txPacket();
  if (dxl_comm_result == COMM_SUCCESS) 
  {
    ROS_DEBUG("setPosition : [ID:%d] [POSITION:%d]", _id_1, position1);
    ROS_DEBUG("setPosition : [ID:%d] [POSITION:%d]", _id_2, position2);
  } 
  else 
  {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    return -1; 
  }


  groupSyncWrite->clearParam();
    
  return 0; 
}


int PanTiltUnit::setVelocity(const std::vector<double> &velocity)
{

  int dxl_goal_velocity = 200;

  uint8_t dxl_error       = 0;
  int dxl_comm_result     = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, _id_1, 104, velocity[0], &dxl_error);
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, _id_2, 104, velocity[1], &dxl_error);
    
  return 0; 

}


int PanTiltUnit::setPositionLimits(const std::vector<double> &position_limits)
{


    return 0;
}


int PanTiltUnit::setVelocityLimits(const std::vector<double> &velocity_limits)
{



    return 0;
}


int PanTiltUnit::setLEDs(const std::vector<bool> &leds)
{
    if(leds.size() != 2)
    {
        std::cerr << "PanTiltUnit::setLEDs: ERROR: leds.size() != 2" << std::endl;
        return -1; 
    }

    uint8_t dxl_error   = 0;
    int dxl_comm_result = COMM_TX_FAIL;


    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_1, ADDR_LED, leds[0], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        ROS_ERROR_STREAM("Failed to enable LED: " << dxl_error);
        return -1;
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_2, ADDR_LED, leds[1], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        ROS_ERROR_STREAM("Failed to enable LED: " << dxl_error);
        return -1;
    }



    return 0;
}



  int PanTiltUnit::setLEDsOn(void) 
  {
    std::vector<bool> ledsOn =  {true, true};
    return this->setLEDs(ledsOn);
  }


  int PanTiltUnit::setLEDsOff(void) 
  {
    std::vector<bool> ledsOff =  {false, false};
    return this->setLEDs(ledsOff);
  }


int PanTiltUnit::enableTorque(void)
{
  uint8_t dxl_error   = 0;
  int dxl_comm_result = COMM_TX_FAIL;


      // set up torque and finish configuration
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_1, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", _id_1);
    return -1;
  }


  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_2, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", _id_2);
    return -1;
  }



  return 0; 
}






int PanTiltUnit::disableTorque(void)
{
  uint8_t dxl_error   = 0;
  int dxl_comm_result = COMM_TX_FAIL;


      // set up torque and finish configuration
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_1, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", _id_1);
    return -1;
  }


  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_2, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", _id_2);
    return -1;
  }



  return 0; 

}





int PanTiltUnit::setPositionMode(void)
{
  uint8_t dxl_error   = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // set the position mode for all drives
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_1, ADDR_DRIVE_MODE, 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to set position mode %d", _id_1);
    return -1;
  }
  
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_2, ADDR_DRIVE_MODE, 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to set position mode %d", _id_2);
    return -1;
  }


  return 0; 
}


int PanTiltUnit::setVelocityMode(void)
{
  uint8_t dxl_error   = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // set the position mode for all drives
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_1, ADDR_DRIVE_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to set position mode %d", _id_1);
    return -1;
  }
  
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, _id_2, ADDR_DRIVE_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to set position mode %d", _id_2);
    return -1;
  }

  return 0; 
}


std::vector<double> PanTiltUnit::getPosition(void) const
{
  uint8_t dxl_error       = 0;
  int dxl_comm_result     = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position1 = 0;
  int32_t position2 = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_addparam_result = groupSyncRead->addParam((uint8_t)_id_1);
  if (dxl_addparam_result != true) 
  {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", _id_1);
  }

  dxl_addparam_result = groupSyncRead->addParam((uint8_t)_id_2);
  if (dxl_addparam_result != true) 
  {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", _id_2);
  }

  std::vector<double> posRet = {0.0, 0.0};

  position1 = groupSyncRead->getData((uint8_t)_id_1, ADDR_PRESENT_POSITION, 4);
  position2 = groupSyncRead->getData((uint8_t)_id_2, ADDR_PRESENT_POSITION, 4);

  posRet[0] = position1; 
  posRet[1] = position2;

  groupSyncRead->clearParam();

  return posRet;


}