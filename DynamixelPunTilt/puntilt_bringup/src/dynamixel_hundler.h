// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun puntilt_bringup dynamixel_hundler
 *
 * Open terminal #3 (run one of below commands at a time)
 *
 * Author: Ryohei Michikawa
*******************************************************************************/

#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>
#include <vector>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"

using namespace dynamixel;

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

class DynamixelHundler{
public:
    enum ControlTableAddress {
        ADDR_TORQUE_ENABLE    = 64 ,
        ADDR_PRESENT_POSITION = 132,
        ADDR_GOAL_POSITION    = 116,
    };
    DynamixelHundler(std::string port_name, int baudrate, 
                     std::vector<int> id_list, std::vector<std::string> joint_list)
        :   portHandler_(PortHandler::getPortHandler(port_name.c_str())),
            packetHandler_(PacketHandler::getPacketHandler(PROTOCOL_VERSION)),
            groupSyncReadPosition_(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4),
            groupSyncWritePosition_(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4)    {

        if (!portHandler_->openPort()) {
            ROS_ERROR("Failed to open the port!");
        }

        if (!portHandler_->setBaudRate(baudrate)) {
            ROS_ERROR("Failed to set the baudrate!");
        }

        if(id_list.size() != joint_list.size()){
            ROS_ERROR("Length of id_list is mismatch to length of joint_list.");
        }

        id_list_ = id_list;
        joint_list_ =joint_list;
        bool success = ReadPositionsAll(); // set now_positions_
        if(success) target_positions_ = now_positions_;
    }

    ~DynamixelHundler(){
        portHandler_->closePort();
        ROS_INFO("Close the port!");
    }

    void CallbackCommand(const trajectory_msgs::JointTrajectory::ConstPtr&);

    void ActivateAll();
    void WritePosisionsAll(const std::vector<int32_t>&);
    bool ReadPositionsAll ();

    std::vector<int32_t> target_positions(){return target_positions_;};
    std::vector<int32_t> now_positions(){return now_positions_;};
    std::vector<int32_t> id_list(){return id_list_;};
    std::vector<std::string> joint_list(){return joint_list_;};

private:
    PortHandler* portHandler_;
    PacketHandler* packetHandler_;
    GroupSyncRead groupSyncReadPosition_;
    GroupSyncWrite groupSyncWritePosition_;
    std::vector<int> id_list_;
    std::vector<std::string> joint_list_;
    std::vector<int32_t> target_positions_;
    std::vector<int32_t> now_positions_;
};
