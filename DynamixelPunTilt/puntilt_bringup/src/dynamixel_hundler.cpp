#include "dynamixel_hundler.h"
#include <boost/bind.hpp>

void DynamixelHundler::CallbackCommand(
    const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    std::vector<int32_t> positions(now_positions_);
    auto last_point = msg->points.back();
    for (int i = 0; i < msg->joint_names.size(); i++) {
        auto joint_name = msg->joint_names[i];
        auto angle = last_point.positions[i];
        for (int j = 0; j < joint_list_.size(); j++) {
            if (joint_list_[j] == joint_name) {
                positions[j] = (int)(angle * 4096 / (2 * M_PI) + 4096 / 2);
            }
        }
    }
    // todo JointTrajectoryを角度指令にちゃんと反映指せるようにしないと...
    // 現状は最後のtrajectory pointに最速で向かうようになってしまっている．

    WritePosisionsAll(positions);
}

void DynamixelHundler::ActivateAll() {
    for (const int id : id_list_) {
        uint8_t dxl_error = 0;
        const int dxl_comm_result = packetHandler_->write1ByteTxRx(
            portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", id);
    }
}

void DynamixelHundler::WritePosisionsAll(
    const std::vector<int32_t> &position_list) {
    if (position_list.size() != id_list_.size()) {
        ROS_ERROR("Length of position_list is mismatch to number of Dynamixel "
                  "in WritePosisionsAll.");
        return;
    }

    for (size_t i = 0; i < id_list_.size(); ++i) {
        uint8_t id = id_list_[i];
        uint32_t position = (unsigned int)(position_list[i]);
        uint8_t param_goal_position[4] = {
            DXL_LOBYTE(DXL_LOWORD(position)), DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)), DXL_HIBYTE(DXL_HIWORD(position))};
        const int dxl_addparam_result =
            groupSyncWritePosition_.addParam(id, param_goal_position);
        if (dxl_addparam_result != true)
            ROS_ERROR(
                "Failed to addparam to groupSyncWrite for Dynamixel ID %d", id);
    }

    const int dxl_comm_result = groupSyncWritePosition_.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        return;
    }

    for (size_t i = 0; i < id_list_.size(); ++i) {
        target_positions_[i] = position_list[i];
        // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", id_list_[i], position_list[i]);
    }
    groupSyncWritePosition_.clearParam();
}

bool DynamixelHundler::ReadPositionsAll() {
    now_positions_.resize(id_list_.size());

    int dxl_addparam_result = false;
    for (const int id : id_list_) {
        dxl_addparam_result = groupSyncReadPosition_.addParam(id);
        if (dxl_addparam_result != true) {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d",
                      id);
            return false;
        }
    }

    // Read positions from groupSyncRead
    const int dxl_comm_result = groupSyncReadPosition_.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
        groupSyncReadPosition_.clearParam();
        return false;
    }

    for (size_t i = 0; i < id_list_.size(); ++i) {
        int32_t position = groupSyncReadPosition_.getData(
            id_list_[i], ADDR_PRESENT_POSITION, 4);
        now_positions_[i] = position;
        // ROS_INFO("ReadPosition : [ID:%d] [POSITION:%d]", id_list_[i], position);
    }
    groupSyncReadPosition_.clearParam();
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_hundler_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    std::string DEVICE_NAME; if (!nh_p.getParam("DEVICE_NAME", DEVICE_NAME)) DEVICE_NAME = "/dev/ttyUSB0";
    int BAUDRATE;            if (!nh_p.getParam("BAUDRATE",    BAUDRATE)   ) BAUDRATE = 57600;
    std::vector<int> id_list;            if (!nh_p.getParam("id_list",    id_list)   ){ROS_ERROR("Failed to get param 'id_list'"); return 0;}
    std::vector<std::string> joint_list; if (!nh_p.getParam("joint_list", joint_list)){ROS_ERROR("Failed to get param 'joint_list'"); return 0;}

    DynamixelHundler dxl_hundler(DEVICE_NAME, BAUDRATE, id_list, joint_list);
    dxl_hundler.ActivateAll();

    ros::Subscriber sub_joint_trajectory = nh.subscribe<trajectory_msgs::JointTrajectory>("/puntilt_controller/command", 10, boost::bind(&DynamixelHundler::CallbackCommand, &dxl_hundler, _1));
    ros::Publisher pub_joint_state = nh.advertise<control_msgs::JointTrajectoryControllerState>("/puntilt_controller/state_tmp", 10);

    ros::Rate rate(10);
    while (ros::ok()) {
        if (dxl_hundler.ReadPositionsAll()) {
            control_msgs::JointTrajectoryControllerState joint_state_msg;
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.joint_names = joint_list;
            for (auto i = 0; i < dxl_hundler.id_list().size(); i++) {
                joint_state_msg.desired.positions.push_back(
                    dxl_hundler.target_positions()[i] * (2 * M_PI) / 4096);
                joint_state_msg.actual.positions.push_back(
                    dxl_hundler.now_positions()[i] * (2 * M_PI) / 4096);
                joint_state_msg.error.positions.push_back(
                    joint_state_msg.desired.positions[i] -
                    joint_state_msg.actual.positions[i]);
            }
            pub_joint_state.publish(joint_state_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
