#include "dynamixel_hundler.h"
#include <boost/bind.hpp>

void DynamixelHundler::CallbackCommand(
    const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    std::vector<int32_t> positions(now_positions_);
    std::vector<int32_t> max_vels(joint_list_.size(), 0);
    std::vector<int32_t> max_accs(joint_list_.size(), 0);
    auto last_point = msg->points.back();
    if (msg->points.size() > 1) ROS_WARN("This program cannot support this points size");

    for (int i = 0; i < msg->joint_names.size(); i++) {
        auto joint_name = msg->joint_names[i];
        auto angle = last_point.positions[i];
        auto time = last_point.time_from_start.toSec();
        for (int j = 0; j < joint_list_.size(); j++) {
            if (joint_list_[j] == joint_name) {
                positions[j] = (int)(angle * 4096 / (2 * M_PI) + 4096 / 2);
                const double pos_diff = abs(positions[j] -  now_positions_[j]) / (4096 / (2 * M_PI)); // [rad]
                const double acc = (4 * pos_diff / (time*time)) * 2; // [rad/s^2]
                const double vel = acc*time/2 - sqrt( (acc*time/2)*(acc*time/2) - acc*pos_diff ); // [rad/s]
                max_accs[j] = (int)(std::max(1.0, acc/(2*M_PI)*3600 / 214.577)); // [rev/min^2]
                max_vels[j] = (int)(std::max(1.0, vel/(2*M_PI)*60   / 0.229 ));  // [rev/min]
                // ROS_INFO("pd %f, acc %f, vel %f, acc %d, vel %d", pos_diff, acc, vel, max_vels[j], max_accs[j]);
            }
        }
    }
    WritePositionsProfilesAll(positions, max_vels, max_vels);
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

void DynamixelHundler::WritePositionsAll(
    const std::vector<int32_t> &position_list) {
        
    if (position_list.size() != id_list_.size()) {ROS_ERROR("Length of position_list is mismatch to number of Dynamixel in WritePositionsAll."); return;}

    for (size_t i = 0; i < id_list_.size(); ++i) {
        uint8_t id = id_list_[i];
        uint32_t position = (unsigned int)(position_list[i]);
        uint8_t param_goal_position[4] = {
            DXL_LOBYTE(DXL_LOWORD(position)), DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)), DXL_HIBYTE(DXL_HIWORD(position))};
        const int dxl_addparam_result =
            syncWritePosition_.addParam(id, param_goal_position);
        if (dxl_addparam_result != true)
            ROS_ERROR(
                "Failed to addparam to groupSyncWrite for Dynamixel ID %d", id);
    }

    const int dxl_comm_result = syncWritePosition_.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        return;
    }

    for (size_t i = 0; i < id_list_.size(); ++i) {
        target_positions_[i] = position_list[i];
        // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", id_list_[i], position_list[i]);
    }
    syncWritePosition_.clearParam();
}

bool DynamixelHundler::ReadPositionsAll() {
    now_positions_.resize(id_list_.size());

    int dxl_addparam_result = false;
    for (const int id : id_list_) {
        dxl_addparam_result = syncReadPosition_.addParam(id);
        if (dxl_addparam_result != true) {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", id);
            return false;
        }
    }

    // Read positions from groupSyncRead
    const int dxl_comm_result = syncReadPosition_.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
        syncReadPosition_.clearParam();
        return false;
    }

    for (size_t i = 0; i < id_list_.size(); ++i) {
        int32_t position = syncReadPosition_.getData(id_list_[i], ADDR_PRESENT_POSITION, 4);
        now_positions_[i] = position;
        // ROS_INFO("ReadPosition : [ID:%d] [POSITION:%d]", id_list_[i], position);
    }
    syncReadPosition_.clearParam();
    return true;
}


void DynamixelHundler::WriteProfilesAll(
    const std::vector<int32_t> &max_vel_list,
    const std::vector<int32_t> &max_acc_list) {

    if (max_vel_list.size() != id_list_.size()) { ROS_ERROR("Length of max_vel_list is mismatch to number of Dynamixel in WritePositionsProfilesAll."); return;}
    if (max_acc_list.size() != id_list_.size()) { ROS_ERROR("Length of max_acc_list is mismatch to number of Dynamixel in WritePositionsProfilesAll."); return;}

    for (size_t i = 0; i < id_list_.size(); ++i) {
        uint8_t id = id_list_[i];
        uint32_t value_acc = (unsigned int)(max_acc_list[i]);
        uint32_t value_vel = (unsigned int)(max_vel_list[i]);
        uint8_t param_values[8] = {
            DXL_LOBYTE(DXL_LOWORD(value_acc)), DXL_HIBYTE(DXL_LOWORD(value_acc)),
            DXL_LOBYTE(DXL_HIWORD(value_acc)), DXL_HIBYTE(DXL_HIWORD(value_acc)),
            DXL_LOBYTE(DXL_LOWORD(value_vel)), DXL_HIBYTE(DXL_LOWORD(value_vel)),
            DXL_LOBYTE(DXL_HIWORD(value_vel)), DXL_HIBYTE(DXL_HIWORD(value_vel))};
        const int dxl_addparam_result =
            syncWriteProfile_.addParam(id, param_values);
        if (dxl_addparam_result != true)
            ROS_ERROR(
                "Failed to addparam to groupSyncWrite for Dynamixel ID %d", id);
    }

    const int dxl_comm_result = syncWriteProfile_.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to set profile! Result: %d", dxl_comm_result);
        return;
    }

    syncWriteProfile_.clearParam();
}

void DynamixelHundler::WritePositionsProfilesAll(
    const std::vector<int32_t> &position_list,
    const std::vector<int32_t> &max_vel_list,
    const std::vector<int32_t> &max_acc_list) {

    if (position_list.size() != id_list_.size()) {ROS_ERROR("Length of position_list is mismatch to number of Dynamixel in WritePositionsAll."); return;}
    if (max_vel_list.size() != id_list_.size()) { ROS_ERROR("Length of max_vel_list is mismatch to number of Dynamixel in WritePositionsProfilesAll."); return;}
    if (max_acc_list.size() != id_list_.size()) { ROS_ERROR("Length of max_acc_list is mismatch to number of Dynamixel in WritePositionsProfilesAll."); return;}

    for (size_t i = 0; i < id_list_.size(); ++i) {
        uint8_t id = id_list_[i];
        uint32_t value_acc = (unsigned int)(max_acc_list[i]);
        uint32_t value_vel = (unsigned int)(max_vel_list[i]);
        uint32_t position = (unsigned int)(position_list[i]);
        uint8_t param_values[12] = {
            DXL_LOBYTE(DXL_LOWORD(value_acc)), DXL_HIBYTE(DXL_LOWORD(value_acc)),
            DXL_LOBYTE(DXL_HIWORD(value_acc)), DXL_HIBYTE(DXL_HIWORD(value_acc)),
            DXL_LOBYTE(DXL_LOWORD(value_vel)), DXL_HIBYTE(DXL_LOWORD(value_vel)),
            DXL_LOBYTE(DXL_HIWORD(value_vel)), DXL_HIBYTE(DXL_HIWORD(value_vel)),
            DXL_LOBYTE(DXL_LOWORD(position)), DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)), DXL_HIBYTE(DXL_HIWORD(position))};
        const int dxl_addparam_result =
            syncWritePositionProfile_.addParam(id, param_values);
        if (dxl_addparam_result != true)
            ROS_ERROR(
                "Failed to addparam to groupSyncWrite for Dynamixel ID %d", id);
    }

    const int dxl_comm_result = syncWritePositionProfile_.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to set position and profile! Result: %d", dxl_comm_result);
        return;
    }

    for (size_t i = 0; i < id_list_.size(); ++i) {
        target_positions_[i] = position_list[i];
        // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", id_list_[i], position_list[i]);
    }
    syncWritePositionProfile_.clearParam();
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
    std::vector<int> init_vel_acc = {0, 0};
    dxl_hundler.WriteProfilesAll(init_vel_acc, init_vel_acc);
    dxl_hundler.ActivateAll();

    ros::Subscriber sub_joint_trajectory = nh.subscribe<trajectory_msgs::JointTrajectory>("/pantilt_controller/command", 10, boost::bind(&DynamixelHundler::CallbackCommand, &dxl_hundler, _1));
    ros::Publisher pub_joint_state = nh.advertise<control_msgs::JointTrajectoryControllerState>("/pantilt_controller/state", 10);

    ros::Rate rate(50);
    while (ros::ok()) {
        if (dxl_hundler.ReadPositionsAll()) {
            control_msgs::JointTrajectoryControllerState joint_state_msg;
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.joint_names = joint_list;
            for (auto i = 0; i < dxl_hundler.id_list().size(); i++) {
                joint_state_msg.desired.positions.push_back(
                    (dxl_hundler.target_positions()[i] - 4096/2) * (2 * M_PI) / 4096);
                joint_state_msg.actual.positions.push_back(
                    (dxl_hundler.now_positions()[i]  - 4096/2) * (2 * M_PI) / 4096);
                joint_state_msg.error.positions.push_back(
                    joint_state_msg.desired.positions[i] - joint_state_msg.actual.positions[i]);
            }
            pub_joint_state.publish(joint_state_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
