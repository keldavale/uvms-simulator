#ifndef ROS2_CONTROL_BLUE_REACH_5__PACKET_ID_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__PACKET_ID_HPP_

namespace alpha::driver
{

/**
 * @brief A unique identifier used to determine how to interpret the packet data.
 */
enum class PacketId : unsigned char
{
PacketID_MODE = 0x01,
    PacketID_VELOCITY = 0x02,
    PacketID_POSITION = 0x03,
    PacketID_OPENLOOP = 0x04,
    PacketID_CURRENT = 0x05,
    PacketID_RELATIVE_POSITION = 0x0E,
    PacketID_AUTO_LIMIT_CURRENT_DEMAND = 0x0F,
    PacketID_SUPPLYVOLTAGE = 0x90,
    PacketID_TEMPERATURE = 0x66,
    PacketID_REQUEST_PACKET = 0x60,
    PacketID_SERIAL_NUMBER = 0x61,
    PacketID_MODEL_NUMBER = 0x62,
    PacketID_VERSION = 0x63,
    PacketID_DEVICE_ID = 0x64,
    PacketID_INTERNAL_HUMIDITY = 0x65,
    PacketID_INTERNAL_TEMPERATURE = 0x66,
    PacketID_DEVICE_TYPE = 0x67,
    PacketID_HARDWARE_STATUS = 0x68,
    PacketID_RUN_TIME = 0x69,
    PacketID_STATE_ESTIMATOR_STATUS = 0x71,
    PacketID_COMS_PROTOCOL = 0x80,
    PacketID_HEARTBEAT_FREQUENCY_SET = 0x92,
    PacketID_HEARTBEAT_SET = 0x91,
    PacketID_SAVE = 0x50,
    PacketID_LOAD = 0x51,
    PacketID_SET_DEFAULTS = 0x52,
    PacketID_FORMAT = 0x53,
    PacketID_CHANGE_PAGE = 0x54,
    PacketID_CURRENT_LIMIT = 0x12,
    PacketID_VELOCITY_LIMIT = 0x11,
    PacketID_POSITION_LIMIT = 0x10,
    PacketID_POSITION_GAIN = 0x13,
    PacketID_VELOCITY_GAIN = 0x14,
    PacketID_CURRENT_GAIN = 0x15,
    PacketID_POSITION_PARAMETERS = 0x20,
    PacketID_VELOCITY_PARAMETERS = 0x21,
    PacketID_CURRENT_PARAMETERS = 0x22,
    PacketID_INPUT_VOLTAGE_PARAMETERS = 0x23,
    PacketID_MOTOR_PARAMETERS = 0x30,
    PacketID_MAX_ACCELERATION = 0x40,
    PacketID_CURRENT_HOLD_THRESHOLD = 0x41,
    PacketID_COMPLIANCE_GAIN = 0x42,
    PacketID_COMPLIANCE_PARAMETERS = 0x44,
    PacketID_ELECTRICAL_VERSION = 0x6A,
    PacketID_MECHANICAL_VERSION = 0x6B,
    PacketID_SOFTWARE_VERSION = 0x6C,
    PacketID_BOOTLOADER_STM = 0xFE,
    PacketID_BOOTLOADER = 0xFF,
    PacketID_KM_CONFIGURATION = 0xA0,
    PacketID_KM_END_POS = 0xA1,
    PacketID_KM_END_VEL = 0xA2,
    PacketID_KM_BOX_OBSTACLE_00 = 0xA3,
    PacketID_KM_BOX_OBSTACLE_01 = 0xA4,
    PacketID_KM_BOX_OBSTACLE_02 = 0xA5,
    PacketID_KM_BOX_OBSTACLE_03 = 0xA6,
    PacketID_KM_BOX_OBSTACLE_04 = 0xA7,
    PacketID_KM_BOX_OBSTACLE_05 = 0xA8,
    PacketID_KM_CYLINDER_OBSTACLE_00 = 0xA9,
    PacketID_KM_CYLINDER_OBSTACLE_01 = 0xAA,
    PacketID_KM_CYLINDER_OBSTACLE_02 = 0xAB,
    PacketID_KM_CYLINDER_OBSTACLE_03 = 0xAC,
    PacketID_KM_CYLINDER_OBSTACLE_04 = 0xAD,
    PacketID_KM_CYLINDER_OBSTACLE_05 = 0xAE,
    PacketID_KM_FLOAT_PARAMETERS = 0xB0,
    PacketID_KM_JOINT_STATE = 0xB2,
    PacketID_KM_JOINT_STATE_REQUEST = 0xB3,
    PacketID_KM_DH_PARAMETERS_0 = 0xB8,
    PacketID_KM_DH_PARAMETERS_1 = 0xB9,
    PacketID_KM_DH_PARAMETERS_2 = 0xBA,
    PacketID_KM_DH_PARAMETERS_3 = 0xBB,
    PacketID_KM_DH_PARAMETERS_4 = 0xBC,
    PacketID_KM_DH_PARAMETERS_5 = 0xBD,
    PacketID_KM_DH_PARAMETERS_6 = 0xBE,
    PacketID_KM_DH_PARAMETERS_7 = 0xBF,
    PacketID_KM_POS_LIMIT_TRANSLATE = 0xC0,
    PacketID_KM_VEL_LIMIT_TRANSLATE = 0xC1,
    PacketID_KM_POS_LIMIT_YAW = 0xC2,
    PacketID_KM_POS_LIMIT_PITCH = 0xC3,
    PacketID_KM_POS_LIMIT_ROLL = 0xC4,
    PacketID_KM_VEL_LIMIT_ROTATE = 0xC5,
    PacketID_KM_POS_GAINS_TRANSLATE = 0xC6,
    PacketID_KM_VEL_GAINS_TRANSLATE = 0xC7,
    PacketID_KM_POS_GAINS_ROTATE = 0xC8,
    PacketID_KM_VEL_GAINS_ROTATE = 0xC9,
    PacketID_KM_JOINT_POS_0 = 0xD0,
    PacketID_KM_JOINT_POS_1 = 0xD1,
    PacketID_KM_JOINT_POS_2 = 0xD2,
    PacketID_KM_JOINT_POS_3 = 0xD3,
    PacketID_KM_JOINT_POS_4 = 0xD4,
    PacketID_KM_JOINT_POS_5 = 0xD5,
    PacketID_KM_JOINT_POS_6 = 0xD6,
    PacketID_KM_JOINT_POS_7 = 0xD7,
    PacketID_KM_COLLISION_FLAG = 0xAF,
    PacketID_KM_COLLISION_COORDS = 0xB1,
    PacketID_VELOCITY_DEMAND_INNER = 0x07,
    PacketID_POSITION_DEMAND_INNER = 0x08,
    PacketID_CURRENT_DEMAND_DIRECT = 0x09,
    PacketID_ICMU_INNER_PARAMETERS = 0x6F,
    PacketID_VELOCITY_LIMIT_INNER = 0x16,
    PacketID_POSITION_GAINS_INNER = 0x17,
    PacketID_VELOCITY_GAINS_INNER = 0x18,
    PacketID_CURRENT_GAINS_DIRECT = 0x19,
    PacketID_VELOCITY_INNER_PARAMETERS = 0x24,
    PacketID_ENCODER = 0xE0,
    PacketID_ENCODER_PARAMETERS = 0xE1,
    PacketID_TEST_PACKET = 0xE2,
    PacketID_MODE_SETTINGS = 0x43,
    PacketID_RESET = 0xFD
};

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__PACKET_ID_HPP_