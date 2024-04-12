#include "turtlebot3_base/motor_driver.h"
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define DEVICE_NAME "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0" 

namespace turtlebot3_base {
    void MotorDriver::Setup(int baudrate, int protocol, int left_motor_id, int right_motor_id) {
        int dxl_comm_result_open_port = COMM_TX_FAIL;
        int dxl_comm_result_set_baud = COMM_TX_FAIL;
        int dxl_comm_result_set_control_l = COMM_TX_FAIL;
        int dxl_comm_result_set_control_r = COMM_TX_FAIL;

        portHandler = PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = PacketHandler::getPacketHandler(protocol);

        // Open Serial Port
        dxl_comm_result_open_port = portHandler->openPort();

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result_set_baud = portHandler->setBaudRate(baudrate);

        // Use Velocity Control Mode
        dxl_comm_result_set_control_l = packetHandler->write1ByteTxRx(portHandler, left_motor_id, ADDR_OPERATING_MODE, 1);

        // Use Velocity Control Mode
        dxl_comm_result_set_control_r = packetHandler->write1ByteTxRx(portHandler, right_motor_id, ADDR_OPERATING_MODE, 1);

        // Enable torque left motor
        packetHandler->write1ByteTxRx(portHandler, left_motor_id, ADDR_TORQUE_ENABLE, 1);

        // Enable torque right motor
        packetHandler->write1ByteTxRx(portHandler, right_motor_id, ADDR_TORQUE_ENABLE, 1);
        
    }

    bool MotorDriver::is_connected(int left_motor_id, int right_motor_id) const {
        int dxl_comm_result = COMM_TX_FAIL;
        int8_t moving = 0;

        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, left_motor_id, 122, reinterpret_cast<uint8_t *>(&moving));
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, right_motor_id, 122, reinterpret_cast<uint8_t *>(&moving));

        if (dxl_comm_result == false) {
            return true;
        } else {
            return false;
        }
    }

    std::array<int32_t, 2> MotorDriver::ReadMotors(int left_motor_id, int right_motor_id) {
        std::array<int32_t, 2> positions;
        int32_t left_position;
        int32_t right_position;

        packetHandler->read4ByteTxRx(portHandler, left_motor_id, 132, reinterpret_cast<uint32_t *>(&left_position));
        packetHandler->read4ByteTxRx(portHandler, right_motor_id, 132, reinterpret_cast<uint32_t *>(&right_position));

        positions[0] = left_position;
        positions[1] = right_position;

        return positions;
    }

    void MotorDriver::SetMotorValues(int val_1, int val_2, int left_motor_id, int right_motor_id) {
        packetHandler->write4ByteTxRx(portHandler, left_motor_id, 104, val_1);
        packetHandler->write4ByteTxRx(portHandler, right_motor_id, 104, val_2);
    }
}
