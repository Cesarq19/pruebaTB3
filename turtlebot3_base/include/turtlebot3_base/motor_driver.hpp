#pragma once

#include <array>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64

namespace turtlebot3_base {
    /// @brief Class to handle TTL communication with the dynamixels motor
    /// It is used to send commands to the motor and read values of position
    class MotorDriver
    {
    private:
        PortHandler *portHandler;
        PacketHandler *packetHandler;
    public:
        /// @brief Default constructor
        MotorDriver() = default;

        /// @brief Type to store the encoder values.
        using Encoders = std::array<int32_t, 2>;

        /// @brief Check if the motors is OK
        /// @return True if the motors response with COMM_SUCCESS, false otherwise
        bool is_connected(int left_motor_id, int right_motor_id) const;

        /// @brief Initialize the motors 
        /// @param serial_port Path to the serial device(eg. /dev/ttyACM0)
        /// @param baudrate Baud rate of the serial connection(eg. 57600)
        /// @param protocol Protocol of communciation of the motors (eg. 1.0 or 2.0)
        /// @param left_motor_id ID of dynamixel left motor
        /// @param right_motor_id ID of dyanamixel right motor
        void Setup(const std::string& serial_port, int baudrate, int protocol, int left_motor_id, int right_motor_id);

        Encoders ReadMotors(int left_motor_id, int right_motor_id);

        /// @brief Set the motor values.
        ///        The unit of the values is in encoder ticks per revolution.
        /// @param val_1 Value for the first motor.
        /// @param val_2 Value for the second motor.
        /// @param left_motor_id ID of dynamixel left motor
        /// @param right_motor_id ID of dyanamixel right motor
        void SetMotorValues(int val_1, int val_2, int left_motor_id, int right_motor_id);
    };
} // namespace turtlebot3_base