#pragma once

#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "turtlebot3_base/motor_driver.hpp"
#include "turtlebot3_base/wheel.h"

namespace turtlebot3_base {
    /// @brief Hardware interface for turtlebot3 robot.
    /// This class is a hardware interface implementation for the turtlebot3 robot. It is responsible for
    /// abstracting away the specifics of the hardware and exposing interfaces that are easy to work with.
    class DiffDriveTurtlebot3 : public hardware_interface::SystemInterface {
        public:
            /// @brief Default constructor for the DiffDriveTurtlebot3 class
            DiffDriveTurtlebot3() = default;
            
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
            
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        
        private:
            const std::string kDiffDriveTurtlebot3{"DiffDriveTurtlebot3"};
            const std::string kLeftWheelNameParam{"left_wheel_name"};
            const std::string kRightWheelNameParam{"right_wheel_name"};
            const std::string kSerialDeviceParam{"serial_device"};
            const std::string kBaudRateParam{"baud_rate"};
            const std::string kProtocol{"protocol"};
            const std::string kMotorId{"id"};
            const std::string kRpmToMs{"RPM_TO_MS"};
            const std::string kTicksToRad{"TICK_TO_RAD"};

            struct Config {
                // Name and ids of the left and right wheels.
                std::string left_wheel_name = "left_wheel";
                std::string right_wheel_name = "right_wheel";
                int left_wheel_id = 0;
                int right_wheel_id = 1;
                // Motors parameters
                double left_wheel_RPM_TO_MS = 0.229 * 0.0034557519189487725;
                double left_wheel_TICK_TO_RAD = 0.001533981;
                double right_wheel_RPM_TO_MS = 0.229 * 0.0034557519189487725;
                double right_wheel_TICK_TO_RAD = 0.001533981;
                // Communication parameters.
                std::string serial_device = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0";
                int protocol = 2.0;
                int baud_rate = 57600;
            };

            // Configuration parameters.
            Config config_;
            // Communication with the firmware in charge of controlling the motors.
            MotorDriver motor_driver_;
            // Left wheel of the robot.
            Wheel left_wheel_;
            // Right wheel of the robot.
            Wheel right_wheel_;
            // Logger.
            rclcpp::Logger logger_{rclcpp::get_logger(kDiffDriveTurtlebot3)};
    };
} // namespace turtlebot3_base