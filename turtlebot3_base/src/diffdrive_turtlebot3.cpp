#include "turtlebot3_base/diffdrive_turtlebot3.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#define DEVICE_NAME "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0" 

namespace turtlebot3_base {

    hardware_interface::CallbackReturn DiffDriveTurtlebot3::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }


        RCLCPP_INFO(logger_, "On init...");
        config_.left_wheel_name = info_.hardware_parameters[kLeftWheelNameParam];
        RCLCPP_INFO(logger_, (kLeftWheelNameParam + static_cast<std::string>(": ") + config_.left_wheel_name).c_str());
        config_.right_wheel_name = info_.hardware_parameters[kRightWheelNameParam];
        RCLCPP_INFO(logger_, (kRightWheelNameParam + static_cast<std::string>(": ") + config_.right_wheel_name).c_str());
        config_.serial_device = info_.hardware_parameters[kSerialDeviceParam];
        RCLCPP_INFO(logger_, (kSerialDeviceParam + static_cast<std::string>(": ") + config_.serial_device).c_str());
        config_.baud_rate = std::stoi(info_.hardware_parameters[kBaudRateParam]);
        RCLCPP_INFO(logger_,
                    (kBaudRateParam + static_cast<std::string>(": ") + info_.hardware_parameters[kBaudRateParam]).c_str());
        config_.protocol = std::stoi(info_.hardware_parameters[kProtocol]);
        RCLCPP_INFO(logger_,
                    (kProtocol + static_cast<std::string>(": ") + info_.hardware_parameters[kProtocol]).c_str());
        
        for (const hardware_interface::ComponentInfo& joint : info.joints) {
            // DiffDriveTurtlebot3 has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                            joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
                            joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        config_.left_wheel_id = std::stoi(info.joints[0].parameters.at("id"));
        config_.right_wheel_id = std::stoi(info.joints[1].parameters.at("id"));

        config_.left_wheel_RPM_TO_MS = std::stod(info.joints[0].parameters.at("RPM_TO_MS"));
        config_.right_wheel_RPM_TO_MS = std::stod(info.joints[1].parameters.at("RPM_TO_MS"));

        config_.left_wheel_TICK_TO_RAD = std::stod(info.joints[0].parameters.at("TICK_TO_RAD"));
        config_.right_wheel_TICK_TO_RAD = std::stod(info.joints[1].parameters.at("TICK_TO_RAD"));

        // Set up the wheels
        left_wheel_.Setup(config_.left_wheel_name, config_.left_wheel_TICK_TO_RAD);
        right_wheel_.Setup(config_.right_wheel_name, config_.right_wheel_TICK_TO_RAD);

        RCLCPP_INFO(logger_, "Finished On init.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveTurtlebot3::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(logger_, "On configure...");
        // Set up communication with motor driver controller.
        motor_driver_.Setup(config_.baud_rate, config_.protocol, config_.left_wheel_id, config_.right_wheel_id);

        RCLCPP_INFO(logger_, "Finished Configuration");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffDriveTurtlebot3::export_state_interfaces() {
        // We need to set up a position and a velocity interface for each wheel
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel_));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_POSITION, &left_wheel_.pos_));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel_));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_POSITION, &right_wheel_.pos_));

        return state_interfaces;
    }    

    std::vector<hardware_interface::CommandInterface> DiffDriveTurtlebot3::export_command_interfaces() {
        // We need to set up a velocity command interface for each wheel
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_));
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_));
        
        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffDriveTurtlebot3::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
        RCLCPP_INFO(logger_, "On activate...");
        RCLCPP_INFO(logger_, "Finished Activation");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveTurtlebot3::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
        RCLCPP_INFO(logger_, "On deactivate...");
        RCLCPP_INFO(logger_, "Finished Deactivation");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffDriveTurtlebot3::read(const rclcpp::Time& /* time */, const rclcpp::Duration& period) {
        const double delta_secs = period.seconds();

        if (!motor_driver_.is_connected(config_.left_wheel_id, config_.right_wheel_id)) {
            RCLCPP_ERROR(logger_, "Motor driver is not connected.");
            return hardware_interface::return_type::ERROR;
        }

        const std::array<int32_t, 2> encoders = motor_driver_.ReadMotors(config_.left_wheel_id, config_.right_wheel_id);

        left_wheel_.enc_ = encoders[0];
        right_wheel_.enc_ = encoders[1];

        const double left_pos_prev = left_wheel_.pos_;
        left_wheel_.pos_ = left_wheel_.Angle();
        left_wheel_.vel_ = (left_wheel_.pos_ - left_pos_prev) / delta_secs;

        const double right_pos_prev = right_wheel_.pos_;
        right_wheel_.pos_ = -right_wheel_.Angle();
        right_wheel_.vel_ = (right_wheel_.pos_ - right_pos_prev) / delta_secs;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffDriveTurtlebot3::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
        if (!motor_driver_.is_connected(config_.left_wheel_id, config_.right_wheel_id)) {
            RCLCPP_ERROR(logger_, "Motor driver is not connected.");
            return hardware_interface::return_type::ERROR;
        }

        // The command is in rad/sec (rps), we need to convert it to ticks/sec (tps)
        // Using the rads per tick(rpt) of the motor information
        // Formula: ticks/sec = rads/sec / rads/tick

        const int left_value_target = static_cast<int>(left_wheel_.cmd_ * 1285.05347);
        const int right_value_target = static_cast<int>(right_wheel_.cmd_ * 1285.05347);
        RCLCPP_INFO(logger_, (left_wheel_.cmd_).c_str());
        RCLCPP_INFO(logger_, (right_wheel_.cmd_).c_str());
        motor_driver_.SetMotorValues(left_value_target, -right_value_target, config_.left_wheel_id, config_.right_wheel_id);

        return hardware_interface::return_type::OK;        
    }
} // namespace turtlebot3_base

PLUGINLIB_EXPORT_CLASS(turtlebot3_base::DiffDriveTurtlebot3, hardware_interface::SystemInterface)
