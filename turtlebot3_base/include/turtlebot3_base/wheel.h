#pragma once

#include <string>

namespace turtlebot3_base {
    class Wheel {
        public:
            /// @brief Default constructor for the Wheel class
            Wheel() = default;

            /// @brief Setup the wheel.
            /// @param wheel_name name of the wheel.
            /// @param ticks_to_rad number of encoder ticks in radians.
            void Setup(const std::string& wheel_name, double ticks_to_rad);

            /// @brief Calculate the angle of the wheel.
            /// @return The angle of the wheel in radians.
            double Angle();

            std::string name_ = "";
            unsigned int enc_ = 0;
            double cmd_ = 0;
            double pos_ = 0;
            double vel_ = 0;
            double eff_ = 0;
            double vel_set_pt_ = 0;
            double rads_per_tick_ = 0;
    };
} //namespace turtlebot3