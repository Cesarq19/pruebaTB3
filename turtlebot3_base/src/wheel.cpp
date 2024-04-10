#include "turtlebot3_base/wheel.h"

#include <cmath>

namespace turtlebot3_base {
    void Wheel::Setup(const std::string& wheel_name, double ticks_to_rad){
        name_ = wheel_name;
        rads_per_tick_ = ticks_to_rad;
    }

    double Wheel::Angle() { return enc_ * rads_per_tick_; }
} // namespace turtlebot3_base