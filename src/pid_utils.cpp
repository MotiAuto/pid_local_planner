#include "pid_local_planner/pid_utils.hpp"

namespace pid_local_planner
{
    PIDController::PIDController()
    {
        integral_ = 0.0;
        prev_prop_ = 0.0;
        low_path_filtered_ = 0.0;
    }

    void PIDController::setConfig(PIDGain gain, float max, float min)
    {
        gain_ = gain;
        out_max_ = max;
        out_min_ = min;
    }

    float PIDController::calc(float target, float actual, float delta_time)
    {
        const auto prop = target - actual;
        integral_ += prop * delta_time; 
        const auto deriv = (prop - prev_prop_) / delta_time;
        prev_prop_ = prop;

        low_path_filtered_ = (deriv - low_path_filtered_) / 8.0;
        const auto du = gain_.p_gain * prop + gain_.i_gain * integral_ + gain_.d_gain * low_path_filtered_;

        return limit(du);
    }

    void PIDController::reset()
    {
        integral_ = 0.0;
        prev_prop_ = 0.0;
        low_path_filtered_ = 0.0;
    }
}