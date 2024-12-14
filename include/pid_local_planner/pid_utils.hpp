#ifndef PID_UTILS_HPP_
#define PID_UTILS_HPP_

namespace pid_local_planner
{
    struct PIDGain
    {
        float p_gain, i_gain, d_gain;

        PIDGain(float p=1.0, float i=1.0, float d=0.1):p_gain(p),i_gain(i),d_gain(d){}
    };
    

    class PIDController
    {
        public:
        PIDController();

        void setConfig(PIDGain gain, float max, float min);

        float calc(float target, float actual, float delta_time);

        void reset();

        private:
        float limit(float value)
        {
            if(value > out_max_)
            {
                return out_max_;
            }
            else if(value < out_min_)
            {
                return out_min_;
            }
            else
            {
                return value;
            }
        }

        PIDGain gain_;
        float out_max_, out_min_;
        float prev_prop_, low_path_filtered_, integral_;
    };
}

#endif