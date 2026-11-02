#ifndef PIDcon_h
#define PIDcon_h


enum Discretization { EULER_METHOD, TUSTIN_METHOD, TUSTIN_PREWARP_METHOD };

class PID {
    public:
        // constructor
        PID(float k_p, float k_i, float k_d, float T_s, float p = 100.0f, Discretization discretization = TUSTIN_METHOD, float w_w = 0.0f);

        // configuration
        void set_tuning(float k_p, float k_i, float k_d);
        void set_filter(float p);
        void set_limit(float u_min, float u_max);
        void set_anti_windup(bool enable);

        // compute PID control signal
        float compute(float r, float y);

        // reset control signals
        void reset();

        // get methods
        float get_kp();
        float get_ki();
        float get_kd();

    private:

        float k_p, k_i, k_d, T_s, p, c;
        float I, D, e_prev, I_prev, D_prev, u_min, u_max;
        bool anti_windup;
        Discretization discretization;
};
#endif