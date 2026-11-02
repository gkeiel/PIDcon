#ifndef PIDcon_h
#define PIDcon_h


enum Discretization { EULER_METHOD, TUSTIN_METHOD  };

class PID {
    public:
        // constructor
        PID(float k_p, float k_i, float k_d, float T_s, float p = 100.0f, Discretization discretization = TUSTIN_METHOD);

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

        float k_p, k_i, k_d, T_s, p;
        float u_min, u_max;
        float I, D, e_prev, I_prev, D_prev;
        bool anti_windup;
        Discretization discretization;
};
#endif