#include "PIDcon.h"


PID::PID(float k_p, float k_i, float k_d, float T_s, float p, Discretization discretization)
{
    // set PID parameters
    set_tuning(k_p, k_i, k_d);

    // set pole for HF filter
    set_filter(p);

    // set default values
    u_max =  255.0;
    u_min = -255.0;
    anti_windup = false;

    // set T_s and discretization method
    this->T_s = T_s;
    this->discretization = discretization;

    reset();
}

// compute
float PID::compute(float r, float y)
{
    // calculates error
    float e = r -y;

    // proportional control action
    float P = k_p*e;

    switch (discretization)
    {
        case EULER_METHOD:
            // integral action
            I = I_prev +k_i*T_s*e; 

            // derivative action
            D = k_d*(e -e_prev)/T_s;
            break;

        case TUSTIN_METHOD:
            // integral control action
            I = I_prev +k_i*(e +e_prev)*T_s/2;

            // derivative control action
            D = D_prev*(2 -p*T_s)/(2 +p*T_s) +2*p*k_d/(2 +p*T_s)*(e -e_prev);
            break;

        default:
            reset();
            return 0.0f;
    }
    
    // control signal
    float u = P +I +D;

    // anti-windup: conditional integration
    if (anti_windup && ((u > u_max && I > I_prev) || (u < u_min && I < I_prev)))
    {
        I = I_prev;
        u = P +I +D;
    }

    // saturation
    float u_sat = u;
    if (u > u_max) u_sat = u_max;
    if (u < u_min) u_sat = u_min;
    
    // update previous
    e_prev = e;
    I_prev = I;
    D_prev = D;
    return u_sat;
}

// reset
void PID::reset()
{
    I = 0.0f;
    D = 0.0f;
    e_prev = 0.0f;
    I_prev = 0.0f;
    D_prev = 0.0f;
}

// set methods
void PID::set_tuning(float k_p, float k_i, float k_d)
{   
    if (k_p < 0 || k_i < 0 || k_d < 0) return;
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
}
void PID::set_filter(float p) { 
    if (p < 0.0f) return;
    this->p = p;
} 
void PID::set_limit(float u_min, float u_max)
{
    if (u_min >= u_max) return;
    this->u_min = u_min;
    this->u_max = u_max;
}
void PID::set_anti_windup(bool enable)
{
    anti_windup = enable;
}

// get methods
float PID::get_kp() { return k_p; }
float PID::get_ki() { return k_i; }
float PID::get_kd() { return k_d; }