#include "PIDcon.h"
#include <math.h>


PID::PID(float k_p, float k_i, float k_d, float T_s, float p, Discretization discretization, float w_w)
{
    // set T_s and discretization method
    this->T_s = (T_s > 0.0f) ? T_s : 0.01f;
    this->discretization = discretization;

    // prewarping coefficient
    c = 2.0f/this->T_s;
    if (discretization == TUSTIN_PREWARP_METHOD) c = w_w/tanf(w_w*this->T_s/2.0f);

    // set PID parameters
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;

    // set pole for HF filter
    this->p = p;

    // set default values
    u_max =  255.0f;
    u_min = -255.0f;
    anti_windup = false;

    update_coefficients();
    
    // reset method
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
            I = I_prev +b_i*e; 

            // derivative action
            D = a_d*D_prev +b_d*(e -e_prev);

            break;

        case TUSTIN_METHOD:            
            // integral control action
            I = I_prev +b_i*(e +e_prev);

            // derivative control action
            D = a_d*D_prev +b_d*(e -e_prev);
            
            break;

        case TUSTIN_PREWARP_METHOD:
            // integral control action
            I = I_prev +b_i*(e +e_prev);

            // derivative control action
            D = a_d*D_prev +b_d*(e -e_prev);

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
    if (u > u_max) u = u_max;
    if (u < u_min) u = u_min;
    
    // update previous
    e_prev = e;
    I_prev = I;
    D_prev = D;
    return u;
}

void PID::update_coefficients()
{
    switch (discretization)
    {
        case EULER_METHOD:
            b_i = k_i*T_s;
            a_d = 1.0f/(1.0f +p*T_s);
            b_d = k_d*p*a_d;
            break;

        case TUSTIN_METHOD:
            b_i = k_i*T_s/2.0f;
            a_d = (2.0f -p*T_s)/(2.0f +p*T_s);
            b_d = 2.0f*p*k_d/(2.0f +p*T_s);
            break;

        case TUSTIN_PREWARP_METHOD:
            b_i = k_i/c;
            a_d = (c -p)/(c +p);
            b_d = p*k_d*c/(c +p);
            break;
    }
}

// reset
void PID::reset()
{
    I      = 0.0f;
    D      = 0.0f;
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
    update_coefficients();
}
void PID::set_filter(float p) { 
    if (p < 0.0f) return;
    this->p = p;
    update_coefficients();
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