#include <PIDcon.h>

const float T_s = 0.1f; // 100 ms
float r = 0.0f; // setpoint
float y = 0.0f; // output
float u = 0.0f; // control input

// reference parameters
float t = 0.0f; // time
uint8_t r_high = 50;
uint8_t r_low  = 0;

// plant parameters
float w_n  = 1.0f; // natural frequency (rad/s)
float zeta = 0.7f; // damping ratio
float v = 0.0f;

// PID parameters
float k_p = 20.0f;
float k_i = 10.0f;
float k_d = 2.0f;


// PID
PID pid(k_p, k_i, k_d, T_s); // default
// PID pid(k_p, k_i, k_d, T_s, 20.0, TUSTIN_METHOD); // explicit

void reference(){
    t += T_s;

    float T = 20.0f;
    float D = 0.5f;
    float t_on = fmod(t, T);
    if (t_on <D*T) r = r_high;
    else           r = r_low;
}

void setup() {
    Serial.begin(115200);
    // pid.set_anti_windup(true); // activate anti-windup
}

void loop() {
    static uint32_t previous = 0; 

    if (millis() -previous >= T_s*1000){
        previous += T_s*1000;

        // square-wave reference
        reference();

        // PID control signal
        u = pid.compute(r, y);

        // second-order plant simulated output
        v += T_s*(w_n*w_n*(u -y) -2*zeta*w_n*v);
        y += T_s*v;

        // Serial Plotter: r, y, u
        Serial.print(r, 2);
        Serial.print(" ");
        Serial.print(y, 2);
        Serial.print(" ");
        Serial.println(u, 2);
    }
}