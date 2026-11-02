# PIDcon

A lightweight **discrete-time proportional-integral-derivative (PID) control library for Arduino**.

The library implements PID controllers using different discretization methods, including **Euler and Tustin approximations**. It also provides a **first-order filter for the derivative action** and **conditional integration for anti-windup** protection.

The library is intended as a practical reference for implementing, testing, and applying PID control algorithms in embedded systems, particularly on **8-bit AVR microcontrollers** such as the ATmega328P. Furthermore, the sampling period is configured by the user and must match the actual interval between consecutive calls to the controller.


## Features

- **PID control**: Implements discrete-time PID action.
- **Discretization methods**: Supports Euler and Tustin approximations.
- **Derivative filtering**: Includes a first-order high-frequency filter to attenuate noise in the derivative action.
- **Anti-windup**: Implements conditional integration to prevent excessive accumulation of the integral action during saturation.
- **Parameter configuration**: Provides methods to adjust PID gains and additional parameters.


## Installation

Download or clone this repository and place the PIDcon folder inside the Arduino libraries directory. Alternatively, install the library through the Arduino IDE using **Sketch → Include Library → Add .ZIP Library**.

## Basic usage

Create a PID controller by specifying its proportional, integral, and derivative gains, together with the sampling period:

```cpp
#include <PIDcon.h>

float k_p = 100.0f;  
float k_i = 5.0f;  
float k_d = 1.0f;  
float T_s = 0.01f;  

PID pid(k_p, k_i, k_d, T_s);
```

Although the constructor uses the Tustin method by default, with a derivative filter parameter of 100.0f, the controller can also be configured explicitly:

```cpp
PID pid(k_p, k_i, k_d, T_s, 100.0f, TUSTIN_METHOD);
```

To calculate the control signal, call compute() with the reference and measured output:
```cpp
float u = pid.compute(r, y);
```

## Configuration

The controller provides the following methods:

| Method | Description |
|---|---|
| `set_tuning(k_p, k_i, k_d)` | Sets the PID gains. |
| `set_filter(p)` | Sets the derivative filter parameter. |
| `set_limit(u_min, u_max)` | Sets the output saturation limits. |
| `set_anti_windup(enable)` | Enables or disables conditional integration. |
| `reset()` | Resets the controller's internal states. |
| `get_kp()` | Returns the proportional gain. |
| `get_ki()` | Returns the integral gain. |
| `get_kd()` | Returns the derivative gain. |


## Notes

Improvements and additional control strategies may be added in future versions.