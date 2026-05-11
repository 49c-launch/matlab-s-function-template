# matlab-s-function-template

A MATLAB/Simulink C MEX S-Function implementing dual-loop PI control with built-in center-aligned PWM generation for a buck converter.

## Architecture

```
ctrl.c          - S-Function: PI control + triangle-carrier PWM + dead time
pi_ctrl.h       - Reusable PI controller module (header-only)
mexfile.m       - Build script
buck.slx        - Simulink model
```

## Features

- Dual-loop PI control (voltage outer loop + current inner loop)
- Positional-form PI with forward Euler integration
- Clamping anti-windup (integrator frozen when output saturates)
- Center-aligned (triangle carrier) PWM generation
- Complementary PWMA/PWMB outputs with adjustable dead time
- PI parameters (Kp, Ki) fed via input ports for online tuning

## Interface

### Input Ports

| Port | Signal   | Description                        |
|------|----------|------------------------------------|
| 0    | Vout     | Measured output voltage            |
| 1    | IL       | Measured inductor current          |
| 2    | Vref     | Voltage reference                  |
| 3    | Kp_u     | Voltage loop proportional gain     |
| 4    | Ki_u     | Voltage loop integral gain         |
| 5    | Kp_i     | Current loop proportional gain     |
| 6    | Ki_i     | Current loop integral gain         |
| 7    | deadtime | Dead time in seconds (e.g. 1e-6)   |

### Output Ports

| Port | Signal | Description                              |
|------|--------|------------------------------------------|
| 0    | PWMA   | High-side gate drive (0 or 1)            |
| 1    | PWMB   | Low-side gate drive (complementary, 0/1) |

## Timing

| Parameter       | Value   |
|-----------------|---------|
| Sample rate     | 50 MHz  |
| PWM frequency   | 20 kHz  |
| Steps per PWM   | 2500    |
| PI update rate  | 20 kHz  |

The sample rate and PWM frequency are defined as macros (`SAMPLE_FREQ`, `PWM_FREQ`) in `ctrl.c`. Changing `PWM_FREQ` automatically adjusts the carrier period.

## PI Module (`pi_ctrl.h`)

Header-only, struct-based PI controller. Can be reused independently of the S-Function.

```c
PI_Ctrl pi;
PI_Init(&pi, out_min, out_max);
real_T output = PI_Update(&pi, Kp, Ki, error);
```

- `Kp`/`Ki` passed per call (supports runtime tuning)
- Anti-windup: integrator reverted when output clips

## Build

In MATLAB:

```matlab
mex ctrl.c
```

No additional source files needed — `pi_ctrl.h` is included as a header.

## Simulink Setup

1. Add an S-Function block, set function name to `ctrl`
2. Wire 8 input signals and connect 2 output ports
3. Set the Simulink solver fixed-step size to `1/50e6` (matching `SAMPLE_FREQ`)