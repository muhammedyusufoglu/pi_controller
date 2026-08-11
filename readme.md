# Dynamic Gain-Scheduling PI Controller (Verilog)

This repository contains a synthesizable Verilog implementation of a signed
Proportional-Integral (PI) controller with dynamic gain-scheduling and
anti-windup logic. The design uses 12-bit signed reference and feedback inputs
and produces a 24-bit signed control output.

## Key Features

- **Dynamic Gain-Scheduling:** $K_p$ and $K_i$ are selected according to the
  magnitude of the control error.
- **Anti-Windup & Saturation:** The output is limited using configurable
  `MAX_OUTPUT` and `MIN_OUTPUT` parameters, while the integral accumulator is
  corrected during saturation.
- **Signed Arithmetic:** 12-bit signed inputs and 24-bit signed output.
- **Target Frequency:** 125 MHz with an 8 ns clock period.

## PI Control Algorithm

The control error is calculated as:

$$
e[n] = r[n] - y[n]
$$

where $r[n]$ is the reference and $y[n]$ is the feedback signal.

The proportional term is:

$$
P[n] = K_p e[n]
$$

The integral term accumulates the error over time:

$$
I[n] = I[n-1] + K_i e[n]
$$

The controller output is then calculated as:

$$
u[n] = P[n] + I[n]
$$

In the RTL, these operations are implemented as:

```verilog
error       = reference - feedback;
p           = error * kp_constant;
delta_i     = error * ki_constant;
accumulator = accumulator + delta_i;
control_sum = accumulator + p;
```

The proportional term provides an immediate response to the current error,
while the integral term accumulates error over time to reduce steady-state
error.

## Dynamic Gain Scheduling

The controller uses different gain values depending on the error magnitude:

$$
(K_p,K_i)=
\begin{cases}
(128,4), & |e[n]| > 500 \\
(32,1), & |e[n]| \leq 500
\end{cases}
$$

Higher gains are used for large errors to obtain a more aggressive response,
while lower gains are used near the reference to reduce the controller
response.

## Anti-Windup

The controller limits the output to:

$$
U_{\min} \leq u[n] \leq U_{\max}
$$

with default values:

$$
U_{\max}=2^{23}-1
$$

$$
U_{\min}=-2^{23}
$$

When saturation occurs, the integral accumulator is corrected:

$$
I[n] = U_{\max} - P[n]
\qquad \text{if } u[n] > U_{\max}
$$

$$
I[n] = U_{\min} - P[n]
\qquad \text{if } u[n] < U_{\min}
$$

This prevents the integral term from continuing to accumulate while the
output is saturated.

## Hardware Implementation

The design was targeted for an 8 ns clock period:

$$
T_{clk}=8\ ns
$$

corresponding to:

$$
f_{clk}=\frac{1}{T_{clk}}=125\ MHz
$$

The output limits are configurable through Verilog parameters:

```verilog
parameter signed [23:0] MAX_OUTPUT = 24'sh7FFFFF;
parameter signed [23:0] MIN_OUTPUT = 24'sh800000;
```

The design can be synthesized and implemented on FPGA devices using standard
RTL synthesis and static timing analysis.
