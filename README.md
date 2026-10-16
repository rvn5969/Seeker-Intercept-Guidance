**# Seeker-Based Autonomous Intercept Guidance and HIL Validation**

C++/MATLAB/Python project for studying planar autonomous guidance under noisy seeker measurements, measurement latency, target maneuvers, actuator limits, and real-time execution constraints.

The project compares a proportional-navigation (PN) baseline against constrained receding-horizon predictive guidance. Both controllers use the same EKF-estimated relative state. neither receives target truth.

**## Architecture**

```text
target / interceptor truth
        ↓
noisy delayed range + bearing seeker
        ↓
relative-state EKF
        ↓
current-state propagation
        ↓
PN or predictive guidance
        ↓
lateral-acceleration command
        ↓
acceleration / slew limits + actuator lag
        ↓
interceptor dynamics
```

Main components:

```text
config/experiment.json        central experiment configuration
cpp/                          EKF, PN, predictive guidance, tests, HIL
matlab/                       Simulink model generation and demo
python/analyze_results.py     Monte Carlo analysis and figures
docs/                         theory and validation notes
results/                      processed results and figures
```

For the mathematical formulation, see [`theorynote.md`](theorynote.md).

**## Build and test**

From the repository root in Ubuntu/WSL:

```bash
cmake --preset release
cmake --build --preset release
ctest --test-dir build/release --output-on-failure
```

**## Monte Carlo validation**

Run the C++ benchmark:

```bash
./build/release/cpp/validation_benchmark
```

The benchmark reads `config/experiment.json` and performs paired PN vs predictive Monte Carlo trials using the same engagement geometry, sensor realization, dropout pattern, and latency for both controllers.

It reports:

- hit rate
- closest-approach statistics
- control effort
- estimator RMSE
- predictive-guidance execution timing

Then generate processed statistics and figures with:

```bash
source ~/.venvs/seeker-guidance/bin/activate
python python/analyze_results.py
```

**### Representative robustness result**

One 5,000-run robustness batch with increased sensor noise, latency, dropout, and target maneuver uncertainty produced:

| Metric | PN | Predictive |
|---|---:|---:|
| Hits | 4719 / 5000 | 4755 / 5000 |
| Hit rate | 94.38% | 95.10% |
| Median closest approach | 0.709 m | 0.703 m |
| p95 closest approach | 5.902 m | 4.927 m |

In that batch, predictive guidance reduced failed engagements by approximately **12.8%** and reduced p95 closest approach by approximately **16.5%**, at approximately **6.4% higher median control effort**.

Predictive-guidance execution time remained below the 10 ms / 100 Hz guidance budget:

```text
median:  88.5 us
p99:    181.2 us
max:    477.8 us
```

These results represent one randomized robustness study.

**## MATLAB / Simulink**

The Simulink model uses the same C++ guidance stack through a compiled S-function.

**### First-time setup**

Configure a supported MATLAB C++ compiler:

```matlab
mex -setup C++
```

**### Run the complete demo**

From the repository `matlab/` directory:

```matlab
run_demo
```

`run_demo` automatically:

- builds the C++ S-function if needed
- builds the Simulink model if needed
- runs the same engagement with predictive guidance and PN
- reports closest approach and hit/miss results
- generates PN-vs-predictive comparison figures

Figures are saved to:

```text
results/figures/
```

including trajectory, range, commanded-acceleration, and actuator-response comparisons.

If physical or sensor parameters in `config/experiment.json` are changed, regenerate the model with:

```matlab
build_models
run_demo
```

**## Guidance methods**

**### Proportional navigation**

PN uses

$$
a_{\mathrm{cmd}} = N V_c \dot{\lambda}
$$

where $V_c$ is closing velocity and $\dot{\lambda}$ is line-of-sight angular rate.

**### Predictive guidance**

Predictive guidance solves a short receding-horizon problem using a cost of the form

$$
J
=
w_z\,\mathrm{ZEM}^2
+
w_u \sum_k u_k^2
+
w_{\Delta u} \sum_k (u_k-u_{k-1})^2
$$

subject to acceleration, slew-rate, and actuator constraints. Only the first optimized command is applied before the problem is solved again at the next guidance update.

**## HIL interface**

A compact UDP HIL controller is also included:

```bash
./build/release/cpp/hil_controller
```

The HIL path uses the same C++ guidance stack as the standalone benchmark and Simulink integration.

**## Scope and limitations**

This is a simulation and controls-validation project. The current model uses:

- planar 2-D kinematics
- constant vehicle speeds
- range/bearing seeker measurements
- simplified target maneuvers
- first-order actuator dynamics
- fixed per-engagement measurement latency
- software/Simulink HIL rather than physical flight hardware