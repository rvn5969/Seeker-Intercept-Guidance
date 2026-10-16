**# Theory Note**



This note summarizes the model used in the project and connects the main equations to the parameters in \`config/experiment.json\`. The setup involves a planar interceptor that receives noisy, delayed seeker measurements, estimates relative motion with an EKF, and generates a lateral-acceleration command using either proportional navigation or a short-horizon predictive controller.



**## 1. Vehicle model**



The interceptor moves at constant speed \$V_I\$. Its state is position \$p_I=[x_I,y_I]^T\$, heading \$\psi_I\$, and actual lateral acceleration \$a_I\$:



$$

\dot x_I = V_I\cos\psi_I,

\qquad

\dot y_I = V_I\sin\psi_I,

$$



$$

\dot\psi_I = \frac{a_I}{V_I}.

$$



The guidance law outputs a commanded lateral acceleration



$$

u = a_{\mathrm{cmd}},

$$



which passes through a first-order actuator model,



$$

\tau_a \dot a_I + a_I = u.

$$



Acceleration magnitude and slew rate are limited separately.



The target uses the same constant-speed planar model,



$$

\dot x_T = V_T\cos\psi_T,

\qquad

\dot y_T = V_T\sin\psi_T,

$$



$$

\dot\psi_T = \frac{a_T}{V_T}.

$$



For each Monte Carlo engagement, \$a_T\$ is sampled once from the configured range and then held constant. A zero value gives straight-line motion; a nonzero value gives a circular arc.



**## 2. Relative state and seeker model**



Relative position and velocity are



$$

r = p_T-p_I,

\qquad

v_r = v_T-v_I.

$$



The EKF state is



$$

x_r =

\begin{bmatrix}

r_x & r_y & v_{r,x} & v_{r,y}

\end{bmatrix}^{T}.

$$



The seeker measures only range and bearing,



$$

R = \sqrt{r_x^2+r_y^2},

$$



$$

\lambda = \operatorname{atan2}(r_y,r_x),

$$



with configurable range noise, bearing noise, measurement dropout, update rate, and latency. Neither guidance law receives target truth directly.



**## 3. Relative-state EKF**



The EKF estimates relative position and velocity from the nonlinear range/bearing measurements. Its prediction step is



$$

\hat x_k^- = f(\hat x_{k-1}),

$$



$$

P_k^- = F_kP_{k-1}F_k^T + Q,

$$



and the measurement update is



$$

K_k =

P_k^-H_k^T

\left(

H_kP_k^-H_k^T + R_k

\right)^{-1},

$$



$$

\hat x_k =

\hat x_k^- +

K_k\left(z_k-h(\hat x_k^-)\right).

$$



The process covariance \$Q\$ represents uncertainty from unmodeled relative acceleration and target maneuvering. \$R_k\$ is set by the seeker noise model.



Measurements retain their timestamps. After a delayed measurement is processed, the estimate is propagated to the current guidance time before the controller is evaluated. This gives timestamp-aware delay compensation for the measurement ordering used here, but it is not a full out-of-sequence-measurement smoother.



The guidance stack also maintains a smoothed estimate of target lateral acceleration. The smoothing factor and maximum allowed estimate are set in the estimator configuration.



**## 4. Proportional navigation**



The PN baseline is computed from the estimated relative state.



Range rate is



$$

\dot R = \frac{r^Tv_r}{R},

$$



so the closing velocity is



$$

V_c = -\dot R.

$$



The line-of-sight rate is



$$

\dot\lambda =

\frac{

r_xv_{r,y}-r_yv_{r,x}

}{

R^2

}.

$$



The PN command is



$$

u_{\mathrm{PN}} = NV_c\dot\lambda,

$$



where \$N\$ is the navigation constant.



PN itself is not an optimization problem. After the command is generated, it passes through the same acceleration limit, slew limit, and actuator dynamics used in the predictive case.



**## 5. Predictive guidance**



The predictive controller starts from the same EKF estimate, but projects candidate control sequences forward over a finite horizon.



A time-to-go estimate is formed as



$$

t_{\mathrm{go}} =

\operatorname{clip}

\left(

-\frac{r^Tv_r}{v_r^Tv_r},

0,

t_{\max}

\right),

$$



and the corresponding zero-effort-miss quantity is



$$

ZEM =

\left\|

r+v_rt_{\mathrm{go}}

\right\|.

$$



The cost used by the current controller is



$$

J =

w_z ZEM^2

+

w_\sigma \sigma_{\mathrm{miss}}^2

+

w_u\sum_{j=0}^{H-1}u_j^2

+

w_{\Delta u}

\sum_{j=1}^{H-1}(u_j-u_{j-1})^2.

$$



The first term penalizes predicted terminal miss. The second penalizes covariance-derived uncertainty in that miss prediction. The last two terms penalize control effort and rapid command changes.



\`weight_uncertainty\` controls the uncertainty term. Setting it to zero recovers the earlier ZEM + effort + slew objective. It is worth noting that this is not a direct latency penalty. Latency can increase propagated estimator uncertainty, which can then affect the covariance-based term.



The forward prediction includes the estimated target lateral acceleration, actuator lag, acceleration limits, and slew-rate limits. The implementation uses a small coordinate-search optimizer with control blocking rather than a general nonlinear-programming package.



Only the first command from the optimized sequence is applied. At the next guidance update, the estimate is refreshed and the problem is solved again. The controller can also fall back to PN or hand off to PN near the terminal portion of the engagement, depending on the configured logic.



**## 6. Configuration reference**



The JSON file is the main source of experiment settings. The parameters below are the ones that materially affect the model or validation.



**### Simulation**



\| Parameter | Role |

\|---|---|

\| \`truth_dt_s\` | Numerical integration step for truth dynamics. |

\| \`guidance_dt_s\` | Guidance update period. \`0.01 s\` corresponds to 100 Hz. |

\| \`max_time_s\` | Maximum engagement duration. |

\| \`intercept_radius_m\` | Range threshold used to classify a hit. |



**### Interceptor**



\| Parameter | Role |

\|---|---|

\| \`speed_mps\` | Constant interceptor speed \$V_I\$. |

\| \`initial_position_m\` | Initial interceptor position. |

\| \`initial_heading_rad\` | Initial heading. |

\| \`initial_lateral_accel_mps2\` | Initial actuator state. |

\| \`max_lateral_accel_mps2\` | Lateral-acceleration limit. |

\| \`max_slew_mps3\` | Acceleration slew-rate limit. |

\| \`actuator_tau_s\` | First-order actuator time constant \$\tau_a\$. |



**### Target**



\| Parameter | Role |

\|---|---|

\| \`speed_mps\` | Constant target speed \$V_T\$. |

\| \`initial_downrange_m\` | Monte Carlo range for initial downrange position. |

\| \`initial_lateral_offset_m\` | Monte Carlo range for initial lateral offset. |

\| \`initial_heading_center_rad\` | Nominal target heading. |

\| \`initial_heading_offset_rad\` | Random heading offset about the nominal value. |

\| \`lateral_accel_mps2\` | Constant target lateral acceleration sampled per engagement. |



**### Seeker**



\| Parameter | Role |

\|---|---|

\| \`update_rate_hz\` | Seeker measurement rate. |

\| \`range_sigma_m\` | Standard deviation of range noise. |

\| \`bearing_sigma_rad\` | Standard deviation of bearing noise. |

\| \`dropout_probability\` | Probability that a measurement is unavailable. |

\| \`latency_s\` | Latency range; one latency value is sampled per engagement. |



**### Estimator**



\| Parameter | Role |

\|---|---|

\| \`initial_position_variance_m2\` | Initial relative-position covariance scale. |

\| \`initial_velocity_variance_m2ps2\` | Initial relative-velocity covariance scale. |

\| \`process_accel_sigma_mps2\` | Process-noise scale for maneuver uncertainty. |

\| \`target_accel_smoothing_alpha\` | Smoothing applied to the target-acceleration estimate. |

\| \`target_accel_max_mps2\` | Bound on the estimated target lateral acceleration. |



**### Guidance**



For PN, \`navigation_constant\` is the gain \$N\$.



The predictive controller uses:



\| Parameter | Role |

\|---|---|

\| \`prediction_dt_s\` | Internal prediction step. |

\| \`horizon_steps\` | Number of prediction steps \$H\$. |

\| \`weight_zem\` | Terminal miss weight \$w_z\$. |

\| \`weight_uncertainty\` | Predicted miss-uncertainty weight \$w\_\sigma\$. |

\| \`weight_accel\` | Control-effort weight \$w_u\$. |

\| \`weight_slew\` | Command-change weight \$w\_{\Delta u}\$. |

\| \`optimizer_iterations\` | Coordinate-search refinement passes. |

\| \`optimizer_step_fraction\` | Initial search step relative to command scale. |

\| \`optimizer_min_step_mps2\` | Minimum search step. |

\| \`control_blocks\` | Number of blocked control variables across the horizon. |

\| \`step_decay\` | Search-step reduction factor. |

\| \`pn_handoff_tgo_multiplier\` | Parameter used in the near-terminal PN handoff. |

\| \`max_time_to_go_s\` | Upper bound on the time-to-go estimate. |

\| \`fallback_to_pn_if_better\` | Allows PN to be selected when it gives the better predicted result. |



**### Monte Carlo and timing**



\`monte_carlo.runs\` sets the number of paired engagements. \`monte_carlo.seed\` controls the random seed, and \`"random"\` requests a fresh seed that is printed for reproducibility. \`max_workers\` sets the parallel worker limit.



The timing benchmark draws random relative states from its configured ranges and measures predictive-guidance execution time independently of the full Monte Carlo simulation.



**### Simulink reference case**



\`simulink_reference\` defines the single engagement used by \`run_demo\`: initial target geometry, target acceleration, latency, and the random seeds used by the sensor/dropout blocks. It is mainly there for visualization and integration testing rather than statistical evaluation.



**## 7. Validation quantities**



PN and predictive guidance are evaluated on paired Monte Carlo cases. For each pair, both controllers use the same initial geometry, target maneuver, latency, sensor noise, and dropout realization.



A hit is recorded if



$$

\min_t R(t) \le R_{\mathrm{hit}}.

$$



The actual closest approach is



$$

d_{\min} = \min_t R(t).

$$



The simulation continues through first closest approach rather than stopping immediately at the hit radius, so \$d\_{\min}\$ remains a closest-approach measure even for successful runs.



Control effort is approximated by



$$

E \approx \sum_k u_k^2\Delta t.

$$



Relative-position RMSE is used to track estimator quality, and predictive-guidance execution time is compared with the guidance-cycle budget. At 100 Hz, the nominal budget is 10 ms per update.



**## 8. Scope**



The comparison is meant to study tradeoffs rather than prove that one controller always dominates the other. PN is already effective in many engagements. The predictive formulation mainly adds the ability to account explicitly for a short future horizon, actuator behavior, command constraints, target-motion estimates, and estimator uncertainty.



The current project remains a 2-D simulation/HIL study with constant vehicle speeds, a simplified seeker, first-order actuator dynamics, and no physical flight hardware.
