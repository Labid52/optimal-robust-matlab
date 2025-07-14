# Inverted Pendulum on a Cart (IPOC) Control System

This repository presents a collection of MATLAB scripts and tools for the simulation and control of an inverted pendulum on a cart (IPOC) system. The IPOC is a widely studied benchmark problem in control theory due to its nonlinear and unstable dynamics. This project explores multiple control strategies including:

* **Pontryagin’s Minimum Principle (PMP)**-based optimal control
* **Linear Quadratic Regulator (LQR)**
* **H-infinity (\$H\_\infty\$) loop shaping control** (single and double Riccati approach)

Simulation results compare the performance, stability, and robustness of these controllers. The goal is to evaluate the trade-offs between optimality and robustness in the presence of disturbances and system uncertainties.

---

## 🔧 Files and Scripts

The repository includes the following core `.m` MATLAB files:

```text
pendcart.m
project_loop_shaping.m
project_loop_shaping_1.m
project_optimal_dan_robas_double_H_infty.m
project_optimal_dan_robas_hloop1.m
project_optimal_dan_robas_lqr.m
project_optimal_dan_robas_lqr_1.m
project_optimal_dan_robas_single_H_infty.m
simpend.m
lqr_dynamic_programming.m
lqr_pendulum_cart.m
```


