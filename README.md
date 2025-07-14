# Optimal and robust control of Inverted Pendulum on a Cart (IPOC)
[Late Upload]

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

<img width="700" height="525" alt="image" src="https://github.com/user-attachments/assets/14abd5ec-1c39-4414-b22d-64849e625cee" />

<img width="700" height="525" alt="image" src="https://github.com/user-attachments/assets/f6638772-7dac-4984-b058-88446557e139" />
<img width="700" height="525" alt="image" src="https://github.com/user-attachments/assets/3c1f30b3-9eff-4c1f-be8e-4a4a96037c4e" />
<img width="700" height="525" alt="image" src="https://github.com/user-attachments/assets/1284f3fd-ab9d-4b2a-b7ab-fc8ab8dd6684" />


