% File List for IPOC Controller Design
% 
% This folder contains MATLAB scripts for simulating and analyzing the control of an Inverted Pendulum on a Cart (IPOC) system. The following files implement various optimal and robust control strategies, including Pontryagin’s Minimum Principle and H-infinity loop shaping.

%% Script List
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

%% Controller Comparison Summary
% | Method           | Stability | Performance | Robustness | Comment                         |
% |------------------|-----------|-------------|------------|----------------------------------|
% | PMP-based        | ✓         | ✓✓          | ~          | Requires costate computation     |
% | LQR              | ✓         | ✓           | ✗          | Sensitive to disturbance         |
% | H∞ (single RE)   | ✓✓        | ✓✓          | ✓          | Trade-off via γ optimization     |
% | H∞ (double RE)   | ✓✓        | ✓✓          | ✓✓         | Better disturbance rejection     |
% | Loop Shaping     | ✓✓        | ✓✓          | ✓✓         | Design flexibility via weighting |


