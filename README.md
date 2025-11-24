# LASER UAV Controllers

This package provides **C++ classes** that implement feedback controllers for the Laser UAV System (LUS), translating desired trajectories into low-level commands.

## Overview and Algorithms

The main objective of this package is to provide controllers capable of tracking desired **reference**.

### NMPC Controller
-   **Description:** Nonlinear Model Predictive Control (NMPC) is an advanced, high-level control algorithm that utilizes a dynamic model of the UAV to predict its future states over a finite time horizon. The controller optimizes control inputs (commands) by iteratively minimizing a cost function to follow a reference trajectory while respecting system constraints (e.g., actuator limits, maximum velocities). This framework enables efficient and highly accurate tracking for complex autonomous maneuvers.

-   **Reference:**
    S. Sun, A. Romero, P. Foehn, E. Kaufmann and D. Scaramuzza, "A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight," in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3357-3373, Dec. 2022, doi: 10.1109/TRO.2022.3177279.

-   **Configurable Parameters:**
    ```yaml
      acados_parameters:
        nmpc_mode: "individual_thrust" # individual_thrust or angular_rates_and_thrust
        N: 30 # Horizon length
        dt: 0.1 # Discrete step in horizon

        # This is the Weight Matrix that penalize state error in FC
        # {W_position_xy, W_position_z, W_rotation_xy, W_rotation_z, W_speed, W_angular_speed}
        Q: [40.0, 30.0, 0.3, 5.0, 0.3, 1.0]

        # This is the weight Matrix that penalize input control effort in FC
        R: 1.0 
    ```
### INDI Controller
-   **Description:** Incremental Nonlinear Dynamic Inversion (INDI) is a robust, low-level, sensor-based controller that computes an incremental change in the control input. It achieves this by capturing and compensating for external disturbances and system nonlinearities through real-time measurement of angular acceleration and motor angular velocity. Consequently, it maintains the required angular acceleration provided by the high-level controller with enhanced efficiency.

-   **Reference:**
    E. Tal and S. Karaman, "Accurate Tracking of Aggressive Quadrotor Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential Flatness," in IEEE Transactions on Control Systems Technology, vol. 29, no. 3, pp. 1203-1218, May 2021, doi: 10.1109/TCST.2020.3001117
