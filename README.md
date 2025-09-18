README DEPRECATED

# LASER UAV Controllers

This package provides **C++ classes** that implement feedback controllers for the Laser UAV System (LUS), translating desired trajectories into low-level commands.

## Overview and Algorithms

The main objective of this package is to provide controllers capable of tracking desired **reference**.

### NMPCController
-   **Description:** A Nonlinear Model Predictive Controller (NMPC) that uses a dynamic model of the UAV to predict its future states. It optimizes control inputs over a finite time horizon to follow a reference trajectory while respecting system constraints.
-   **Reference:**
    ```bibtex
    @article{sun2022comparative,
        title={A comparative study of nonlinear mpc and differential-flatness-based control for quadrotor agile flight},
        author={Sun, Sihao and Romero, Angel and Foehn, Philipp and Kaufmann, Elia and Scaramuzca, Davide},
        journal={IEEE Transactions on Robotics},
        volume={38},
        number={6},
        pages={3357--3373},
        year={2022},
        publisher={IEEE}
    }
    ```
-   **Configurable Parameters:**
    ```yaml
    nmpc_controller:
      quadrotor_parameters:
        # Quadrotor Mass
        mass: 1.60 # Kg

        # Diagonal of Inertia Matrix
        inertia: [0.021667, 0.021667, 0.0425]

        # Torque constant {Ctau}
        c_tau: 0.59 # This constant is the torque prop coefficient divided by thrust coefficient cq / ct

        # Drag in x, y and z
        drag: [0.0, 0.0, 0.0] # For now we dont use this

        # Motors positions to the CoM
        motors_positions: [-0.185, -0.18, 0.185, -0.18, -0.185, 0.18, 0.185, 0.18] # {motor_0_x, motor_0_y, motor_1_x, motor_1_y, ......}

        # This is used to model the thrust x throtle curve for normalize input to the px4
        quadratic_motor_model:
          a: 0.27665
          b: -0.19642

        # Thrust Constraints
        thrust_min: 0.0 # N
        thrust_max: 15.7 # N
        total_thrust_max: 62.8 # N

      acados_parameters:
        N: 30 # Horizon length
        dt: 0.1 # Discrete step in horizon

        # This is the Weight Matrix that penalize state error in FC
        # {W_position_xy, W_position_z, W_rotation_xy, W_rotation_z, W_speed, W_angular_speed}
        Q: [10.0, 8.0, 0.2, 3.0, 0.3, 0.3] # Real drone gains 

        # This is the weight Matrix that penalize input control effort in FC
        R: 0.0 # stable on gazebo
    ```
