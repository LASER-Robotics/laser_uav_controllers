# LASER UAV Controllers

`laser_uav_controllers` implements the NMPC and INDI controllers used by the LASER UAV System (LUS). They turn a desired state or trajectory and the measured UAV state into control corrections.

The package provides a C++ library. The `control_manager` node in `laser_uav_managers` runs the control loops, reads the configuration and publishes commands to the hardware API.

## Controllers

### NMPC

`NmpcController` predicts the vehicle motion over a finite horizon and solves an optimal control problem with ACADOS. Its cost penalizes tracking error and control effort, subject to the vehicle's thrust and angular-rate limits.

`getCorrection()` accepts either one `laser_msgs/msg/ReferenceState` or a vector of reference states, together with `nav_msgs/msg/Odometry`. The manager uses the result to produce individual motor commands or angular rates and total thrust, depending on `nmpc_mode`.

### INDI

`IndiController` adjusts individual thrust using measured angular acceleration, motor speed and the vehicle allocation matrices. The manager runs it as the inner loop when using individual thrust control.

`getCorrection()` receives angular acceleration, motor-speed estimates, the nominal individual thrust and angular velocity. It returns the corrected motor command vector. Its internal timestep is 0.004 s, corresponding to the supplied 250 Hz inner-loop configuration.

## Configuration

The vehicle-specific NMPC files are [params/nmpc_controller/lr7pro.yaml](params/nmpc_controller/lr7pro.yaml) and [params/nmpc_controller/x500.yaml](params/nmpc_controller/x500.yaml). Parameters are under `nmpc_controller` inside `ros__parameters`.

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `nmpc_mode` | `individual_thrust` | Selects `individual_thrust` or `angular_rates_and_thrust`. Match this to the hardware API's `control_input_mode`. |
| `N` | `20` | Number of prediction intervals. The reference horizon contains `N + 1` states. |
| `dt` | `0.05` | Prediction timestep in seconds, separate from the control-loop period. |
| `Q` | Vehicle-specific array of six weights | Tracking weights for horizontal position, vertical position, horizontal attitude, yaw, linear velocity and angular velocity, in that order. |
| `R` | `1.0` | Weight on control effort. |

Physical parameters, including mass, inertia, `G1`, `G2`, thrust coefficients and limits, come from the airframe YAML in `laser_uav_managers/params/laser_uavs/`. INDI uses these values too. Use the calibration for the actual airframe; the supplied values are not interchangeable between vehicles.

## Build and use

The [LUS installer](https://github.com/LASER-Robotics/laser_uav_system) builds the ACADOS submodule and configures `ACADOS_SOURCE_DIR` and the library path. If building manually, complete that setup before building this package.

From the workspace root:

```bash
colcon build --packages-up-to laser_uav_controllers
source install/setup.bash
```

To run the controllers as part of LUS, launch `control_manager` as described in [LASER UAV Managers](https://github.com/LASER-Robotics/laser_uav_managers). This library does not advertise ROS topics or services itself.

## References

- S. Sun, A. Romero, P. Foehn, E. Kaufmann and D. Scaramuzza, "A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight," IEEE Transactions on Robotics, 38(6), pp. 3357-3373, 2022. DOI: 10.1109/TRO.2022.3177279.
- E. Tal and S. Karaman, "Accurate Tracking of Aggressive Quadrotor Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential Flatness," IEEE Transactions on Control Systems Technology, 29(3), pp. 1203-1218, 2021. DOI: 10.1109/TCST.2020.3001117.
