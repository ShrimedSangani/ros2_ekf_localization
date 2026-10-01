# ROS 2 EKF Localization

- Built a complete localization pipeline from **Gazebo simulation → synthetic sensor noise → EKF sensor fusion → ROS 2 pose estimation → quantitative evaluation**.
- Implemented an **Extended Kalman Filter (EKF) from scratch** for 2D TurtleBot3 localization using **ROS 2 and Gazebo**.

## Implementation

- Simulated a **TurtleBot3 Burger in Gazebo** and controlled the robot using ROS 2.
- Used `/odom` and `/imu` as the original simulated sensor measurements.

<p align="center">
  <img src="results/gazebo_simulation.png" width="600">
</p>

- Developed a **ROS 2 noise wrapper** that adds Gaussian noise to the simulated sensor measurements and publishes `/odom_noisy` and `/imu_noisy`.
- Used the state `[x, y, θ]`, where `x` and `y` represent position and `θ` represents the robot's heading.
- Implemented the **EKF prediction step** using linear and angular velocity with a nonlinear velocity motion model and its Jacobian.
- Implemented the **EKF correction step** by fusing noisy odometry position `(x, y)` with IMU yaw `θ`.
- Published the final localization estimate to `/ekf/pose`.
- Ran the same noisy sensor measurements through **ROS 2 `robot_localization`** for comparison.

```text
             Gazebo TurtleBot3
                    │
              /odom + /imu
                    │
                    ▼
              Noise Wrapper
               /         \
        /odom_noisy    /imu_noisy
               \         /
                \       /
                  ▼
              Custom EKF
                  │
              /ekf/pose
```

## Results

- Recorded the trajectories using **ROS bags** and evaluated them using **Absolute Pose Error (APE)** with `evo`.
- Reduced localization error by approximately **36%** compared with the noisy odometry input and validated the implementation against **`robot_localization`**.

| Method | Translation RMSE |
|---|---:|
| Noisy Odometry | 0.1412 m |
| **Custom EKF** | **0.0903 m** |
| `robot_localization` | 0.0669 m |

### Trajectory Comparison

<p align="center">
  <img src="results/trajectory_comparision.png" width="600">
</p>

**Black:** Reference odometry · **Green:** Noisy odometry · **Red:** Custom EKF · **Yellow:** `robot_localization`

- The **noisy odometry** shows noticeable variation from the reference trajectory, especially around turns.
- The **custom EKF** stays closer to the reference trajectory, while **`robot_localization`** achieved the lowest overall translational error.

### Position Comparison

<p align="center">
  <img src="results/position_comparision.png" width="600">
</p>

- The noisy `x` and `y` measurements fluctuate around the reference position throughout the run.
- After filtering, the **custom EKF** produces smoother position estimates that stay closer to the reference.

### Orientation Comparison

<p align="center">
  <img src="results/orientation_comparision.png" width="600">
</p>

- Since the EKF models **2D planar motion**, yaw `θ` is the orientation component used for localization.
- The filtered yaw follows the robot's heading while reducing the noise introduced into the IMU measurements.
