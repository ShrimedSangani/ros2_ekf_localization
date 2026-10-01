# ROS 2 Extended Kalman Filter Localization

A from-scratch Extended Kalman Filter (EKF) for 2D TurtleBot3 localization using ROS 2 and Gazebo. The filter fuses noisy wheel-odometry information with IMU yaw measurements to estimate the robot state:

\[
\mathbf{x} = [x,\ y,\ \theta]^T
\]

The project includes synthetic sensor-noise injection, a custom EKF implementation, trajectory evaluation using `evo`, and a comparison against the ROS 2 `robot_localization` EKF.

## Results

The custom EKF reduced translation RMSE by approximately **36%** compared with the noisy odometry input.

| Method | Translation RMSE |
|---|---:|
| Noisy Odometry | 0.1412 m |
| **Custom EKF** | **0.0903 m** |
| ROS 2 `robot_localization` | 0.0669 m |

RMSE was calculated using Absolute Pose Error (APE) with SE(3) Umeyama alignment.

### Trajectory Comparison

![Trajectory Comparison](results/trajectory_comparision.png)

- **Black:** Gazebo `/odom` reference
- **Green:** `/odom_noisy`
- **Red:** Custom EKF `/ekf/pose`
- **Yellow:** ROS 2 `robot_localization` `/odometry/filtered`

The noisy odometry shows substantial position variation, while both EKF implementations produce smoother trajectories that more closely follow the reference.

### Position Estimates

![Position Comparison](results/position_comparision.png)

### Orientation Estimates

![Orientation Comparison](results/orientation_comparision.png)

The EKF is designed for planar localization, so yaw is the relevant orientation state.

## System Architecture

```text
                    TurtleBot3 + Gazebo
                           |
                         /odom
                           |
                 Synthetic Noise Wrapper
                    /             \
             /odom_noisy       /imu_noisy
                  |                 |
          x, y, v, omega           yaw
                  \                 /
                   \               /
                    Custom EKF
                        |
                    /ekf/pose
                        |
                 evo Evaluation
```

The original Gazebo `/odom` topic is not consumed by the custom EKF and is retained as a reference trajectory for evaluation.

## Extended Kalman Filter

### Prediction

The control input is

\[
\mathbf{u} = [v,\ \omega]^T
\]

where \(v\) is linear velocity and \(\omega\) is angular velocity.

For nonzero angular velocity, the nonlinear velocity motion model is

\[
x' = x - \frac{v}{\omega}\sin(\theta)
     + \frac{v}{\omega}\sin(\theta+\omega\Delta t)
\]

\[
y' = y + \frac{v}{\omega}\cos(\theta)
     - \frac{v}{\omega}\cos(\theta+\omega\Delta t)
\]

\[
\theta' = \theta+\omega\Delta t
\]

The covariance prediction is

\[
\Sigma' = G\Sigma G^T + R
\]

where \(G\) is the Jacobian of the nonlinear motion model and \(R\) is the process-noise covariance.

A separate straight-line model is used when angular velocity approaches zero to avoid division by zero.

### Measurement Update

The measurement vector is

\[
\mathbf{z} =
\begin{bmatrix}
x_{\text{odom}} \\
y_{\text{odom}} \\
\theta_{\text{imu}}
\end{bmatrix}
\]

Since the measurements directly observe the three state variables,

\[
h(\mathbf{x}) = \mathbf{x}
\]

and therefore

\[
H = I_{3\times3}
\]

The Kalman gain is

\[
K = \Sigma'H^T(H\Sigma'H^T + Q)^{-1}
\]

followed by

\[
\mu = \mu' + K(\mathbf{z}-h(\mu'))
\]

\[
\Sigma = (I-KH)\Sigma'
\]

where \(Q\) represents measurement-noise covariance.

## ROS 2 Topics

| Topic | Type | Purpose |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Gazebo reference trajectory |
| `/odom_noisy` | `nav_msgs/Odometry` | Synthetic noisy odometry |
| `/imu_noisy` | `sensor_msgs/Imu` | Synthetic noisy IMU |
| `/ekf/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Custom EKF estimate |
| `/odometry/filtered` | `nav_msgs/Odometry` | `robot_localization` estimate |

## Project Structure

```text
ekf_localization/
├── config/
│   └── ekf.yaml
├── ekf_localization/
│   ├── __init__.py
│   ├── ekf_node.py
│   └── synthetic_noise_wrapper.py
├── results/
│   ├── trajectory_comparision.png
│   ├── position_comparision.png
│   ├── orientation_comparision.png
│   └── velocity_comparision.png
├── resource/
│   └── ekf_localization
├── test/
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot3
- Gazebo Classic
- Python 3
- NumPy
- `robot_localization`
- `evo`

## Build

Clone the repository into the `src` directory of a ROS 2 workspace and build it:

```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ..

source /opt/ros/humble/setup.bash
colcon build --symlink-install

source install/setup.bash
```

## Run

Start the TurtleBot3 Gazebo simulation:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Start the synthetic noise wrapper:

```bash
ros2 run ekf_localization wrapper
```

Start the custom EKF:

```bash
ros2 run ekf_localization ekf_node --ros-args -p use_sim_time:=true
```

The custom estimate is published to:

```text
/ekf/pose
```

For comparison, `robot_localization` can be run using the provided configuration:

```bash
ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file <path-to-ekf.yaml> \
  -p use_sim_time:=true
```

## Evaluation

Trajectories were recorded with:

```bash
ros2 bag record /odom /odom_noisy /ekf/pose /odometry/filtered
```

Absolute Pose Error was evaluated using `evo`:

```bash
evo_ape bag2 <bag> /odom /ekf/pose -a
```

The custom EKF achieved a translation RMSE of **0.0903 m**, compared with **0.1412 m** for noisy odometry.

The filter was also run continuously for more than five minutes to check for numerical instability, NaNs, covariance divergence, or loss of pose output.

## Key Takeaways

This project demonstrates:

- Nonlinear state estimation using an Extended Kalman Filter
- Derivation and implementation of motion-model Jacobians
- Prediction and measurement-correction steps implemented from scratch
- Odometry and IMU sensor fusion
- Quaternion-to-yaw conversion
- Covariance propagation
- ROS 2 publishers, subscribers, callbacks, and timers
- Synthetic sensor-noise modeling
- ROS bag data collection
- Quantitative trajectory evaluation with APE/RMSE
- Comparison with the production ROS 2 `robot_localization` package

## License

MIT
