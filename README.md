# AMR Simulation Scalability and Control Reliability

An experimental investigation into the scalability, performance, and control reliability of multi-robot simulations using **ROS 2 Jazzy, Gazebo Harmonic, and Clearpath Robotics**.

The project investigates how simulation performance and robot responsiveness change as the number of simulated autonomous mobile robots (AMRs) increases.

The objective is to identify performance bottlenecks through controlled, reproducible experiments rather than assuming that simulation slowdown and loss of robot control have the same underlying cause.

## 1. Research Objectives

This investigation examines how increasing the number of simulated AMRs affects:

1. Simulation performance and real-time execution.
2. Robot-command responsiveness and control reliability.
3. Computational resource utilization.
4. Sensor data publication and communication.
5. Overall simulation stability.

The experiments aim to distinguish between potential bottlenecks in physics computation, sensor rendering, robot controllers, and ROS 2 communication.

## 2. Simulation Environment

### Hardware and Software

| Component | Configuration |
|---|---|
| Host OS | Ubuntu 24.04 LTS |
| Containerization | Docker and Docker Compose |
| ROS Distribution | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic 8.15.0 |
| Robot Platform | Clearpath Jackal J100 |
| Simulation World | Clearpath Warehouse |
| GPU | NVIDIA GeForce RTX 3060 (12 GB) |
| Robot Sensors | Hokuyo 2D LiDAR and simulated IMU |
| Controller Update Rate | 50 Hz (configured) |
| Simulation Architecture | Single Gazebo server |
| Initial Robot Count | 1 |

### Container Environment

The simulation runs inside Docker to minimize host dependencies and improve reproducibility.

The initial environment uses Clearpath's official simulation image:

`ghcr.io/clearpathrobotics/clearpath_docker:jazzy-sim-latest`

For reproducibility, the Docker Compose configuration references the following image digest:

```text
sha256:970015d533d52c816d1d59ec10cfc2d5df20977b8d8d34c89c67f58ee6543315
```

The repository also includes the upstream Clearpath Simulator as a Git submodule, pinned to commit:

```text
7a3e5d38dcc37b83dab1de0d986501234d669f78
```

**Important:** The current Docker environment uses Clearpath packages installed in the official container image. The Git submodule provides a version-controlled upstream source reference but is not currently compiled into the container. Package versions inside the image must be documented independently.

### Robot Configuration

The initial robot is a Clearpath Jackal J100 configured with:

- Four-wheel differential/skid-steer motion control.
- Wheel odometry.
- A front-mounted Hokuyo 2D LiDAR.
- Simulated inertial measurements.
- ROS 2 velocity-command interfaces.

Additional rendering-intensive sensors, such as 3D LiDAR and cameras, may be introduced as controlled experimental variables.

The source robot configuration is stored in:

```text
config/jackal/robot.yaml
```

Generated controller, robot-description, and sensor-configuration files are excluded from version control because Clearpath recreates them during initialization.

## 3. Experimental Methodology

The investigation proceeds incrementally.

### Phase 1 — Environment and Functional Validation

Establish a working simulation containing one Jackal.

Validation includes:

- Launching the warehouse environment.
- Spawning the robot.
- Activating its controllers.
- Verifying velocity-command execution.
- Confirming odometry publication.
- Confirming sensor-data publication.
- Establishing reliable performance-measurement procedures.

**Status:** Functional validation and automated performance measurement validation completed. Warehouse collision verification remains pending.

### Phase 2 — Single-Robot Performance Baseline

Develop automated experiments to establish reference measurements for one sensor-equipped Jackal.

The baseline will measure:

| Metric | Purpose |
|---|---|
| Real-Time Factor (RTF) | Simulation speed relative to real time |
| CPU Utilization | Processor workload |
| Memory Utilization | Resource consumption |
| GPU Utilization | Rendering and computational workload |
| LiDAR Publication Rate | Sensor performance |
| Command Responsiveness | Robot control behavior |
| Odometry | Observed simulated motion |
| Tracking Error | Difference between commanded and observed behavior |

Measurements will distinguish wall-clock time from simulation time.

### Phase 3 — Incremental Multi-Robot Scaling

Increase the number of simulated Jackals while maintaining a consistent robot configuration and experimental workload.

The initial planned robot counts are:

```text
1 → 2 → 3 → 5 → 10 → 20 → ... → 100+
```

The progression will be adjusted according to measured results and hardware limitations.

Each robot will have:

- A unique identity and namespace.
- An independent control interface.
- An identical baseline sensor configuration.
- A defined starting position.
- A reproducible control workload.

Experiments will use repeated runs under controlled conditions.

### Phase 4 — Bottleneck Investigation

When performance degradation or control failures are observed, investigate potential causes individually.

Areas of investigation include:

- Physics computation.
- Sensor simulation and rendering.
- ROS 2 communication and discovery.
- Controller execution.
- CPU and GPU resource saturation.
- Simulation scheduling and timing.

Controlled comparisons will be used to distinguish correlation from supported explanations.

For example, disabling LiDAR while preserving robot count and control workload can help estimate the performance impact of sensor simulation.

### Phase 5 — Analysis and Conclusions

Analyze the experimental measurements to determine:

- How simulation performance scales with robot count.
- Whether robot responsiveness changes as simulation load increases.
- Which resources become limiting.
- Whether failures originate in simulation, communication, or control.
- Which changes measurably improve performance or reliability.

Conclusions will be supported by recorded measurements and documented experimental conditions.

## 4. Reproducing the Simulation

### Prerequisites

The current setup requires:

- Ubuntu or another compatible Linux environment.
- Docker Engine and Docker Compose.
- NVIDIA container support for the current GPU-enabled configuration.
- A graphical environment compatible with the Docker Compose display configuration.

ROS 2 and Gazebo do not need to be installed on the host.

### Clone the Repository

```bash
git clone --recurse-submodules \
  https://github.com/mathrosas/amr-simulation-scalability.git

cd amr-simulation-scalability
```

### Launch the Simulation

```bash
docker compose up
```

This launches the Clearpath warehouse environment with one configured Jackal J100.

For the current configuration, Gazebo's graphical interface opens on the host.

### Verify Container Health

In another terminal:

```bash
docker ps
```

The simulation container should report a healthy status after initialization.

### Access the ROS 2 Environment

```bash
docker compose exec simulation bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
```

ROS 2 commands can then be executed without installing ROS on the host.

### Independent RViz2 Visualization

Start the visualization service while the simulation is running:

```bash
docker compose \
  -f compose.yaml \
  -f compose.visualization.yaml \
  up -d --no-deps visualization
```

Stop visualization independently:

```bash
docker compose \
  -f compose.yaml \
  -f compose.visualization.yaml \
  stop visualization
```

### Stop the Simulation

Gracefully stop both services without removing their containers:

```bash
docker compose \
  -f compose.yaml \
  -f compose.visualization.yaml \
  stop -t 45
```

The Gazebo model cache is stored in a persistent Docker volume.

The visualization configuration and experimental results are stored
in the repository independently of the running containers.

## 5. Experimental Results

### Milestone 0 — Repository Initialization

**Status: Completed**

The research repository was initialized and connected to GitHub.

The official Clearpath Simulator was included as a Git submodule.

The initial research objectives, experimental methodology, and project structure were established.

### Milestone 1 — Container Environment Configuration

**Status: Completed**

The official Clearpath Docker simulation image was downloaded and validated.

The following software components were confirmed:

| Component | Result |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic 8.15.0 |
| Clearpath Gazebo Package | Installed |
| NVIDIA Container Runtime | Available |
| Jackal Configuration Parser | Validation passed |

A sensor-equipped Jackal J100 configuration was created and validated using Clearpath's configuration parser.

### Milestone 2 — Single-Robot Functional Validation

**Date:** November 15, 2026
**Status:** Functional validation passed.

#### Objective

Verify that a simulated Jackal can be spawned, controlled, and observed through ROS 2 while publishing sensor data.

#### Environment

- Gazebo Harmonic 8.15.0.
- ROS 2 Jazzy.
- Clearpath warehouse world.
- One Jackal J100.
- One front-mounted Hokuyo 2D LiDAR.
- Simulated IMU.
- Active wheel controllers.

#### Controller Verification

Both controllers were confirmed active through the ROS 2 controller manager.

| Controller | State | Configured Update Rate |
|---|---|---|
| `joint_state_broadcaster` | Active | 50 Hz |
| `platform_velocity_controller` | Active | 50 Hz |

These values represent the controllers' configured update rates, not independently measured execution frequencies.

#### Sensor Verification

The Gazebo simulator successfully initialized the 2D LiDAR.

The corresponding ROS 2 bridge was created, and a `sensor_msgs/msg/LaserScan` message was successfully received.

Filtered odometry was also received through ROS 2.

The IMU initialized and reported receiving sensor data.

#### Movement Test

A sequence of velocity commands was published through ROS 2.

| Parameter | Value |
|---|---|
| Commanded Linear Velocity | 0.2 m/s |
| Commanded Angular Velocity | 0 rad/s |
| Command Publication Rate | 10 Hz |
| Number of Commands | 30 |
| Initial X Position | Approximately 0 m |
| Final X Position | Approximately 0.718 m |
| Reported X Displacement | Approximately 0.718 m |

The robot was visually observed moving forward and stopping correctly after the command sequence.

The nominal commanded travel distance for three seconds at 0.2 m/s is 0.6 m.

However, this manual test did not synchronize command timestamps, simulation time, and odometry measurements. Therefore, the difference between the nominal distance and the reported displacement cannot yet be interpreted as a measured tracking error.

#### Outstanding Observations

**1. ROS 2 discovery inconsistencies**

ROS 2 topic-listing commands returned incomplete or inconsistent information.

Direct subscriptions using explicit message types successfully received odometry and LiDAR data.

The underlying cause remains undetermined.

**2. Physics collision warnings**

Gazebo reported DART physics warnings indicating that certain mesh collision geometries could not be created.

Warehouse obstacle collisions must be verified before collision-dependent experiments are conducted.

**3. Simulation timing**

The movement test confirmed functional command execution but did not measure controller latency or trajectory-tracking accuracy.

These measurements require an automated test using synchronized timestamps.

**4. Asset caching**

The initial warehouse launch downloaded Gazebo Fuel resources.

Subsequent benchmark experiments must account for differences between initial startup and runs using cached assets.

#### Conclusion

The single-Jackal functional validation was successful.

The simulation demonstrated:

- Successful robot spawning.
- Controller initialization.
- ROS 2 velocity-command execution.
- Forward movement and automatic stopping.
- Odometry publication.
- LiDAR data publication.

This establishes a functional foundation for subsequent benchmarking.

**No conclusions about multi-robot scalability, command latency, or performance degradation can yet be drawn from this experiment.**

### Milestone 3 — Automated Single-Robot Baseline

**Dates:** November 16–18, 2026
**Status:** Completed.

#### Simulation Performance and Resource Monitoring

Automated collection tools were developed to measure Gazebo real-time factor, CPU utilization, memory consumption, and GPU utilization.

The unified baseline runner combines simulation statistics, resource measurements, and optional ROS 2 sensor timing measurements.

Relevant scripts:

- `scripts/collect_rtf.py`
- `scripts/collect_resources.py`
- `scripts/run_baseline.py`

#### ROS 2 Sensor Timing

Automated monitoring was implemented for:

- LiDAR.
- Raw odometry.
- Filtered odometry.
- IMU.
- Joint states.

Measurements include message arrival timestamps, ROS message header timestamps, arrival intervals, and message timestamp intervals.

Relevant scripts:

- `scripts/ros_sensor_monitor.py`
- `scripts/analyze_sensor_timing.py`

#### DDS Communication Benchmark

A controlled same-host ROS 2 publisher/subscriber benchmark was implemented to measure application-level communication performance.

The benchmark records latency, message delivery, missing messages, duplicate messages, and message ordering.

Three repeatability trials were conducted using Reliable QoS.

The DDS benchmark is independent of robot motion and should not be interpreted as a measurement of complete robot-control latency.

Relevant scripts:

- `scripts/dds_probe.py`
- `scripts/analyze_dds_probe.py`

#### Automated Motion-Control Benchmark

A motion-control probe was developed to evaluate the response of the Clearpath Jackal to repeatable velocity commands.

Four observation points are included in the final implementation:

| Stream | Observation point |
|---|---|
| Requested command | Local velocity publication |
| Multiplexer output | `/j100_0000/platform/cmd_vel` |
| Controller output | `/j100_0000/platform_velocity_controller/cmd_vel_out` |
| Odometry | `/j100_0000/platform/odom` |

The probe records monotonic wall-clock timestamps, simulation time, ROS message header timestamps, velocity, and odometry position.

The controller was verified to use feedback-based odometry
(`open_loop=false`).

##### Experimental Configuration

| Parameter | Value |
|---|---|
| Robot count | 1 |
| Linear velocity | 0.10 m/s |
| Angular velocity | 0 rad/s |
| Command publication frequency | 20 Hz |
| Initial stationary period | 1 s |
| Movement duration | 2 s (wall time) |
| Stopping observation period | 2 s |
| Controller command timeout | 0.5 s |
| Gazebo GUI | Enabled |
| RViz2 | Disabled during repeatability trials |
| Nav2 | Not running |
| Open-RMF | Not running |

One functional smoke test and two sets of three repeatability trials were conducted.

The initial smoke test was conducted with RViz2 running. The repeatability datasets were collected with RViz2 stopped.

The second repeatability dataset included multiplexer instrumentation.

##### Instrumented Repeatability Results

Results from three trials:

| Metric | Mean | Sample standard deviation |
|---|---:|---:|
| Command frequency (Hz) | 19.9541 | 0.0050 |
| Controller 90% response (ms) | 49.582 | 0.085 |
| Odometry 90% response (ms) | 217.704 | 0.167 |
| Multiplexer stopping response (ms) | 0.179 | 0.008 |
| Controller stopping response (ms) | 23.415 | 0.230 |
| Odometry stopping response (ms) | 212.552 | 0.209 |
| Steady-state odometry velocity (m/s) | 0.100060 | 0.000105 |
| Odometry displacement (mm) | 198.240 | 0.073 |
| Displacement error (%) | -0.930 | 0.036 |
| Effective motion RTF | 1.000441 | 0.000004 |

All three instrumented trials completed successfully and reached effectively zero odometry velocity during the final observation window.

##### Stopping Anomaly

One trial in the original repeatability dataset exhibited an unexpected stopping delay.

The controller continued reporting approximately 0.10 m/s for approximately 250 ms after zero-velocity commands began.

The observed odometry stopping response was 465.116 ms, compared with approximately 212.5 ms in the other two original trials.

Odometry displacement increased from approximately 198.3 mm to 223.5 mm.

Inspection of the recorded velocity profiles indicated that the additional delay occurred before the controller output decreased, rather than during the subsequent odometry deceleration.

Multiplexer monitoring was subsequently added to improve visibility into the command pipeline.

The anomaly did not recur in the three instrumented trials.

Its underlying cause remains undetermined. All experimental results, including the anomalous trial, are retained.

##### Experimental Limitations

- Odometry is not independent Gazebo ground truth.
- Observed response times include communication, processing, controller updates, simulated dynamics, and callback scheduling.
- The experiments do not measure pure DDS latency through the complete robot-control pipeline.
- Displacement includes the deceleration period.
- Starting poses were not reset between consecutive trials.
- Three trials per configuration provide preliminary repeatability measurements, not statistically established reliability limits.
- These results do not establish multi-robot scalability.

##### Scripts and Datasets

Relevant scripts:

- `scripts/control_probe.py`
- `scripts/analyze_control_probe.py`
- `scripts/analyze_control_repeat.py`

Experimental datasets:

- `results/raw/control_probe_smoke_1robot_20261118_061410/`
- `results/raw/control_probe_repeat_1robot_20261118_061938/`
- `results/raw/control_probe_mux_repeat_1robot_20261118_064334/`

Individual experiments contain raw CSV measurements and JSON analysis.
Repeated experiments also include aggregate descriptive statistics.

#### Independent RViz2 Visualization

RViz2 was validated using the Clearpath visualization configuration.

The following functionality was verified:

- Jackal robot model visualization.
- Namespaced TF transformations.
- Simulation clock synchronization.
- LiDAR visualization.
- Simultaneous Gazebo and RViz2 operation.
- Independent visualization service startup and shutdown.

RViz2 runs in a separate Docker container using the same pinned
Clearpath image as the simulation.

Configuration files:

- `compose.visualization.yaml`
- `config/rviz/config.rviz`

Visualization is disabled during standard performance experiments unless its computational overhead is an experimental variable.

#### Conclusion

The automated single-robot baseline establishes reference measurements for simulation performance, resource consumption, sensor timing, communication, and motion-control responsiveness.

The next development stage introduces autonomous navigation through Nav2, followed by multi-robot simulation and Open-RMF coordination.

## 6. Current Project Status

| Milestone | Status |
|---|---|
| 0 — Repository Initialization | Completed |
| 1 — Container Environment Configuration | Completed |
| 2 — Single-Robot Functional Validation | Completed |
| 3A — RTF and Resource Monitoring | Completed |
| 3B — Automated Sensor Monitoring | Completed |
| 3C — ROS 2 Graph Validation | Completed |
| 3D — DDS Communication Benchmarking | Completed |
| 3E — Automated Motion-Control Benchmarking | Completed |
| 3F — Independent RViz2 Visualization | Completed |
| 3G — Nav2 Autonomous Navigation | Next |
| 4A — Two-Robot Simulation | Planned |
| 4B — Open-RMF Fleet Coordination | Planned |
| 4C — Incremental Fleet Scalability | Planned |
| 5 — Bottleneck Investigation | Planned |
| 6 — Analysis and Research Report | Planned |

**Next objective:** Integrate Nav2 into the existing containerized environment and validate autonomous navigation with a single Jackal before introducing additional robots and Open-RMF fleet coordination.

## 7. Reproducibility and Future Deliverables

The project will progressively include:

- Version-controlled robot configurations.
- Docker Compose configurations.
- A custom Dockerfile when additional software is required.
- Versioned Docker images published to a container registry.
- Automated experiment scripts.
- Raw measurement datasets.
- Processed experimental results.
- Performance graphs and visualizations.
- Documented experimental limitations.

Results will be added as experiments are completed and validated.

## 8. Upstream Projects

This research builds on the official Clearpath Robotics software ecosystem.

**Clearpath Simulator:**
https://github.com/clearpathrobotics/clearpath_simulator

**Clearpath Docker:**
https://github.com/clearpathrobotics/clearpath_docker

**Clearpath Configuration:**
https://github.com/clearpathrobotics/clearpath_config

The investigation is independent and does not represent an official Clearpath Robotics benchmark.