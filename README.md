# AMR Simulation Scalability and Control Reliability

An experimental investigation into the performance and reliability of multi-robot simulations using Gazebo Harmonic and ROS 2 Jazzy.

## Research Objective

Investigate how increasing the number of simulated autonomous mobile robots (AMRs) affects:

1. Simulation performance and real-time execution.
2. Robot-command responsiveness and control reliability.
3. Computational resource utilization.
4. The stability of the simulation environment.

The objective is to identify performance bottlenecks through controlled, reproducible experiments rather than assuming that simulation slowdown and loss of robot control have the same cause.

## Simulation Environment

| Component | Configuration |
|---|---|
| Host OS | Ubuntu 24.04 LTS |
| Containerization | Docker |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Robot platform | Clearpath Jackal |
| Environment | Clearpath Warehouse |
| GPU | NVIDIA GeForce RTX 3060 (12 GB) |
| Simulation architecture | Single Gazebo server with multiple AMRs |

The simulation software will run inside Docker to minimize host dependencies and improve reproducibility.

## Experimental Methodology

The investigation will proceed incrementally.

### Phase 1 — Environment Validation

- Build and launch the Docker environment.
- Simulate one Jackal in the warehouse.
- Verify robot movement and command execution.
- Validate performance measurement methods.

### Phase 2 — Incremental Scaling

Increase the number of simulated robots while measuring:

- Real-time factor (RTF).
- CPU and memory utilization.
- GPU utilization where applicable.
- Simulation stability.
- Robot-command responsiveness.

Each configuration will be tested repeatedly under controlled conditions.

### Phase 3 — Bottleneck Investigation

Investigate observed performance degradation by isolating potential causes, including:

- Physics simulation.
- Sensor simulation.
- Robot-control execution.
- ROS 2 communication.

### Phase 4 — Analysis and Conclusions

Compare experimental results, identify supported explanations for observed failures, and evaluate potential improvements.

## Reproducibility

The repository will provide:

- Version-controlled simulation configurations.
- A Dockerfile and Docker Compose configuration.
- A published Docker image.
- Experiment scripts and measurement procedures.
- Results and visualizations.
- Documented limitations.

## Upstream Project

This investigation uses the official [Clearpath Simulator](https://github.com/clearpathrobotics/clearpath_simulator), maintained by Clearpath Robotics.

The simulator is included as a Git submodule to preserve the upstream source and reference a reproducible version.

## Project Status

**Milestone 0 — Repository initialization**

The experimental environment has not yet been built or benchmarked.

Results and conclusions will be added as experiments are completed.