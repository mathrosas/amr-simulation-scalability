# System Architecture

**Project:** AMR Simulation Scalability and Control Reliability  
**Architecture baseline:** November 20, 2026  
**ROS distribution:** ROS 2 Jazzy  
**Simulator:** Gazebo Harmonic  
**Robot platform:** Clearpath Jackal J100

## 1. Architecture Overview

The project investigates the scalability, performance, and control reliability of autonomous mobile robot simulations.

The system is designed to support incremental fleet scaling while separating simulation, localization, navigation, visualization, and fleet coordination.

The architecture currently supports two independently controlled and navigated robots operating in a shared Gazebo environment.

Open-RMF integration is planned as the next architectural milestone.

### Design Principles

The implementation follows these principles:

1. Separate major system components into independent Docker services.
2. Maintain a common fleet configuration.
3. Assign each robot a unique ROS 2 namespace.
4. Preserve independent localization and navigation interfaces.
5. Reuse the same robot model and baseline configuration.
6. Separate simulation performance from navigation and communication performance.
7. Collect structured experimental measurements.
8. Support reproducible experiments.
9. Extend the architecture without redesigning it for each additional robot.
10. Introduce centralized fleet coordination through Open-RMF.

## 2. Current System Architecture

The validated two-robot architecture is:

```text
                  HOST — UBUNTU 24.04
                           |
                      DOCKER COMPOSE
                           |
        +------------------+------------------+
        |                  |                  |
        v                  v                  v
  FLEET SIMULATION  FLEET LOCALIZATION  FLEET NAVIGATION
        |                  |                  |
        |            +-----+-----+      +-----+-----+
        |            |           |      |           |
        |            v           v      v           v
        |          AMCL 0      AMCL 1  NAV2 0      NAV2 1
        |            |           |      |           |
        |            +-----------+------+-----------+
        |                         |
        |                    ROS 2 / DDS
        |                         |
        v                         |
     GAZEBO HARMONIC <------------+
        |
        +-- Warehouse World
        |
        +-- Jackal j100_0000
        |     +-- Physics
        |     +-- Sensors
        |     +-- Controllers
        |
        +-- Jackal j100_0001
              +-- Physics
              +-- Sensors
              +-- Controllers


                OPTIONAL VISUALIZATION
                       |
                      RViz2
```

Both robots share one Gazebo simulation environment.

Each robot has independent ROS 2 interfaces, localization, and navigation.

The current architecture does not include centralized fleet task allocation or traffic coordination.

## 3. Docker Architecture

The project uses Docker Compose to separate major functionality into independently managed services.

### Core Fleet Services

| Service | Container | Responsibility |
|---|---|---|
| `fleet_simulation` | `amr-scalability-fleet-simulation` | Gazebo, robot simulation, sensors, and controllers |
| `fleet_localization` | `amr-scalability-fleet-localization` | Robot-specific map servers and AMCL |
| `fleet_navigation` | `amr-scalability-fleet-navigation` | Independent Nav2 stacks |

An additional visualization service provides RViz2 when graphical inspection is required.

### Docker Compose Files

The architecture is distributed across:

```text
compose.yaml
compose.visualization.yaml
compose.navigation.yaml
compose.fleet.yaml
```

The fleet services extend the existing single-robot service definitions.

This allows the single-robot implementation and the multi-robot implementation to reuse common infrastructure.

### Container Communication

The validated services use:

```yaml
network_mode: host
ipc: host
```

ROS 2 communication uses DDS.

The current environment uses:

```text
ROS_DOMAIN_ID=0
```

Host networking allows the containers to communicate using the same ROS 2 network environment.

Host IPC is enabled for the existing container architecture.

### Docker Images

The simulation environment is based on the Clearpath Robotics Docker image pinned to:

```text
sha256:970015d533d52c816d1d59ec10cfc2d5df20977b8d8d34c89c67f58ee6543315
```

The custom navigation image is:

```text
amr-scalability-nav2:jazzy
```

Its Dockerfile is located at:

```text
docker/nav2/Dockerfile
```

The navigation image installs the ROS 2 Jazzy Nav2 packages and Clearpath navigation demonstrations.

The Clearpath base image is digest-pinned.

Additional ROS packages installed through APT are not currently pinned to exact package versions.

Therefore, future image rebuilds may resolve different package versions unless the environment is further locked down.

## 4. Fleet Configuration

The central fleet definition is:

```text
config/fleet/fleet.yaml
```

The validated configuration contains:

```yaml
world: warehouse

robots:
  - namespace: j100_0000
    setup_path: j100_0000

    spawn:
      x: 0.0
      y: 0.0
      z: 0.3
      yaw: 0.0

  - namespace: j100_0001
    setup_path: j100_0001

    spawn:
      x: 1.0
      y: 0.0
      z: 0.3
      yaw: 0.0
```

Each robot has:

- A unique namespace.
- A dedicated Clearpath setup directory.
- A configured initial position.
- A configured initial orientation.

Robot-specific configuration directories are located under:

```text
config/fleet/j100_0000/
config/fleet/j100_0001/
```

Clearpath-generated platform, sensor, manipulator, and robot-description files are excluded from Git.

These generated artifacts must be recreated as part of environment initialization.

### Fleet Scaling

The fleet definition is intended to support additional robots.

Planned experimental fleet sizes are:

```text
1 → 2 → 3 → 5 → 10
```

Additional robot configurations must include valid namespaces, setup directories, and spawn positions.

The architecture should scale by extending the fleet configuration rather than manually duplicating entire Docker services.

This is an architectural objective; automatic scaling beyond two robots has not yet been validated.

## 5. Simulation Layer

The simulation service launches a shared Gazebo warehouse world.

The fleet simulation launch file is:

```text
launch/fleet_simulation.launch.py
```

The service receives the fleet configuration using:

```text
fleet_config:=/etc/clearpath/fleet.yaml
```

The simulation layer is responsible for:

- Loading the warehouse environment.
- Spawning simulated robots.
- Simulating robot dynamics.
- Executing robot controllers.
- Publishing simulated sensor data.
- Publishing robot odometry.
- Exposing ROS 2 control interfaces.

The two-robot implementation has been validated.

## 6. Localization Layer

The localization layer uses AMCL and a static map server for each robot.

The fleet localization launcher is:

```text
launch/fleet_localization.launch.py
```

Each robot has an independent localization namespace.

The validated namespaces are:

```text
/j100_0000
/j100_0001
```

### Coordinate Transformations

Each robot uses a namespaced transformation hierarchy:

```text
map
 |
 v
odom
 |
 v
base_link
```

AMCL provides localization relative to the map.

Odometry provides the robot's estimated motion within its local reference frame.

The complete transformation chain is required for autonomous navigation.

### Initial Pose

AMCL requires an initial pose estimate.

The current implementation provides fleet initialization through:

```text
scripts/initialize_fleet.py
```

The script initializes the robots using the configured starting positions and validates the resulting localization estimates.

The initialization procedure has been validated for two robots.

## 7. Navigation Layer

Each robot runs an independent Nav2 stack.

The fleet navigation launcher is:

```text
launch/fleet_navigation.launch.py
```

This launcher reads the central fleet configuration and creates a Nav2 launch process for each robot.

The validated configuration uses:

```text
use_sim_time:=true
slam:=false
autostart:=false
use_composition:=false
```

Automatic Nav2 lifecycle activation is disabled so that startup can be controlled and validated explicitly.

### Navigation Configuration

The common navigation parameters are stored in:

```text
config/navigation/j100/nav2.yaml
```

The configuration is mounted at the path expected by the Clearpath navigation launcher.

The existing configuration uses:

- NavFn global planning.
- MPPI local control.
- Namespaced odometry.
- Nav2 lifecycle management.
- Independent navigation actions.

The controller odometry topic is configured as:

```text
platform/odom/filtered
```

### Controlled Lifecycle Startup

The project provides:

```text
scripts/start_nav2_controlled.py
```

This script checks the simulation clock, localization state, required transformations, and initial navigation lifecycle states.

It then requests Nav2 lifecycle startup and verifies that the navigation nodes become active.

The validated fleet contains 10 navigation lifecycle nodes per robot.

Six independent robot startup attempts succeeded across the three two-robot navigation experiments.

## 8. Simultaneous Navigation

The current fleet supports simultaneous navigation using independent Nav2 actions.

The navigation benchmark is implemented in:

```text
scripts/simultaneous_navigation_probe.py
```

The probe:

1. Checks navigation readiness.
2. Requests a global path for each robot.
3. Dispatches navigation goals.
4. Records navigation action results.
5. Collects timing and robot-state measurements.
6. Records recovery behavior.
7. Produces structured experimental results.

Three independent two-robot navigation experiments were completed.

All six navigation actions succeeded.

### Important Limitation

Simultaneous Nav2 navigation is not equivalent to centralized fleet management.

The current implementation does not provide:

- Centralized task allocation.
- RMF-managed route scheduling.
- Fleet-level traffic negotiation.
- RMF mission monitoring.
- Centralized multi-robot dispatch.

These capabilities belong to the planned Open-RMF integration.

## 9. Experimental Measurement Architecture

The project records measurements at different system layers.

### Simulation

Examples include:

- Real-time factor.
- CPU utilization.
- GPU utilization.
- Memory utilization.
- Sensor publication rates.

### Communication and Control

Examples include:

- ROS 2 communication performance.
- Command responsiveness.
- Controller behavior.
- Odometry.
- Concurrent robot operation.

### Navigation

Examples include:

- Planning success.
- Navigation action success.
- Navigation duration.
- Fleet makespan.
- Dispatch timing.
- Action overlap.
- Recovery behavior.
- Final robot positions.

### Result Storage

Experiment results are stored under:

```text
results/raw/
```

The validated two-robot navigation experiments are:

```text
fleet_simnav_controlled_execute_2robots_20261120_01/

fleet_simnav_repeat_2robots_20261120_trial_01/

fleet_simnav_repeat_2robots_20261120_trial_02/
```

Detailed results and methodology are documented in:

[Two-Robot Baseline](baseline-2-robots.md).

The existing direct-Nav2 experiments will serve as a comparison reference when Open-RMF is introduced.

## 10. Planned Open-RMF Architecture

**Status: Not implemented.**

Open-RMF will introduce centralized fleet management above the existing navigation infrastructure.

The proposed architecture is:

```text
                  BENCHMARK RUNNER
                         |
                         v
                      OPEN-RMF
                         |
                  Task Allocation
                  Traffic Scheduling
                  Fleet Coordination
                         |
                         v
                    FLEET ADAPTER
                         |
                  ROS 2 Interfaces
                         |
             +-----------+-----------+
             |                       |
             v                       v
        Nav2 Robot 0            Nav2 Robot 1
             |                       |
             +-----------+-----------+
                         |
                         v
                  GAZEBO HARMONIC
```

### Planned Components

The Open-RMF implementation will introduce:

1. A Docker environment for Open-RMF.
2. An RMF-compatible warehouse navigation graph.
3. A fleet adapter managing the simulated robots.
4. Integration with each robot's Nav2 actions.
5. Robot state and execution feedback.
6. RMF-managed navigation tasks.
7. Fleet traffic coordination.
8. Structured fleet-management benchmarks.

### Fleet Adapter

The planned fleet adapter will connect Open-RMF to the existing robot navigation interfaces.

It should:

- Discover robots from the fleet configuration.
- Associate each robot with its ROS 2 namespace.
- Report robot state to Open-RMF.
- Translate RMF navigation assignments into Nav2 requests.
- Monitor execution.
- Report completion or failure.
- Handle cancellation and interruption appropriately.

The exact adapter implementation and interfaces remain to be designed and tested.

### Warehouse Navigation Graph

Open-RMF will require a graph describing permitted navigation routes and relevant traffic constraints.

This graph must be consistent with the warehouse environment and the robots' navigation capabilities.

It will be developed and validated separately from the existing Nav2 costmaps.

## 11. Reproducibility Requirements

The target environment should support:

1. Building the required container images.
2. Loading a specified fleet configuration.
3. Starting the simulation.
4. Initializing robot localization.
5. Activating Nav2.
6. Starting Open-RMF.
7. Registering all fleet robots.
8. Dispatching a predefined workload.
9. Collecting experimental measurements.
10. Stopping the environment and preserving results.

### Current Status

The existing two-robot environment has been validated through individually executed stages.

A single automated end-to-end workflow has not yet been implemented.

The automated workflow will be required before considering the Open-RMF two-robot baseline complete.

## 12. Future Scalability Experiments

Following Open-RMF integration, experiments will increase the robot count.

The planned sequence is:

```text
2 → 3 → 5 → 10
```

Each experiment should preserve a consistent methodology while recording the fleet size and configuration.

The investigation will examine:

- Simulation real-time factor.
- CPU, GPU, and memory utilization.
- ROS 2 communication behavior.
- Navigation execution performance.
- Fleet task completion.
- Traffic coordination.
- Reliability and failure modes.
- Performance changes with fleet size.

The objective is to identify measurable bottlenecks and distinguish their possible causes.

## 13. Architecture Status

| Component | Status |
|---|---|
| Gazebo warehouse simulation | Validated |
| Single-robot control | Validated |
| Two-robot simulation | Validated |
| Two-robot localization | Validated |
| Two-robot Nav2 navigation | Validated |
| Controlled navigation startup | Validated |
| Simultaneous navigation experiments | Completed |
| Open-RMF integration | Pending |
| Fleet adapter | Pending |
| RMF warehouse graph | Pending |
| RMF-managed navigation | Pending |
| End-to-end automated baseline | Pending |
| Scaling beyond two robots | Pending |

The current architecture provides the validated foundation for Open-RMF integration and subsequent fleet scalability experiments.