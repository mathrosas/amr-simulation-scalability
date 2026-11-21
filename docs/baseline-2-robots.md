# Two-Robot Baseline — Simulation and Autonomous Navigation

**Project:** AMR Simulation Scalability and Control Reliability  
**Validation date:** November 20, 2026  
**Status:** Validated — Nav2-only baseline  
**Fleet size:** 2 Clearpath Jackal J100 robots  
**Fleet management:** Open-RMF integration pending

## 1. Objective

Establish a reproducible two-robot simulation and autonomous navigation baseline before introducing centralized fleet coordination through Open-RMF.

The baseline validates:

- Simultaneous simulation of two independent mobile robots.
- Independent ROS 2 namespaces and communication interfaces.
- Independent robot control and odometry.
- Independent localization using AMCL.
- Independent Nav2 navigation stacks.
- Controlled Nav2 lifecycle activation.
- Simultaneous navigation goal execution.
- Repeatability across multiple independent experiments.
- Preservation of experimental results for subsequent comparison.

The experiments provide a reference against which future Open-RMF-managed navigation and larger fleet configurations can be evaluated.

## 2. Experimental Environment

| Component | Configuration |
|---|---|
| Operating system | Ubuntu 24.04 LTS |
| ROS distribution | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Robot model | Clearpath Jackal J100 |
| Simulation world | Warehouse |
| Navigation | Nav2 |
| Localization | AMCL |
| Global planner | NavFn |
| Local controller | MPPI |
| ROS middleware | Fast DDS |
| Containerization | Docker Compose |
| CPU | AMD Ryzen 5 7600 |
| Memory | 32 GB DDR5 |
| GPU | NVIDIA RTX 3060 12 GB |

### Container architecture

The fleet implementation uses three principal services:

| Service | Responsibility |
|---|---|
| `fleet_simulation` | Gazebo simulation, robots, controllers, and sensors |
| `fleet_localization` | Map servers and AMCL instances |
| `fleet_navigation` | Independent Nav2 navigation stacks |

RViz2 is available separately for visualization.

The services communicate using ROS 2 with host networking and host IPC.

### Configuration files

The implementation uses:

```text
compose.yaml
compose.visualization.yaml
compose.navigation.yaml
compose.fleet.yaml

config/fleet/fleet.yaml
config/fleet/j100_0000/robot.yaml
config/fleet/j100_0001/robot.yaml

config/navigation/j100/nav2.yaml

launch/fleet_simulation.launch.py
launch/fleet_localization.launch.py
launch/fleet_navigation.launch.py
```

The fleet configuration is the central definition of robot membership, namespaces, setup paths, and initial positions.

The launch files iterate over this configuration to instantiate the corresponding robot components.

## 3. Fleet Configuration

The validated fleet consists of two robots.

| Parameter | Robot 1 | Robot 2 |
|---|---|---|
| Namespace | `j100_0000` | `j100_0001` |
| Initial X | 0.0 m | 1.0 m |
| Initial Y | 0.0 m | 0.0 m |
| Initial Z | 0.3 m | 0.3 m |
| Initial yaw | 0.0 rad | 0.0 rad |

Both robots operate in the same Gazebo world.

Each robot has independent:

- ROS 2 namespaced interfaces.
- Odometry.
- Sensor data.
- AMCL localization.
- Nav2 action servers.
- Navigation lifecycle nodes.

Both use the same underlying robot model and navigation configuration.

## 4. Experimental Procedure

Three independent two-robot navigation executions were performed:

1. Initial controlled navigation baseline.
2. Repeatability Trial 1.
3. Repeatability Trial 2.

Each execution followed the same general procedure.

### Stage A — Simulation startup

Start the fleet simulation using the Docker Compose fleet configuration.

Verify that both robots have been spawned at their expected initial positions.

### Stage B — Localization

Start the fleet localization service.

Verify that each robot has an active map server and AMCL instance.

Initialize each robot's localization using its configured spawn pose.

Verify localization and the required transformations:

```text
map → odom → base_link
```

### Stage C — Controlled navigation startup

Start the fleet navigation service with automatic Nav2 activation disabled.

Activate each robot's Nav2 stack independently using:

```text
scripts/start_nav2_controlled.py
```

The startup procedure validates:

- An advancing simulation clock.
- Active localization.
- Valid coordinate transformations.
- Initial navigation lifecycle states.
- Successful lifecycle activation.
- Final active states of all navigation nodes.

Each robot has 10 navigation lifecycle nodes.

### Stage D — Simultaneous navigation

Execute:

```text
scripts/simultaneous_navigation_probe.py
```

The probe verifies navigation readiness, requests global plans, dispatches both navigation goals, and records execution results.

The requested goals remain unchanged between experiments.

| Parameter | Robot 1 | Robot 2 |
|---|---|---|
| Goal X | -2.0 m | 3.0 m |
| Goal Y | 0.0 m | 0.0 m |
| Goal yaw | π rad | 0.0 rad |
| Goal frame | `map` | `map` |

The navigation goal timeout is 120 seconds.

### Stage E — Result collection

Collect:

- Navigation action results.
- Navigation execution durations.
- Fleet makespan.
- Goal dispatch timing.
- Action overlap duration.
- Recovery counts.
- Estimated final robot positions.
- Final Gazebo ground-truth positions.
- Navigation logs.

### Stage F — Shutdown

Preserve the experimental results and stop the fleet services.

Each subsequent trial begins from a newly initialized simulation and navigation environment.

**Reproducibility status:** The individual stages have been validated. A single automated procedure for executing all stages from startup to result collection is not yet implemented.

## 5. Navigation Results

### Fleet performance

| Metric | Baseline | Trial 1 | Trial 2 |
|---|---:|---:|---:|
| Navigation actions succeeded | 2/2 | 2/2 | 2/2 |
| Fleet makespan (s) | 12.651 | 12.551 | 12.201 |
| Robot 1 duration (s) | 12.651 | 12.551 | 12.201 |
| Robot 2 duration (s) | 4.601 | 4.751 | 4.651 |
| Dispatch skew (ms) | 0.115 | 0.133 | 0.113 |
| Action overlap (s) | 4.601 | 4.751 | 4.651 |
| Recovery behaviors | 0 | 0 | 0 |
| Manual stop required | No | No | No |

### Aggregate results

| Metric | Result |
|---|---|
| Independent experiments | 3 |
| Total navigation actions | 6 |
| Successful navigation actions | 6 |
| Observed action success rate | 100% |
| Mean fleet makespan | 12.468 s |
| Total observed recovery behaviors | 0 |
| Manual intervention during navigation | None required |

All six navigation actions completed successfully.

No navigation recovery behaviors were reported.

Fleet makespan remained between approximately 12.2 and 12.7 seconds across the three experiments.

These measurements describe a small experimental sample and should not be interpreted as statistical reliability guarantees.

## 6. Controlled Nav2 Startup Results

Each experiment required independent lifecycle activation of both navigation stacks.

| Experiment | Robot 1 startup | Robot 2 startup |
|---|---:|---:|
| Baseline | 10.74 s | 9.45 s |
| Trial 1 | 11.13 s | 11.35 s |
| Trial 2 | 8.18 s | 8.99 s |

All six controlled lifecycle activations succeeded.

Each activation verified that all 10 navigation lifecycle nodes reached the active state.

A total of 20 navigation lifecycle nodes were active during the two-robot navigation experiments.

## 7. Ground-Truth Positioning Measurements

Final robot positions were independently obtained from Gazebo after navigation.

| Experiment | Robot 1 error | Robot 2 error |
|---|---:|---:|
| Baseline | 0.220 m | 0.154 m |
| Trial 1 | 0.260 m | 0.111 m |
| Trial 2 | 0.302 m | 0.140 m |

These measurements are retained as additional experimental observations.

They are distinct from the Nav2 navigation action results.

Nav2 goal completion is evaluated using robot state estimates and configured goal-checker tolerances, while the measurements above use Gazebo ground truth.

The configured XY goal tolerance is 0.25 m.

Four of the six recorded final ground-truth positions fell within 0.25 m of the requested goal.

No additional positioning-accuracy tuning was performed during these experiments.

## 8. Experimental Artifacts

The three navigation experiments are preserved under:

```text
results/raw/fleet_simnav_controlled_execute_2robots_20261120_01/

results/raw/fleet_simnav_repeat_2robots_20261120_trial_01/

results/raw/fleet_simnav_repeat_2robots_20261120_trial_02/
```

The recorded artifacts include structured navigation summaries, per-robot results, and supporting measurements.

The repeatability trial directories also contain initialization data, final Gazebo ground-truth measurements, and navigation logs.

The primary fleet-level summary file is:

```text
fleet_summary.json
```

These directories should remain unchanged when developing the Open-RMF integration.

The experimental results are intended to serve as a reference dataset for later comparisons.

## 9. Measurement Limitations

The current benchmark has several limitations.

### Navigation goal dispatch

Goals are dispatched independently through namespaced Nav2 action interfaces.

The reported dispatch skew measures the time between client dispatch calls, not the precise timing of DDS delivery or physical robot movement.

### Simultaneous execution

The recorded action overlap indicates overlapping navigation action intervals.

It does not independently establish the exact duration of simultaneous physical robot motion.

### Localization

Navigation operates using estimated robot poses.

Ground-truth measurements are collected separately.

### Ground truth

The current experiments use final Gazebo state measurements rather than continuous independent ground-truth trajectory recording.

### Fleet coordination

No centralized fleet-management or traffic-scheduling framework was active.

Although both robots navigated simultaneously, Open-RMF did not assign their tasks or coordinate their routes.

### Benchmark automation

Simulation startup, localization initialization, controlled navigation activation, and result collection have been validated individually.

Complete end-to-end experiment orchestration remains to be implemented.

## 10. Baseline Conclusion

The two-robot Nav2-only implementation successfully demonstrated:

- Simultaneous operation of two simulated Jackal robots.
- Independent localization and navigation.
- Successful controlled Nav2 startup.
- Reliable completion of the tested simultaneous navigation actions.
- Repeatable navigation measurements.
- Preservation of structured experimental evidence.

All six navigation actions succeeded across three independent experiments.

The results establish a functional two-robot navigation reference, but they do not yet establish Open-RMF-managed fleet operation.

## 11. Next Development Phase — Open-RMF

The next phase will integrate Open-RMF into the existing architecture.

The planned additions are:

1. An Open-RMF Docker environment compatible with ROS 2 Jazzy.
2. A warehouse navigation graph.
3. A fleet adapter connecting Open-RMF to the existing Nav2 interfaces.
4. Robot registration and state reporting.
5. RMF-managed task execution.
6. Fleet-level traffic coordination.
7. Automated experiment startup and shutdown.
8. Reproducible result collection.

The objective is to establish a fully reproducible two-robot Open-RMF baseline.

After validation, the same architecture will support incremental experiments with 3, 5, and 10 robots.

The existing Nav2-only results will remain available for comparison.