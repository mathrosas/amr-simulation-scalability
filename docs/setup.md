# Reproducing the Two-Robot Navigation Baseline

**Project:** AMR Simulation Scalability and Control Reliability  
**Baseline:** Two Clearpath Jackal J100 robots  
**Validated:** November 20, 2026  
**Status:** Manual procedure validated; end-to-end automation pending

## 1. Purpose

This guide documents the procedure used to reproduce the validated two-robot navigation experiments.

The environment consists of:

- One Gazebo Harmonic warehouse simulation.
- Two Clearpath Jackal J100 robots.
- Independent ROS 2 namespaces.
- Independent AMCL localization.
- Independent Nav2 navigation stacks.
- Controlled Nav2 lifecycle activation.
- Simultaneous navigation experiments.
- Structured experimental result collection.

Open-RMF is not part of this baseline.

The procedure is intended to preserve a reference implementation before Open-RMF fleet management and larger robot populations are introduced.

## 2. Prerequisites

The validated environment uses:

| Component | Configuration |
|---|---|
| Operating system | Ubuntu 24.04 LTS |
| ROS distribution | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Containerization | Docker Compose |
| Robot platform | Clearpath Jackal J100 |
| Middleware | Fast DDS |
| GPU | NVIDIA RTX 3060 |
| CPU | AMD Ryzen 5 7600 |
| Memory | 32 GB |

Docker Engine, Docker Compose, and the necessary NVIDIA container support must be installed and configured.

The host requires Python 3 for the startup and analysis scripts.

ROS 2 and Gazebo run inside Docker containers.

The following commands assume that the terminal is open in the repository root.

## 3. Project Configuration

The fleet is defined in:

```text
config/fleet/fleet.yaml
```

The validated configuration contains two robots:

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

Each robot has its own Clearpath configuration directory.

```text
config/fleet/j100_0000/
config/fleet/j100_0001/
```

The simulation launcher generates the additional Clearpath configuration files required for execution.

Generated configuration artifacts are excluded from Git and must not be treated as the authoritative source configuration.

## 4. Docker Compose Configuration

The project uses four Compose files for the two-robot environment:

```text
compose.yaml
compose.visualization.yaml
compose.navigation.yaml
compose.fleet.yaml
```

Define a Bash array for subsequent commands:

```bash
FC=(
  docker compose
  -f compose.yaml
  -f compose.visualization.yaml
  -f compose.navigation.yaml
  -f compose.fleet.yaml
)
```

The array is defined in the current terminal session and must be defined again when using a new shell.

Verify the combined Compose configuration:

```bash
"${FC[@]}" config --quiet
```

An exit status of zero indicates that Docker Compose accepted the configuration.

This does not independently verify that all services can start successfully.

## 5. Build the Navigation Image

The fleet navigation and localization services use the custom Nav2 image.

Build the fleet navigation image:

```bash
"${FC[@]}" build fleet_navigation
```

The Dockerfile is:

```text
docker/nav2/Dockerfile
```

The image is based on a digest-pinned Clearpath Robotics container image.

Additional ROS packages are installed through APT without exact package-version pins.

Consequently, rebuilding the image in the future may produce a different package environment.

For strict reproducibility, the installed package versions and the resulting image identity should also be recorded.

## 6. Prepare the Experiment

Each experiment must have a unique identifier.

Define:

```bash
RUN_ID="nav2_2robots_$(date -u +%Y%m%dT%H%M%SZ)"

RUN_DIR="results/raw/$RUN_ID"

INIT_ID="${RUN_ID}_initialization"

NAV_ID="${RUN_ID}_navigation"
```

Create the results directory:

```bash
mkdir -p "$RUN_DIR"
```

The experiment identifiers must remain defined in the same shell throughout the procedure.

The initialization and navigation scripts each require a separate, previously nonexistent output directory inside their respective containers.

Using unique identifiers prevents results from different experiments from being mixed.

## 7. Start the Fleet Simulation

Ensure that any previous fleet experiment has been stopped.

Start a fresh simulation container:

```bash
"${FC[@]}" up -d \
  --no-deps \
  --no-build \
  --force-recreate \
  fleet_simulation
```

The simulation launcher:

1. Reads the fleet configuration.
2. Validates robot namespaces.
3. Validates robot serial numbers.
4. Validates robot configuration paths.
5. Generates the required Clearpath files.
6. Launches the warehouse world.
7. Spawns both robots.

Wait until Gazebo and both robots have initialized.

Inspect the simulation logs if necessary:

```bash
docker logs --tail 50 \
  amr-scalability-fleet-simulation
```

### Verify Initial Robot Positions

Run:

```bash
"${FC[@]}" exec -T fleet_simulation bash -lc '
source /opt/ros/jazzy/setup.bash

for robot in j100_0000 j100_0001; do
    echo "===== $robot ====="
    gz model -m "$robot/robot" --pose 2>/dev/null
done
'
```

Expected horizontal starting positions:

| Robot | X | Y |
|---|---:|---:|
| `j100_0000` | 0.0 m | 0.0 m |
| `j100_0001` | 1.0 m | 0.0 m |

The configured spawn height is 0.3 m. After the robots settle, their Gazebo Z coordinates may differ from that value.

Do not continue if either robot is missing or incorrectly positioned.

## 8. Start Fleet Localization

Start the localization service:

```bash
"${FC[@]}" up -d \
  --no-deps \
  --no-build \
  --force-recreate \
  fleet_localization
```

The localization launcher creates an independent map server and AMCL instance for each robot.

### Verify Localization Lifecycle States

Run:

```bash
"${FC[@]}" exec -T fleet_localization bash -lc '
source /opt/ros/jazzy/setup.bash

for robot in j100_0000 j100_0001; do
    echo "===== $robot ====="

    for node in map_server amcl; do
        printf "%-20s " "$node"
        timeout 15s ros2 lifecycle get "/$robot/$node"
    done
done
'
```

Expected lifecycle state for all four nodes:

```text
active [3]
```

If a node is not active, inspect the localization logs and resolve the issue before proceeding.

## 9. Initialize Fleet Localization

The initialization script is:

```text
scripts/initialize_fleet.py
```

It reads the configured robot positions, initializes AMCL, and validates localization readiness.

### Copy the Script

Create a scripts directory inside the localization container:

```bash
"${FC[@]}" exec -T fleet_localization \
  mkdir -p /tmp/amr_scripts
```

Copy the script:

```bash
docker cp \
  scripts/initialize_fleet.py \
  amr-scalability-fleet-localization:/tmp/amr_scripts/initialize_fleet.py
```

### Execute Fleet Initialization

Run:

```bash
"${FC[@]}" exec -T fleet_localization bash -lc '
source /opt/ros/jazzy/setup.bash

exec python3 /tmp/amr_scripts/initialize_fleet.py \
  --fleet-config /etc/clearpath/fleet.yaml \
  --output-dir "$1"
' _ "/tmp/${INIT_ID}"
```

The initialization must succeed for both robots.

The validated initialization procedure checks localization lifecycle states and resulting robot localization estimates.

Do not proceed if initialization fails.

### Preserve Initialization Results

Copy the results to the host:

```bash
docker cp \
  "amr-scalability-fleet-localization:/tmp/${INIT_ID}/." \
  "$RUN_DIR/"
```

The initialization summary is saved alongside the experiment's other results.

## 10. Start Fleet Navigation

Start a fresh navigation container:

```bash
"${FC[@]}" up -d \
  --no-deps \
  --no-build \
  --force-recreate \
  fleet_navigation
```

The navigation launcher is:

```text
launch/fleet_navigation.launch.py
```

It creates one namespaced Nav2 stack per robot.

The validated configuration uses:

```text
use_sim_time:=true
slam:=false
autostart:=false
use_composition:=false
```

Nav2 automatic lifecycle activation is disabled.

Each robot must be activated using the controlled startup script.

## 11. Activate Both Nav2 Stacks

The controlled startup script is:

```text
scripts/start_nav2_controlled.py
```

### Activate Robot 1

```bash
python3 scripts/start_nav2_controlled.py \
  --namespace j100_0000 \
  --service fleet_navigation \
  --compose-file compose.yaml \
  --compose-file compose.visualization.yaml \
  --compose-file compose.navigation.yaml \
  --compose-file compose.fleet.yaml
```

Expected final result:

```text
NAV2 STARTUP PASSED
Navigation nodes: 10 active
```

### Activate Robot 2

```bash
python3 scripts/start_nav2_controlled.py \
  --namespace j100_0001 \
  --service fleet_navigation \
  --compose-file compose.yaml \
  --compose-file compose.visualization.yaml \
  --compose-file compose.navigation.yaml \
  --compose-file compose.fleet.yaml
```

Expected final result:

```text
NAV2 STARTUP PASSED
Navigation nodes: 10 active
```

Both robots must complete controlled startup successfully.

The validated two-robot environment contains 20 active Nav2 lifecycle nodes.

### Verify Both Robots

```bash
"${FC[@]}" exec -T fleet_navigation bash -lc '
source /opt/ros/jazzy/setup.bash

for robot in j100_0000 j100_0001; do
    echo "===== $robot ====="

    for node in controller_server planner_server bt_navigator; do
        printf "%-24s " "$node"
        timeout 15s ros2 lifecycle get "/$robot/$node"
    done
done
'
```

All six checked nodes should report:

```text
active [3]
```

## 12. Prepare Navigation Benchmark Scripts

The simultaneous navigation benchmark depends on three scripts:

```text
scripts/navigation_probe.py
scripts/fleet_navigation_probe.py
scripts/simultaneous_navigation_probe.py
```

Create the destination directory:

```bash
"${FC[@]}" exec -T fleet_navigation \
  mkdir -p /tmp/amr_scripts
```

Copy the scripts:

```bash
for file in \
  navigation_probe.py \
  fleet_navigation_probe.py \
  simultaneous_navigation_probe.py
do
  docker cp \
    "scripts/$file" \
    "amr-scalability-fleet-navigation:/tmp/amr_scripts/$file"
done
```

The scripts are copied into the container because the current navigation service does not mount the host's scripts directory.

They must be copied again whenever the navigation container is recreated.

## 13. Execute the Simultaneous Navigation Benchmark

The validated experiment uses these goals:

| Robot | Goal X | Goal Y | Goal yaw |
|---|---:|---:|---:|
| `j100_0000` | -2.0 m | 0.0 m | π rad |
| `j100_0001` | 3.0 m | 0.0 m | 0.0 rad |

Execute:

```bash
"${FC[@]}" exec -T fleet_navigation bash -lc '
source /opt/ros/jazzy/setup.bash

exec python3 \
  /tmp/amr_scripts/simultaneous_navigation_probe.py \
  "$@"
' _ \
  --goal j100_0000 -2.0 0.0 3.141592653589793 \
  --goal j100_0001 3.0 0.0 0.0 \
  --goal-timeout 120.0 \
  --output-dir "/tmp/$NAV_ID" \
  --execute
```

This command sends movement goals to both robots.

The probe:

1. Waits for the required ROS 2 interfaces.
2. Checks navigation readiness.
3. Requests a global plan for each robot.
4. Dispatches both navigation goals.
5. Monitors their execution.
6. Records timing and navigation results.
7. Saves structured experimental data.

### Success Criteria

For the validated navigation baseline:

- Both global planning requests must succeed.
- Both navigation actions must report `SUCCEEDED`.
- The fleet summary must report `simultaneous_navigation_passed: true`.
- The fleet summary must report `requires_manual_stop: false`.
- No execution error must be reported.

The benchmark also records recovery counts, positioning measurements, and timing information.

If a navigation action fails or the probe requests a manual stop, investigate the condition before starting another experiment.

## 14. Preserve Navigation Results

Copy the navigation results to the host:

```bash
docker cp \
  "amr-scalability-fleet-navigation:/tmp/${NAV_ID}/." \
  "$RUN_DIR/"
```

The fleet summary is:

```text
fleet_summary.json
```

### Verify the Summary

```bash
python3 -c '
import json
import sys

with open(sys.argv[1]) as f:
    data = json.load(f)

print("Robots:", data["robot_count"])
print("Planning:", data["fleet_planning_passed"])
print("Navigation:", data["simultaneous_navigation_passed"])
print("Makespan:", data["fleet_makespan_s"])
print("Dispatch skew:", data["dispatch_skew_s"])
print("Manual stop:", data["requires_manual_stop"])
print("Error:", data["error"])
' "$RUN_DIR/fleet_summary.json"
```

Verify that the reported results match the navigation probe's terminal output.

## 15. Record Final Gazebo Ground Truth

Record the final positions of both robots:

```bash
"${FC[@]}" exec -T fleet_simulation bash -lc '
source /opt/ros/jazzy/setup.bash

for robot in j100_0000 j100_0001; do
    echo "===== $robot ====="
    gz model -m "$robot/robot" --pose 2>/dev/null
done
' | tee "$RUN_DIR/gazebo_ground_truth.txt"
```

These measurements provide final Gazebo positions independently of the Nav2 localization estimates.

They are retained as experimental observations.

Ground-truth positioning accuracy and Nav2 action success must be reported as distinct metrics.

## 16. Preserve Container Logs

Before stopping the containers, save their logs.

### Simulation

```bash
docker logs amr-scalability-fleet-simulation \
  > "$RUN_DIR/fleet_simulation.log" 2>&1
```

### Localization

```bash
docker logs amr-scalability-fleet-localization \
  > "$RUN_DIR/fleet_localization.log" 2>&1
```

### Navigation

```bash
docker logs amr-scalability-fleet-navigation \
  > "$RUN_DIR/fleet_navigation.log" 2>&1
```

These logs can support subsequent investigation of startup problems, lifecycle transitions, warnings, or navigation failures.

## 17. Stop the Fleet

Stop the three fleet services:

```bash
"${FC[@]}" stop -t 30 \
  fleet_navigation \
  fleet_localization \
  fleet_simulation
```

Verify that no containers remain running:

```bash
docker ps --format "table {{.Names}}\t{{.Status}}"
```

The expected result for this project is an empty list of running containers, assuming no unrelated containers are active.

Stopping the containers preserves their filesystems, but the next reproducibility trial uses container recreation to start a new environment.

## 18. Repeat the Experiment

For another independent experiment:

1. Confirm that the previous fleet has stopped.
2. Generate a new experiment identifier.
3. Create a new results directory.
4. Recreate the simulation container.
5. Verify the initial robot positions.
6. Recreate the localization container.
7. Initialize localization.
8. Recreate the navigation container.
9. Activate both Nav2 stacks.
10. Execute the same navigation goals.
11. Preserve results and logs.
12. Stop the environment.

The fleet configuration, navigation parameters, and goals should remain unchanged when evaluating repeatability.

Any intentional changes must be documented as separate experimental conditions.

### Important Reset Considerations

A new experiment must not reuse a previous navigation endpoint as its starting position.

Both robots must begin from their configured initial poses.

The localization initialization process must correspond to those physical starting positions.

Each benchmark must use new output directories.

## 19. Validated Reference Results

Three independent two-robot navigation experiments were completed on November 20, 2026.

| Metric | Baseline | Trial 1 | Trial 2 |
|---|---:|---:|---:|
| Successful navigation actions | 2/2 | 2/2 | 2/2 |
| Fleet makespan (s) | 12.651 | 12.551 | 12.201 |
| Robot 1 duration (s) | 12.651 | 12.551 | 12.201 |
| Robot 2 duration (s) | 4.601 | 4.751 | 4.651 |
| Recovery behaviors | 0 | 0 | 0 |

The mean fleet makespan was approximately 12.468 seconds.

All six navigation actions succeeded.

The complete experiment results are documented in:

[Two-Robot Baseline](baseline-2-robots.md).

The original result directories are preserved under:

```text
results/raw/fleet_simnav_controlled_execute_2robots_20261120_01/

results/raw/fleet_simnav_repeat_2robots_20261120_trial_01/

results/raw/fleet_simnav_repeat_2robots_20261120_trial_02/
```

## 20. Current Reproducibility Limitations

The individual startup, initialization, navigation, and result-collection stages have been validated.

However, the following work remains:

- Automate the complete experiment lifecycle.
- Eliminate manual copying of scripts into containers.
- Capture complete software package and container image versions.
- Record machine-readable experiment metadata.
- Introduce automated experiment configuration validation.
- Add Open-RMF fleet management.
- Integrate fleet-level task dispatch and monitoring.
- Support fleet-size changes without manually repeating setup steps.

These are development requirements, not failures of the validated navigation baseline.

## 21. Next Phase: Open-RMF

The next development phase introduces Open-RMF above the existing Nav2 infrastructure.

The planned implementation will include:

- An Open-RMF Docker environment.
- A warehouse navigation graph.
- A fleet adapter integrating RMF with the robots' Nav2 interfaces.
- Centralized task dispatch.
- Fleet traffic coordination.
- Robot-state reporting.
- Automated experiment execution.

The existing two-robot Nav2-only benchmark will remain unchanged.

Once an Open-RMF-managed two-robot baseline is validated, the architecture will be extended to 3, 5, and 10 robots.

The objective is to compare fleet-management and simulation performance under repeatable, controlled workloads.