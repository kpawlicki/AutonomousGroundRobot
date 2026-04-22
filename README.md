# Autonomous Ground Robot

A modular, open-source Python software stack for autonomous ground robot navigation. Install it on any standardised differential-drive platform and deploy in the field with GPS waypoint following, multi-sensor obstacle avoidance, and optional carrier-agnostic ground station telemetry — all without needing physical hardware (full simulation mode included).

---

## Table of Contents

1. [Features](#features)
2. [System Architecture](#system-architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Repository Layout](#repository-layout)
5. [Installation](#installation)
6. [Quick Start](#quick-start)
7. [Configuration Reference](#configuration-reference)
8. [Python API](#python-api)
9. [Navigation Algorithms](#navigation-algorithms)
10. [Sensors](#sensors)
11. [Communication & Ground Station Protocol](#communication--ground-station-protocol)
12. [Motor Control](#motor-control)
13. [Running Tests](#running-tests)
14. [Extending the Stack](#extending-the-stack)
15. [License](#license)

---

## Features

| Category | Capability |
|---|---|
| **Sensors** | GPS (NMEA/serial), 2-D LiDAR (RPLidar), Intel RealSense depth camera, HC-SR04 ultrasonic |
| **Navigation** | A\*, Dijkstra, Artificial Potential Fields; binary occupancy grid fused from all sensors |
| **Communication** | Carrier-agnostic ground station link – LTE (TCP), LoRa (UART), Radio (serial/SiK) |
| **Ground station protocol** | Binary framing + CRC-8 JSON messages: HEARTBEAT, TELEMETRY, COMMAND, STATUS, ACK/NACK |
| **Motor control** | Proportional heading controller → differential-drive PWM |
| **Configuration** | Single YAML file; every hardware subsystem has a simulation fallback |
| **Simulation** | Full software-in-the-loop mode — no hardware needed for development or testing |

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                     Robot (robot.py)                │
│                                                     │
│  ┌─────────────┐   ┌─────────────────────────────┐  │
│  │   Sensors   │   │    NavigationManager        │  │
│  │  GPS        │──▶│  sense → fuse → plan        │  │
│  │  LiDAR      │   │  → waypoint follow          │  │
│  │  DepthCam   │   │  → emergency stop           │  │
│  │  Ultrasonic │   └───────────┬─────────────────┘  │
│  └─────────────┘               │                    │
│                                ▼                    │
│  ┌─────────────┐   ┌─────────────────────────────┐  │
│  │  Comms      │   │    MotorController          │  │
│  │  LTE/LoRa/  │   │  heading Kp → PWM L/R      │  │
│  │  Radio      │   └─────────────────────────────┘  │
│  └─────────────┘                                    │
└─────────────────────────────────────────────────────┘
```

The **NavigationManager** runs a threaded loop that:
1. Reads all sensors
2. Fuses readings into a binary **occupancy grid**
3. Replans a path using the chosen algorithm (A\*, Dijkstra, or Potential Fields)
4. Selects the next waypoint along the path
5. Commands the **MotorController** with desired heading and speed
6. Triggers an **emergency stop** if any sensor reports an obstacle closer than `emergency_stop_distance`

All hardware drivers have a **simulation counterpart** that is activated automatically when no hardware port/pin is configured, enabling full development and testing on any laptop.

---

## Hardware Requirements

### Minimum (simulation only)
- Any machine running **Python 3.8+**

### Recommended field platform
| Component | Supported hardware |
|---|---|
| **Compute** | Raspberry Pi 4 / 5 (or any Linux SBC) |
| **GPS** | Any u-blox or NMEA-compatible receiver (UART/USB) |
| **LiDAR** | RPLidar A1/A2/A3/S1 (USB) |
| **Depth camera** | Intel RealSense D415 / D435 / D455 (USB 3) |
| **Ultrasonic** | HC-SR04 × N (GPIO trigger/echo) |
| **Motor driver** | Any PWM-capable H-bridge (GPIO BCM pins) |
| **Communication** | LTE modem (TCP), LoRa module (UART e.g. RAK811, E32), Radio modem (SiK, XBee, RFD900x) |

Optional Python packages for hardware (see [Installation](#installation)):
- `pyserial` — GPS, LiDAR, LoRa, Radio UART/USB serial
- `RPi.GPIO` — Raspberry Pi GPIO (ultrasonic trigger/echo, motor PWM)
- `pyrealsense2` — Intel RealSense SDK
- `rplidar-roboticia` — RPLidar driver

---

## Repository Layout

```
autonomous_ground_robot/        # Main Python package
│
├── sensors/                    # Hardware sensor interfaces
│   ├── base.py                 #   Abstract BaseSensor (context-manager lifecycle)
│   ├── gps.py                  #   GPS – NMEA GGA/RMC serial parser + simulation
│   ├── lidar.py                #   2-D LiDAR – RPLidar point cloud + simulation
│   ├── depth_camera.py         #   Depth camera – RealSense per-pixel depth + simulation
│   └── ultrasonic.py           #   HC-SR04 ultrasonic ranger – GPIO + simulation
│
├── navigation/                 # Path planning and navigation
│   ├── algorithms/
│   │   ├── astar.py            #   A* with octile heuristic (configurable weight)
│   │   ├── dijkstra.py         #   Dijkstra uniform-cost search
│   │   └── potential_fields.py #   Artificial Potential Fields (reactive, per-tick)
│   ├── path_planner.py         #   OccupancyGrid + PathPlanner façade
│   └── manager.py              #   NavigationManager – threaded sense→plan→command loop
│
├── communication/              # Ground station link
│   ├── base.py                 #   Abstract BaseComm carrier interface
│   ├── protocol.py             #   GroundStationProtocol – binary framing + CRC-8 + JSON
│   ├── lte.py                  #   LTE carrier – TCP socket
│   ├── lora.py                 #   LoRa carrier – UART transparent mode
│   └── radio.py                #   Radio carrier – serial (SiK / XBee / RFD900x)
│
├── control/
│   └── motor_controller.py     # Differential-drive proportional heading controller
│
├── utils/
│   ├── coordinates.py          # GPS ↔ local ENU frame conversions
│   └── logger.py               # Centralised logging helper
│
├── config/
│   └── default_config.yaml     # Default (simulation) configuration
│
└── robot.py                    # Top-level Robot orchestrator

main.py                         # CLI entry point  (also installed as `agr` console script)
tests/                          # Pytest test suite (108 tests)
requirements.txt                # Runtime + dev dependencies
setup.py                        # Package setup
```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/kpawlicki/AutonomousGroundRobot.git
cd AutonomousGroundRobot
```

### 2. Install the package

```bash
# Core runtime only (simulation works without any extras)
pip install pyyaml
pip install -e .

# Or install with development/test dependencies
pip install -e ".[dev]"
```

### 3. Install hardware drivers (target platform only)

```bash
pip install -e ".[hardware]"
# Installs: pyserial, RPi.GPIO, pyrealsense2, rplidar-roboticia
```

> **Tip:** The `[hardware]` extras are only needed when running on physical hardware. All subsystems fall back to simulation automatically when their hardware config keys (port, pin numbers, etc.) are absent.

---

## Quick Start

### Run in simulation (no hardware required)

```bash
python main.py
```

The robot starts, logs its simulated state every second, and runs until you press **Ctrl-C**.

### Navigate to a local waypoint (metres from start)

```bash
python main.py --goal-x 10 --goal-y 5
```

### Navigate to a GPS coordinate

```bash
python main.py --goal-lat 48.8600 --goal-lon 2.3550
```

### Choose a navigation algorithm

```bash
python main.py --algorithm dijkstra
python main.py --algorithm potential_fields
```

### Use a custom configuration file

```bash
python main.py --config /path/to/my_config.yaml
```

### Run for a fixed duration then exit

```bash
python main.py --goal-x 5 --goal-y 5 --duration 30
```

### All CLI options

```
usage: main.py [-h] [--config PATH] [--goal-lat DEG] [--goal-lon DEG]
               [--goal-x M] [--goal-y M]
               [--algorithm {astar,dijkstra,potential_fields}]
               [--duration SEC]

  --config PATH         Path to a YAML configuration file
  --goal-lat DEG        Goal latitude in degrees (WGS-84)
  --goal-lon DEG        Goal longitude in degrees (WGS-84)
  --goal-x M            Goal X in local ENU frame (metres, East)
  --goal-y M            Goal Y in local ENU frame (metres, North)
  --algorithm           Override the navigation algorithm
  --duration SEC        Run for this many seconds then exit
```

The `agr` console script (installed by `setup.py`) is an alias for `python main.py`:

```bash
agr --goal-lat 48.860 --goal-lon 2.355 --algorithm astar
```

---

## Configuration Reference

Copy `autonomous_ground_robot/config/default_config.yaml` and edit it for your platform. Pass it with `--config`.

```yaml
# ---------------------------------------------------------------------------
# GPS sensor
# ---------------------------------------------------------------------------
gps:
  port: /dev/ttyUSB0   # Uncomment for real hardware
  baud: 9600
  sim_latitude: 48.8566
  sim_longitude: 2.3522

# ---------------------------------------------------------------------------
# LiDAR sensor
# ---------------------------------------------------------------------------
lidar:
  port: /dev/ttyUSB1   # Uncomment for real hardware (RPLidar)
  num_points: 360
  max_range: 12.0              # metres
  obstacle_distance: 2.5       # simulated obstacle distance (m)
  obstacle_width_deg: 30.0

# ---------------------------------------------------------------------------
# Depth camera
# ---------------------------------------------------------------------------
depth_camera:
  width: 640
  height: 480
  fps: 30
  sim_obstacle_distance: 1.5   # metres
  sim_background_distance: 5.0

# ---------------------------------------------------------------------------
# Ultrasonic sensors  (list – one entry per sensor direction)
# ---------------------------------------------------------------------------
ultrasonic:
  - direction: front
    sim_distance: 3.0
    max_range: 4.0
  - direction: rear
    sim_distance: 3.0
    max_range: 4.0

# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------
navigation:
  algorithm: astar              # astar | dijkstra | potential_fields
  allow_diagonal: true
  astar_weight: 1.0             # >1 = faster but suboptimal (weighted A*)
  apf_attractive_gain: 1.0
  apf_repulsive_gain: 100.0
  apf_influence_radius: 5.0

  grid_resolution: 0.1          # metres per grid cell
  grid_width: 201               # cells (total map width = grid_width × resolution)
  grid_height: 201

  cruise_speed: 1.0             # m/s
  emergency_stop_distance: 0.3  # metres – triggers full stop

  update_interval: 0.1          # seconds between navigation loop iterations

# ---------------------------------------------------------------------------
# Motor controller
# ---------------------------------------------------------------------------
control:
  pwm_left_pin: 12              # BCM GPIO pin – omit for simulation
  pwm_right_pin: 13
  max_speed: 2.0                # m/s
  wheel_base: 0.3               # metres (distance between wheels)
  heading_kp: 1.2               # proportional heading gain
  pwm_frequency: 50             # Hz

# ---------------------------------------------------------------------------
# Communication  (comment out entire block to disable)
# ---------------------------------------------------------------------------
communication:
  carrier: lora                 # lte | lora | radio
  enabled: true
  telemetry_interval: 1.0       # seconds
  heartbeat_interval: 5.0

  lte:
    host: 127.0.0.1
    port: 5760
    timeout: 5.0

  lora:
    port: /dev/ttyS0            # Omit for simulation
    baud: 9600
    timeout: 2.0

  radio:
    port: /dev/ttyUSB2          # Omit for simulation
    baud: 57600
    timeout: 1.0
```

> **Simulation fallback rule:** Any sensor or carrier whose hardware key (`port`, `pwm_left_pin`, etc.) is absent or commented out will start in simulation mode automatically. You can run the entire stack on a laptop without any hardware attached.

---

## Python API

```python
from autonomous_ground_robot import Robot

# Use built-in defaults (simulation)
robot = Robot()

# Or load a custom config
robot = Robot("config/field.yaml")

robot.start()

# --- Navigation ---

# GPS waypoint (WGS-84 degrees)
robot.navigation.set_goal_gps(lat=48.860, lon=2.355)

# Local ENU waypoint (metres from starting position)
robot.navigation.set_goal_local(x=10.0, y=5.0)

# Query current state
print(robot.navigation.state)      # NavigationState.NAVIGATING
print(robot.navigation.position)   # LocalPoint(x=..., y=...)
print(robot.navigation.heading)    # degrees, 0 = North

robot.stop()
```

### Context manager usage

```python
with Robot("config/field.yaml") as robot:
    robot.navigation.set_goal_local(x=5.0, y=5.0)
    import time; time.sleep(10)
# robot.stop() is called automatically on exit
```

---

## Navigation Algorithms

All algorithms implement a common `BaseAlgorithm` interface and operate on the shared binary **OccupancyGrid**.

| Algorithm | Style | Best for |
|---|---|---|
| **A\*** (`astar`) | Graph search | Static or slowly-changing environments; shortest path |
| **Dijkstra** (`dijkstra`) | Uniform-cost search | Optimal baseline; guaranteed shortest path |
| **Potential Fields** (`potential_fields`) | Reactive | Dynamic environments; runs once per tick |

Switch algorithm at runtime via CLI (`--algorithm`) or config (`navigation.algorithm`), or at runtime with the API:

```python
robot.config["navigation"]["algorithm"] = "potential_fields"
```

### A\* tuning

```yaml
navigation:
  astar_weight: 1.0   # 1.0 = optimal A*; >1 = weighted (faster, suboptimal)
  allow_diagonal: true
```

### Potential Fields tuning

```yaml
navigation:
  apf_attractive_gain: 1.0     # pull towards goal
  apf_repulsive_gain: 100.0    # push away from obstacles
  apf_influence_radius: 5.0    # metres – obstacle repulsion radius
```

---

## Sensors

All sensors share the `BaseSensor` abstract interface and support Python context-manager lifecycle (`with sensor: ...`).

### GPS

Parses standard NMEA GGA and RMC sentences from a serial GPS receiver (u-blox, SiRF, etc.). Provides latitude, longitude, altitude, fix quality, and speed over ground. The internal coordinate helper converts GPS fixes to a local ENU (East-North-Up) frame anchored at the first valid fix.

```yaml
gps:
  port: /dev/ttyUSB0
  baud: 9600
```

### LiDAR (RPLidar)

Streams a 360° point cloud from an RPLidar A/S-series scanner. The NavigationManager projects the full scan into the occupancy grid and also exposes a `min_distance_in_sector(angle, width)` query for fast emergency-stop checking.

```yaml
lidar:
  port: /dev/ttyUSB1
  max_range: 12.0
```

### Depth Camera (Intel RealSense)

Reads per-pixel depth frames from a RealSense D4xx camera (D415/D435/D455). Provides region-of-interest helpers used by the NavigationManager to project obstacle pixels onto the occupancy grid.

```yaml
depth_camera:
  width: 640
  height: 480
  fps: 30
```

### Ultrasonic (HC-SR04)

Supports multiple HC-SR04 sensors on configurable GPIO trigger/echo pairs. Each sensor is assigned a direction label (`front`, `rear`, `left`, `right`) and its readings are used to mark directional threat zones in the occupancy grid and trigger emergency stops.

```yaml
ultrasonic:
  - direction: front
    trigger_pin: 23   # BCM GPIO – omit for simulation
    echo_pin: 24
    max_range: 4.0
```

---

## Communication & Ground Station Protocol

The communication layer is **carrier-agnostic**: swap LTE, LoRa, or Radio without changing any higher-level code.

### Carriers

| Carrier class | Transport | Typical hardware |
|---|---|---|
| `LteComm` | TCP socket | LTE modem, direct IP link |
| `LoraComm` | UART transparent mode | RAK811, E32, E22 |
| `RadioComm` | Serial | SiK 3DR Radio, XBee, RFD900x |

Select the carrier in config:

```yaml
communication:
  carrier: lora   # lte | lora | radio
  enabled: true
```

### Protocol (`GroundStationProtocol`)

Sits on top of any carrier. Each message is binary-framed with a CRC-8 integrity check and carries a JSON payload.

```
Frame format:
  [0xAA] [type:1] [length:2 LE] [json_payload:N] [crc8:1]
```

| Message type | Direction | Purpose |
|---|---|---|
| `HEARTBEAT` | both | Keepalive / link quality check |
| `TELEMETRY` | robot → GS | Position, heading, navigation state, speed |
| `COMMAND` | GS → robot | Navigation commands (see below) |
| `STATUS` | robot → GS | Ready, error, goal-reached notifications |
| `ACK` | both | Successful command acknowledgement |
| `NACK` | both | Failed command / parse error |

### Sending commands to the robot

```json
// Go to GPS coordinate
{"type": "goto_gps", "latitude": 48.860, "longitude": 2.355, "altitude": 0}

// Go to local ENU position (metres)
{"type": "goto_local", "x": 10.0, "y": 5.0}

// Emergency stop
{"type": "stop"}

// Switch navigation algorithm
{"type": "set_algorithm", "algorithm": "dijkstra"}
```

---

## Motor Control

`MotorController` implements a **proportional heading controller** that converts a `(desired_heading_deg, speed_m_s)` command into left/right PWM duty cycles for a differential-drive platform.

```
error = desired_heading - current_heading   (wrapped to ±180°)
correction = Kp × error
left_duty  = base_duty + correction
right_duty = base_duty - correction
```

PWM is generated via `RPi.GPIO` on real hardware or logged as simulation commands when no GPIO pins are configured.

Key config parameters:

```yaml
control:
  wheel_base: 0.3     # metres
  heading_kp: 1.2     # increase for sharper turns; decrease to reduce oscillation
  max_speed: 2.0      # m/s
  pwm_frequency: 50   # Hz
```

---

## Running Tests

```bash
# Install dev dependencies
pip install pytest pytest-cov
pip install -e .

# Run the full test suite
python -m pytest tests/ -v

# With coverage report
python -m pytest tests/ -v --cov=autonomous_ground_robot --cov-report=term-missing
```

The test suite covers:
- Sensor simulation and NMEA parsing (GPS)
- All three navigation algorithms on known grids
- Occupancy grid fusion operations
- Ground station protocol encode/decode/CRC
- Each communication carrier (send / inject-receive helpers)
- Motor controller heading math
- Full robot start / stop lifecycle

---

## Extending the Stack

### Adding a new communication carrier

1. Create `autonomous_ground_robot/communication/my_carrier.py`
2. Subclass `BaseComm` and implement `connect()`, `disconnect()`, `send_bytes()` (and optionally `receive_bytes()`)
3. Export it from `communication/__init__.py`
4. Register the carrier name in `robot.py` → `_build_protocol()`

### Adding a new navigation algorithm

1. Create `autonomous_ground_robot/navigation/algorithms/my_algo.py`
2. Subclass `BaseAlgorithm` and implement `find_path(grid, start, goal) → AlgorithmResult`
3. Export it from `algorithms/__init__.py`
4. Register the name in `navigation/manager.py` → `_build_algorithm()`

### Adding a new sensor

1. Create `autonomous_ground_robot/sensors/my_sensor.py`
2. Subclass `BaseSensor` and implement `open()`, `close()`, `read()`
3. Instantiate it in `robot.py` → `_build_sensors()` and pass it to the `NavigationManager`

---

## License

MIT
