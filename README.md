# Autonomous Ground Robot

A modular, open-source software stack for autonomous ground robot navigation.
Upload it to any standardised ground platform and drive it anywhere.

---

## Features

| Category | Capability |
|---|---|
| **Sensors** | GPS (NMEA/serial), 2-D LiDAR (RPLidar), Intel RealSense depth camera, HC-SR04 ultrasonic |
| **Navigation** | A\*, Dijkstra, Artificial Potential Fields; binary occupancy grid updated from all sensors |
| **Communication** | Carrier-agnostic ground station link – LTE (TCP), LoRa (UART), Radio (serial/SiK) |
| **Ground station protocol** | Binary framed + CRC-8 JSON messages: HEARTBEAT, TELEMETRY, COMMAND, STATUS, ACK |
| **Motor control** | Proportional heading controller → differential-drive PWM |
| **Configuration** | Single YAML file; all hardware is optional (simulation fallback built-in) |

---

## Repository layout

```
autonomous_ground_robot/        # Main Python package
│
├── sensors/                    # Hardware sensor interfaces
│   ├── base.py                 #   Abstract BaseSensor
│   ├── gps.py                  #   GPS (NMEA serial / simulation)
│   ├── lidar.py                #   2-D LiDAR (RPLidar / simulation)
│   ├── depth_camera.py         #   Depth camera (RealSense / simulation)
│   └── ultrasonic.py           #   HC-SR04 (GPIO / simulation)
│
├── navigation/                 # Path planning and navigation
│   ├── algorithms/
│   │   ├── astar.py            #   A* with octile heuristic
│   │   ├── dijkstra.py         #   Dijkstra uniform-cost search
│   │   └── potential_fields.py #   Artificial Potential Fields (reactive)
│   ├── path_planner.py         #   OccupancyGrid + PathPlanner
│   └── manager.py              #   NavigationManager (threaded)
│
├── communication/              # Ground station link
│   ├── base.py                 #   Abstract BaseComm carrier
│   ├── protocol.py             #   GroundStationProtocol (framing, CRC)
│   ├── lte.py                  #   LTE TCP socket carrier
│   ├── lora.py                 #   LoRa UART carrier
│   └── radio.py                #   Radio serial carrier
│
├── control/
│   └── motor_controller.py     # Differential-drive motor controller
│
├── utils/
│   ├── coordinates.py          # GPS ↔ local ENU conversions
│   └── logger.py               # Centralised logging
│
├── config/
│   └── default_config.yaml     # Default (simulation) configuration
│
└── robot.py                    # Top-level Robot orchestrator

main.py                         # CLI entry point
tests/                          # Pytest test suite
requirements.txt
setup.py
```

---

## Quick start

### 1. Install dependencies

```bash
pip install pyyaml pytest
# For real hardware also install: pyserial RPi.GPIO pyrealsense2 rplidar-roboticia
```

### 2. Run in simulation (no hardware required)

```bash
python main.py
```

Navigate to a local waypoint (metres from start):

```bash
python main.py --goal-x 10 --goal-y 5
```

Navigate to a GPS coordinate:

```bash
python main.py --goal-lat 48.8600 --goal-lon 2.3550
```

Choose a navigation algorithm:

```bash
python main.py --algorithm dijkstra
python main.py --algorithm potential_fields
```

Run for a fixed duration then exit:

```bash
python main.py --goal-x 5 --goal-y 5 --duration 30
```

### 3. Use the Python API

```python
from autonomous_ground_robot import Robot

robot = Robot("config/my_config.yaml")   # or Robot() for defaults
robot.start()

# Navigate to a GPS coordinate
robot.navigation.set_goal_gps(48.860, 2.355)

# Or navigate in the local ENU frame (metres from starting position)
robot.navigation.set_goal_local(x=5.0, y=10.0)

# Inspect state
print(robot.navigation.state)   # NavigationState.NAVIGATING
print(robot.navigation.position)  # LocalPoint(x=..., y=...)

robot.stop()
```

---

## Configuration

Copy and edit `autonomous_ground_robot/config/default_config.yaml`.

Key sections:

```yaml
navigation:
  algorithm: astar        # astar | dijkstra | potential_fields
  cruise_speed: 1.0       # m/s
  emergency_stop_distance: 0.3  # metres

communication:
  enabled: true
  carrier: lora           # lte | lora | radio
  lora:
    port: /dev/ttyS0      # omit for simulation
```

Pass the custom config path on the command line:

```bash
python main.py --config /path/to/my_config.yaml
```

---

## Ground station protocol

Messages are binary-framed with CRC-8 integrity checking.

| Type | Direction | Purpose |
|---|---|---|
| `HEARTBEAT` | both | Keepalive / link check |
| `TELEMETRY` | robot → GS | Position, heading, state, speed |
| `COMMAND` | GS → robot | `goto_gps`, `goto_local`, `stop`, `set_algorithm` |
| `STATUS` | robot → GS | Ready, error, goal-reached notifications |
| `ACK` / `NACK` | both | Command acknowledgement |

### Example COMMAND payload

```json
{"type": "goto_gps", "latitude": 48.860, "longitude": 2.355, "altitude": 0}
{"type": "goto_local", "x": 10.0, "y": 5.0}
{"type": "stop"}
```

---

## Running tests

```bash
pip install pytest
pytest tests/ -v
```

---

## Adding a new communication carrier

1. Create `autonomous_ground_robot/communication/my_carrier.py`
2. Subclass `BaseComm` and implement `connect`, `disconnect`, `send_bytes` (and optionally `receive_bytes`)
3. Add it to `communication/__init__.py`
4. Register the carrier name in `robot.py:_build_protocol`

---

## Extending navigation algorithms

1. Create `autonomous_ground_robot/navigation/algorithms/my_algo.py`
2. Subclass `BaseAlgorithm` and implement `find_path(grid, start, goal) → AlgorithmResult`
3. Add it to `algorithms/__init__.py`
4. Register the name in `navigation/manager.py:_build_algorithm`

---

## License

MIT
