# Pharma Bot — Intelligent Warehouse Delivery Robot
---

## Project summary
Pharma Bot is an **autonomous delivery robot system** designed for a structured warehouse-style map.  
It detects shop/package locations and map configuration from an overhead image, sets up a simulated arena, plans shortest delivery routes while accounting for blocked roads and traffic signals, and executes routes using **line-following** on a physical robot.  

The system integrates:  
- **OpenCV-based perception**  
- **Graph-based path planning**  
- **ZeroMQ / socket communication** for simulation ↔ robot coordination  
- **CoppeliaSim** for virtual testing with an overhead camera  

---

## Key features

### Arena configuration from overhead image
- Detects node grid, start/end nodes, traffic signals, blocked horizontal/vertical roads, and package locations using OpenCV.  
- Supports dynamic arena configurations — shop/package positions and blocked roads vary with each `config_image.png`.

### Package detection & placement
- Extracts package color, shape, and centroid from the configuration image.  
- Places matching 3D models into CoppeliaSim at appropriate shop positions.

### Graph-based path planning
- Builds a connectivity graph between grid nodes.  
- Removes edges for roads under construction and computes shortest feasible routes.  
- Converts node sequences into discrete actions (`STRAIGHT`, `LEFT`, `RIGHT`, `REVERSE`, `WAIT_5`) while tracking orientation and traffic signals.

### Vision & localization
- Uses **ArUco markers + perspective transform** to crop the overhead camera view to arena coordinates.  
- Maps ArUco pose into CoppeliaSim world coordinates and updates robot position/orientation in the simulator.

### QR code reading for deliveries
- Reads QR codes placed at checkpoints using a simulated vision sensor.  
- Determines package delivery destinations dynamically.

### Robot control (Raspberry Pi)
- Line-following implementation with PiCamera + OpenCV (thresholding + masking).  
- **PID-based steering** with encoder-assisted turns (left/right/reverse/rotate-in-place).  
- Node detection with traffic-aware actions (pause, wait, pick/drop).

### Socket-based robot ↔ simulator protocol
- Simulation side acts as a socket server.  
- Sends move sequences to the Pi client.  
- Receives pickup/delivery confirmations back from the robot.

### CoppeliaSim integration
- Automatically loads package, traffic signal, barricade, start & end node models via the Remote API.  
- Simulates package pick/drop through parenting in scene graph.  
- Uses an overhead IP camera feed to emulate real-world inputs.

---


Pi-side (line following + GPIO control) contains:  
- Motion primitives: `forward`, `stop`, `left_turn`, `right_turn`, `reverse`, `wait`  
- Vision helpers: `process_img`, `hsv_mask`, `detect_line`, `detect_node`  
- Control: `PID`, `follow_line`, `control_logic`, `move_bot`

---

## How it works — end-to-end flow

1. **Configuration detection**  
   - Read `config_image.png`.  
   - Detect nodes, start/end, traffic signals, blocked roads, and medicine package details.  

2. **Scene setup**  
   - Place packages, traffic signals, barricades, start & end markers into CoppeliaSim via Remote API.  

3. **Path planning**  
   - Build connectivity graph of nodes.  
   - Remove edges for blocked roads.  
   - Plan shortest path(s) and convert to moves.  

4. **Commanding the robot**  
   - Simulator sends move sequence to Raspberry Pi.  
   - Pi executes line-following + encoder-based turns at nodes.  

5. **Pickup & delivery**  
   - At shop node → simulator attaches package model to robot.  
   - Robot reads QR codes at checkpoints to identify correct delivery points.  
   - Packages dropped at correct nodes.  

6. **Finish**  
   - Robot navigates to end node.  
   - Simulation stops.  

---

## Dependencies

**General:**  
- Python 3.x  
- OpenCV (`cv2`)  
- NumPy  
- `pyzbar` (QR code decoding)  
- `zmqRemoteApi` (CoppeliaSim Remote API)  
- `zmq` (if used)  
- `socket` (Python stdlib)  

**Raspberry Pi side:**  
- `RPi.GPIO`  
- `picamera`  

**Simulation:**  
- CoppeliaSim (latest with ZeroMQ Remote API enabled)

---

## How to run

### Simulation (host PC)
> Assumes CoppeliaSim is running with Remote API server enabled.  

1. Place `config_image.png` and models (`package_models/`, `signals/`, `barricades/`) in repo root.  
2. Start CoppeliaSim and ensure `/Arena` object exists in the scene.  
3. Run:

```bash
python3 task_6.py

