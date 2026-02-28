# PyBullet UR5 Grasping Objects

A physics-based robot manipulation simulation built with **PyBullet**, featuring a **UR5 robotic arm** equipped with a **Robotiq 85 gripper** and an **Intel RealSense D435 depth camera**. The robot autonomously detects and grasps a randomly spawned coloured cube on a table using computer vision and iterative inverse kinematics.

---

## Demo Overview

Each simulation cycle follows this sequence:

```
User Input ('s')
    → Spawn cube at random position
    → Overhead camera detects cube (HSV colour segmentation)
    → Robot moves to visual inspection pose
    → Wrist camera estimates 3-D cube position (depth + back-projection)
    → IK-planned grasp: hover → descend → close gripper → lift → release
    → Robot returns to home pose
    → Cycle resets, awaiting next input
```

---

## Project Structure

```
Pybullet_UR5_Grasping_Objects/
├── main.py               # Entry point – simulation loop & state machine
├── config.py             # All tunable parameters and shared state variables
├── ur5_robot.py          # UR5ROBOT class: FK, IK, arm/gripper control, camera
├── depth_estimation.py   # DETECTION class: colour detection, depth, 3-D projection
├── utils.py              # Helpers: simulation setup, camera setup, cube spawning
└── Model/
    ├── urdf/
    │   ├── ur5_robotiq_85.urdf          # UR5 + Robotiq 85 (no camera)
    │   └── ur5_robotiq_85_d435.urdf     # UR5 + Robotiq 85 + RealSense D435
    └── meshes/                           # Visual & collision mesh files
```

---

## Module Details

### `main.py` — Simulation Entry Point

Initialises PyBullet in GUI mode and runs the main event loop. Implements a **five-stage state machine** driven by config flags:

| Stage | Condition | Action |
|-------|-----------|--------|
| 0 | `waiting_for_input` | Prompt user (`s` / `q`) |
| 1 | `cube_spawned` & not detected | Overhead detection; arm to inspection pose |
| 2 | `cube_detected` & not localised | Wrist-cam depth → 3-D world position |
| 3 | Localised & not grasped | Hover → descend → grasp → lift → release |
| 4 | Grasped & not homed | Return to `initial_pose`; reset all flags |

Key function: `reset_state()` — removes the current cube body and clears all per-cycle config flags between cycles.

---

### `config.py` — Centralised Configuration

All magic numbers live here. Edit this file to change behaviour without touching logic.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CUBE_SIZE` | `0.03` m | Half-extent of the spawned cube |
| `x_range` | `[0.6, 0.8]` | Random X spawn range (world frame) |
| `y_range` | `[-0.3, 0.3]` | Random Y spawn range (world frame) |
| `x_axis` / `y_axis` | `-1` | Sign conventions for IK target computation |
| `depth_parameter` | 640×480, FOV 69°, Far 3 m | Shared camera intrinsic settings |
| `initial_pose` | `[0, -2.356, 2.356, 0, 1.5, 0]` | Home joint angles (radians) |
| `visual_object_position` | — | Joint angles for overhead inspection pose |

Runtime state flags (`cube_spawned`, `cube_detected`, `cube_point_collected`, `robot_moved`, `initial_pose_moved`, `waiting_for_input`) are also stored here for cross-module access.

---

### `ur5_robot.py` — UR5ROBOT Class

Wraps all robot control logic.

| Method | Description |
|--------|-------------|
| `load_robot()` | Loads URDF, parses joints, sets up gripper mimic constraints |
| `move_arm(joint_angles)` | Position-control all 6 arm joints simultaneously |
| `move_gripper(open_length)` | Opens/closes gripper; range 0 – 0.085 m |
| `calculate_fk(q)` | Forward kinematics via DH parameters → 4×4 homogeneous matrix |
| `calculate_ik(T_target)` | Numerical IK using damped least-squares Jacobian (max 200 iters) |
| `numerical_jacobian(q)` | 6×6 Jacobian via finite differences |
| `get_camera_view()` | Computes wrist-camera view matrix (ROS optical → OpenGL conversion) |
| `compute_extrinsics()` | Returns 4×4 camera-to-world transform T for back-projection |
| `pose_xyz_rpy(x,y,z,r,p,y)` | Builds a target homogeneous matrix from position + RPY angles |

**DH Parameters (UR5):**

| i | a (m) | α (rad) | d (m) |
|---|-------|---------|-------|
| 1 | 0 | π/2 | 0.0892 |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.3922 | 0 | 0 |
| 4 | 0 | π/2 | 0.1092 |
| 5 | 0 | -π/2 | 0.0947 |
| 6 | 0 | 0 | 0.0823 |

---

### `depth_estimation.py` — DETECTION Class

Handles all perception tasks.

| Method | Description |
|--------|-------------|
| `detect(camera_type, image, h, w)` | HSV colour segmentation (red hue) → centroid (u, v) |
| `compute_intrinsics(w, h, fov)` | Returns camera matrix K and focal lengths |
| `pixel_to_depth(image, ..., u, v)` | Linearises PyBullet depth buffer → metric Z (metres) |
| `projection_point(u, v, Z, ...)` | Back-projects pixel + depth → 3-D camera-frame point |
| `projection_world(xyz_cam, T)` | Transforms camera-frame point → world frame using extrinsics |
| `bgr_to_rgb(image, h, w)` | Reshapes PyBullet RGBA buffer → BGR for OpenCV display |

**Depth linearisation formula:**
```
Z = (2 · near · far) / (far + near − (2·z_buffer − 1) · (far − near))
```

---

### `utils.py` — Simulation Utilities

| Function | Description |
|----------|-------------|
| `setup_simulation()` | Sets gravity, loads ground plane and table URDF |
| `spawn_cube(pos, color)` | Creates a dynamic cube with friction/damping properties |
| `depth_camera_setup(...)` | Computes projection & optional view matrices for any camera |
| `render_camera(view, proj, w, h)` | Calls `getCameraImage` via hardware OpenGL renderer |
| `update_simulation(steps)` | Steps physics and sleeps for the given number of ticks |

---

## Requirements

```
python >= 3.8
pybullet
opencv-python   (cv2)
numpy
```

Install dependencies:

```bash
pip install pybullet opencv-python numpy
```

---

## Running the Simulation

```bash
python main.py
```

**Controls:**

| Input | Action |
|-------|--------|
| `s` + Enter | Spawn a cube and run one full grasp cycle |
| `q` + Enter | Quit the simulation (terminal prompt) |
| `q` key | Quit the simulation (OpenCV window focused) |

**Camera Windows:**

| Window | Feed |
|--------|------|
| `Robot Camera` | Live wrist-mounted RealSense view |
| `Overhead Camera` | Top-down view with cube detection overlay |

---

## How It Works — Technical Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│  1. SPAWN    Cube placed at random (x, y) on table surface  │
│                                                             │
│  2. DETECT   Overhead cam → HSV mask → contour → centroid   │
│              (u_oh, v_oh) confirms cube is present          │
│                                                             │
│  3. LOCALISE Wrist cam at inspection pose:                  │
│              • pixel (u,v) → metric depth Z                 │
│              • Back-project → camera frame P_c              │
│              • T_world_cam · P_c  → world frame P_w         │
│                                                             │
│  4. GRASP    IK solves q* for:                              │
│              • Hover   (P_w + 0.10 m Z-offset)             │
│              • Descend (P_w + 0.05 m Z-offset)             │
│              • Gripper close → 0.04 m                       │
│              • Lift    (P_w + 0.10 m Z-offset)             │
│              • Gripper open  → 0.085 m (release)           │
│                                                             │
│  5. RETURN   Arm back to initial_pose; cube removed         │
└─────────────────────────────────────────────────────────────┘
```

---

## Configuration Tips

- **Spawn area**: Adjust `x_range` and `y_range` in `config.py` to change where cubes appear.
- **Camera FOV / resolution**: Edit the `depth_parameter` dictionary.
- **Grasp height offsets**: Modify the Z values (`0.05`, `0.10`) in the grasp block inside `main.py`.
- **IK tolerance / iterations**: Change `tol` and `max_iters` in `UR5ROBOT.calculate_ik()`.
- **Cube size**: Change `CUBE_SIZE` in `config.py` (also affects collision shape).

---

## Known Limitations

- **Colour detection only**: The detector uses HSV thresholding for red objects. Other colours require updating the `inRange` bounds in `DETECTION.detect()`.
- **Single cube per cycle**: Only one cube is active at a time; multi-object grasping is not supported.
- **Blocking input**: `input()` in the main loop blocks the simulation render thread during prompts.
- **IK convergence**: The numerical IK may print *"Did not fully converge"* for configurations far from the initial pose; the returned approximation is still used.
