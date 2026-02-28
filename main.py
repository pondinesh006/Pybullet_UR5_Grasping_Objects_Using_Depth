import cv2
import time
import random
import config
import pybullet as p
import pybullet_data
from ur5_robot import UR5ROBOT
from depth_estimation import DETECTION
from utils import (setup_simulation, depth_camera_setup, 
                    render_camera, update_simulation, spawn_cube)


def reset_state():
    """
    Remove the current cube from simulation and reset all state flags in config.
    """
    if config.current_cube_id is not None:
        try:
            p.removeBody(config.current_cube_id)
        except Exception:
            pass
    config.current_cube_id = None
    config.cube_spawned = False
    config.cube_detected = False
    config.cube_point_collected = False
    config.robot_moved = False
    config.initial_pose_moved = False
    config.cube_position = []
    config.overhead_img = None
    config.waiting_for_input = True


def main():
    """
    Main function to control the robot and perform the task.
    Each cycle waits for user input before spawning a cube, then performs
    the full detect → grasp → return sequence. After completion the cube is
    removed and the user is prompted to start a new cycle.
    """
    p.connect(p.GUI)
    objects_ids = setup_simulation()

    robot = UR5ROBOT(config.initial_pose, p)
    robot.load_robot()

    detector = DETECTION()

    update_simulation(200)

    # Setup cameras
    width, height, proj_matrix, _ = depth_camera_setup(
        width=config.depth_parameter["Width"],
        height=config.depth_parameter["Height"],
        fov=config.depth_parameter["FOV"],
        near=config.depth_parameter["Near"],
        far=config.depth_parameter["Far"],
    )
    _, _, _, overhead_view = depth_camera_setup(
        width=config.depth_parameter["Width"],
        height=config.depth_parameter["Height"],
        fov=config.depth_parameter["FOV"],
        near=config.depth_parameter["Near"],
        far=config.depth_parameter["Far"],
        eye_pos=[0.5, 0, 1.5],
        target_pos=[0.5, 0, 0]
    )

    robot.move_arm(config.initial_pose)
    update_simulation(200)

    # Per-cycle variables are managed via config (see config.py)
    print("""
    ========================================
     UR5 Grasping Simulation
     Type 's' to spawn a cube and
     start the motion sequence.
     Press [q] in the camera window to quit.
    ========================================
    """)

    while p.isConnected():
        update_simulation(1)

        # ── Render cameras ────────────────────────────────────────────────────
        overhead_img_raw = render_camera(overhead_view, proj_matrix, width, height)
        robot_view = robot.get_camera_view()
        robot_img_raw = render_camera(robot_view, proj_matrix, width, height)
        robot_img = detector.bgr_to_rgb(robot_img_raw, height, width)

        # ── Wait for user input before starting a new cycle ───────────────────
        if config.waiting_for_input:
            user_input = input("\nType 's' to cycle, 'q' to quit: ").strip().lower()
            if user_input == 'q':
                print("Quitting simulation.")
                break

            if user_input != 's':
                print(f"  Unknown command '{user_input}'. Type 's' to start or 'q' to quit.")
                continue

            # Reset state and spawn a new cube at a random position within config ranges
            reset_state()
            rand_x = random.uniform(config.x_range[0], config.x_range[1])
            rand_y = random.uniform(config.y_range[0], config.y_range[1])
            print(f"Spawning cube at x={rand_x:.3f}, y={rand_y:.3f}")
            config.current_cube_id = spawn_cube([rand_x, rand_y, 0.65], [1, 0, 0, 1])
            config.cube_spawned = True
            config.waiting_for_input = False
            print("Cube spawned! Starting motion sequence...")

        # ── State machine ─────────────────────────────────────────────────────
        if config.cube_spawned and not config.cube_detected:
            config.cube_detected, _, v, config.overhead_img = detector.detect(
                "overhead", overhead_img_raw, height, width)
            if config.cube_detected:
                robot.move_arm(config.visual_object_position)
                update_simulation(200)

        elif config.cube_detected and not config.cube_point_collected:
            u, v = detector.detect("wrist", robot_img_raw, height, width)
            if u is None or v is None:
                continue
            k, fx, fy, cx, cy = detector.compute_intrinsics(
                width, height, config.depth_parameter["FOV"])
            depth = detector.pixel_to_depth(
                robot_img_raw, height, width, u, v,
                fx, fy, cx, cy, config.depth_parameter["Far"])
            cam_xyz = detector.projection_point(u, v, depth, fx, fy, cx, cy)
            config.cube_position = detector.projection_world(
                cam_xyz, robot.compute_extrinsics())
            config.cube_position = config.cube_position.astype(float)
            print(f"Cube detected at ({config.cube_position[0]}, "
                  f"{config.cube_position[1]}, {config.cube_position[2]})")
            config.cube_point_collected = True

        elif config.cube_detected and config.cube_point_collected and not config.robot_moved:
            y = round(config.cube_position[1], 2)
            if y < 0:
                y = y - 0.01
            else:
                y = y + 0.01

            # Hover above cube
            joint_angles = robot.calculate_ik(robot.pose_xyz_rpy(
                (round(config.cube_position[0], 2) - 0.15) * config.x_axis,
                y * config.y_axis,
                0.1, 1.5708, 0.0, -1.5
            ))
            robot.move_arm(joint_angles)
            update_simulation(500)

            # Drop down to grasp pose
            joint_angles = robot.calculate_ik(robot.pose_xyz_rpy(
                (round(config.cube_position[0], 2) - 0.15) * config.x_axis,
                y * config.y_axis,
                0.05, 1.5708, 0.0, -1.5
            ))
            robot.move_arm(joint_angles)
            update_simulation(500)

            # Grasp
            robot.move_gripper(0.04)
            update_simulation(500)

            # Hover back up
            joint_angles = robot.calculate_ik(robot.pose_xyz_rpy(
                (round(config.cube_position[0], 2) - 0.15) * config.x_axis,
                y * config.y_axis,
                0.1, 1.5708, 0.0, -1.5
            ))
            robot.move_arm(joint_angles)
            update_simulation(500)

            # Release
            robot.move_gripper(0.085)
            update_simulation(500)
            config.robot_moved = True

        elif (config.cube_detected and config.cube_point_collected
              and config.robot_moved and not config.initial_pose_moved):
            robot.move_arm(config.initial_pose)
            update_simulation(500)
            config.initial_pose_moved = True
            print("\n✓ Cycle complete! Cube removed and state reset.")
            # ── Cycle finished – remove cube and prompt for next cycle ─────
            reset_state()

        # ── Display camera feeds ──────────────────────────────────────────────
        cv2.imshow("Robot Camera", robot_img)
        if config.overhead_img is not None:
            cv2.imshow("Overhead Camera", config.overhead_img)

        # Break on 'q' key in the OpenCV window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
