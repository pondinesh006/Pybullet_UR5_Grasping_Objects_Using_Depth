CUBE_SIZE = 0.03 # Cube size in meters

x_axis = -1
y_axis = -1

x_range = [0.6, 0.8]
y_range = [-0.3, 0.3]

depth_parameter = {
    "Width": 640, 
    "Height": 480, 
    "FOV": 69, 
    "Near": 0.01, 
    "Far": 3.0
}

initial_pose = [0, -2.35619, 2.35619, 0, 1.5, 0]
visual_object_position = [0, -2.35619, 2.35619, 0.436332, 1.5, 0.0]

cube_position = []
robot_moved = False
cube_spawned = False
cube_detected = False
initial_pose_moved = False
cube_point_collected = False

# Per-cycle runtime variables
current_cube_id = None   # PyBullet body ID of the active cube
overhead_img = None      # Keep last frame so OpenCV always has something to show
waiting_for_input = True # Start by asking the user
