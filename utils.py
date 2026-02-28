import time
import pybullet as p
import pybullet_data
from config import CUBE_SIZE

def spawn_cube(pos, color):
    """
    Spawn a cube at the given position with the given color.
    """
    cube_id = p.createMultiBody(
        baseMass=0.05,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[CUBE_SIZE]*3),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX, halfExtents=[CUBE_SIZE]*3, rgbaColor=color),
        basePosition=pos
    )

    p.changeDynamics(cube_id, -1,
                     lateralFriction=2.5,
                     spinningFriction=0.8,
                     rollingFriction=0.001,
                     restitution=0,
                     contactStiffness=1000,
                     contactDamping=50)

    return cube_id

def setup_simulation():
    """
    Set up the simulation environment and objects.
    """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)

    return table

def depth_camera_setup(
        width=640,
        height=480,
        fov=69,
        near=0.01,
        far=3.0,
        eye_pos=None,
        target_pos=None,
        up_vector=[0, 1, 0]
    ):
    """
    Setup both projection and view matrix
    """
    aspect = width / height

    # ---- Projection (Intrinsics) ----
    projection_matrix = p.computeProjectionMatrixFOV(
        fov,
        aspect,
        near,
        far
    )

    # ---- View (Extrinsics) ----
    if eye_pos is not None and target_pos is not None:
        view_matrix = p.computeViewMatrix(
            eye_pos,
            target_pos,
            up_vector
        )
    else:
        view_matrix = None

    return width, height, projection_matrix, view_matrix

def render_camera(view_matrix, projection_matrix, width, height):
    """
    Render an image from a camera view and return as BGR for OpenCV.
    """
    img_data = p.getCameraImage(
        width,
        height,
        view_matrix,
        projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    return img_data

def update_simulation(steps, sleep_time=0.01):
    """
    Update the simulation by stepping and waiting for a specified time.
    """
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(sleep_time)
