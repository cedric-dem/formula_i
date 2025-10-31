#terrain = p.loadURDF("f1_tracks/obj_files/13_belgian_gp.obj")  # replace with your .obj if converted to URDF
import pybullet as p
import pybullet_data
import time
import math

# --- SETUP ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load your terrain mesh (replace with your own path)
terrain_shape = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="f1_tracks/obj_files/13_belgian_gp.obj",  # your terrain mesh
    meshScale=[1, 1, 1]
)
terrain_body = p.createMultiBody(0, terrain_shape)

# Create a cube to act as the car
cube_start_pos = [-430, 1084, 106]
cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.25, 0.2])
cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.25, 0.2], rgbaColor=[0, 0, 1, 1])
cube = p.createMultiBody(baseMass=800, baseCollisionShapeIndex=cube_collision,
                         baseVisualShapeIndex=cube_visual,
                         basePosition=cube_start_pos,
                         baseOrientation=cube_start_orientation)
# --- after creating bodies ---
p.setRealTimeSimulation(0)

# tame friction/damping a bit
p.changeDynamics(cube, -1, lateralFriction=0.8, rollingFriction=0.1,
                 spinningFriction=0.1, linearDamping=0.02, angularDamping=0.02)
p.changeDynamics(terrain_body, -1, lateralFriction=1.0)

# --- CONTROL PARAMETERS ---
throttle_force = 0.0          # in Newtons (applied along local +X)
steering_angle = 0.0
max_steering = math.radians(25)
max_throttle = 20000.0        # 20 kN cap is plenty for a sliding cube
brake_force = 0.0

throttle_step = 2000.0        # how much N you add/remove per frame when holding key
brake_strength = 8000.0       # N opposing current heading

while True:
    keys = p.getKeyboardEvents()

    # helpers for "is key held?"
    def down(ch):
        return (keys.get(ord(ch), 0) & p.KEY_IS_DOWN) != 0

    # --- Steering (about Z) ---
    if down('o'):
        steering_angle = max(-max_steering, steering_angle - 0.05)
    elif down('l'):
        steering_angle = min(max_steering, steering_angle + 0.05)
    else:
        steering_angle *= 0.9

    # --- Throttle & Brake ---
    if down('p'):  # throttle forward
        throttle_force = min(max_throttle, throttle_force + throttle_step)
    elif down('m'):  # throttle reverse
        throttle_force = max(-max_throttle, throttle_force - throttle_step)
    else:
        throttle_force *= 0.9  # coast down

    brake_force = brake_strength if (keys.get(ord(' '), 0) & p.KEY_IS_DOWN) else 0.0

    # --- Apply Forces ---
    pos, orn = p.getBasePositionAndOrientation(cube)

    # apply throttle along local +X (no need to compute "forward")
    p.applyExternalForce(
        objectUniqueId=cube, linkIndex=-1,
        forceObj=[throttle_force, 0, 0],
        posObj=[0, 0, 0],
        flags=p.LINK_FRAME  # <-- key change: local frame
    )

    # simple brake as opposite to local +X
    if brake_force > 0:
        p.applyExternalForce(
            cube, -1, [-brake_force, 0, 0], [0, 0, 0], p.LINK_FRAME
        )

    # --- Steering (rotate the cube "artificially") ---
    yaw_change = steering_angle * 0.05
    euler = list(p.getEulerFromQuaternion(orn))
    euler[2] += yaw_change
    p.resetBasePositionAndOrientation(cube, pos, p.getQuaternionFromEuler(euler))

    # --- CAMERA FOLLOW (unchanged except we read yaw after orientation reset) ---
    cam_distance, cam_height, cam_pitch = 5, 5, -60
    euler = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(cube)[1])
    cam_yaw = math.degrees(euler[2])
    p.resetDebugVisualizerCamera(cam_distance, cam_yaw, cam_pitch, pos)

    p.stepSimulation()
    time.sleep(1/240)
