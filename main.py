import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)   
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1) 
p.resetDebugVisualizerCamera(
	cameraDistance=5,
	cameraYaw=45,
	cameraPitch=-30,
	cameraTargetPosition=[0, 0, 0]
)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")

car_body_half_extents = [1.0, 0.5, 0.05]
wheel_half_extents = [0.2, 0.2, 0.2]

car_body_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=car_body_half_extents
)
car_body_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=car_body_half_extents, rgbaColor=[0.8, 0.1, 0.1, 1]
)

wheel_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=wheel_half_extents
)
wheel_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=wheel_half_extents, rgbaColor=[0.1, 0.1, 0.1, 1]
)

wheel_offsets = [
    [0.8, 0.45, -0.25],   # Front-right
    [0.8, -0.45, -0.25],  # Front-left
    [-0.8, 0.45, -0.25],  # Rear-right
    [-0.8, -0.45, -0.25], # Rear-left
]

car_body = p.createMultiBody(
    baseMass=2,
    baseCollisionShapeIndex=car_body_collision,
    baseVisualShapeIndex=car_body_visual,
    basePosition=[0, 0, 0.3],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    linkMasses=[0.5] * 4,
    linkCollisionShapeIndices=[wheel_collision] * 4,
    linkVisualShapeIndices=[wheel_visual] * 4,
    linkPositions=wheel_offsets,
    linkOrientations=[p.getQuaternionFromEuler([0, 0, 0])] * 4,
    linkInertialFramePositions=[[0, 0, 0]] * 4,
    linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0])] * 4,
    linkParentIndices=[0, 0, 0, 0],
    linkJointTypes=[p.JOINT_FIXED] * 4,
    linkJointAxis=[[0, 0, 0]] * 4,
)

for _ in range(240 * 5):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()