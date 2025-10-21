
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

cube_static = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
cube_static_body = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=cube_static,
    basePosition=[0, 0, 0.5]
)

cube_dynamic = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
cube_dynamic_body = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=cube_dynamic,
    basePosition=[0, 0, 3]
)

for _ in range(240 * 5):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()
