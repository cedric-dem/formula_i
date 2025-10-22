import math
import time

import pybullet as p


CAMERA_TYPE = "FOLLOW_CAR"  # Options: "FIXED", "FOLLOW_CAR", "ROTATE_AROUND_CAR"
MOVE_DECISION = "KEYBOARD"  # Options: "DEFAULT", "AI", "KEYBOARD"
SIMPLIFIED_MODEL = False
MAX_STEERING_ANGLE = math.radians(10.0)

BASE_COLOR = (0.9, 0.1, 0.1)
HELMET_COLOR = (0.1, 0.1, 0.1)
CAR_BODY_BLOCKS = [
    # (delta_x, delta_z, size_x, size_y, size_z, color_rgb)
    (0,0, 1.7, 1.5, 0.5, BASE_COLOR),  # Main chassis
    (0.5, 0.0, 3.8, 0.4, 0.4,BASE_COLOR),  # chassis lengthwise
    (2.3, -0.24, 0.5, 1.4, 0.1,  BASE_COLOR),  # Front wing assembly
    (-1.8, 0.24, 0.5, 1.0, 0.4, BASE_COLOR) ,  # Rear wing assembly
    (-0.6, 0.3, 0.7, 0.2, 0.4,BASE_COLOR),  # Center cell upper
    (0, 0.3, 0.3, 0.2, 0.4,HELMET_COLOR),  # Center cell upper
]

###################################

camera_rotation_angle = 0.0

###################################


class Car:
    def __init__(self):
        self.body_id = self._create_car()
        self.current_speed = 0.0
        _, orientation = p.getBasePositionAndOrientation(self.body_id)
        _, _, self.current_angle = p.getEulerFromQuaternion(orientation)
        self.current_steering = 0.0

    def apply_steering(self, steering_input):
        steering_angle = clamp(float(steering_input), -1.0, 1.0) * MAX_STEERING_ANGLE
        self.current_steering = steering_angle

        for joint_index in (0, 1):
            p.resetJointState(self.body_id, joint_index, steering_angle)
            p.setJointMotorControl2(
                self.body_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=steering_angle,
                force=100,
            )

    def _create_car(self):
        block_specs = CAR_BODY_BLOCKS if not SIMPLIFIED_MODEL else CAR_BODY_BLOCKS[:1]

        (
            car_body_collision,
            car_body_visual,
            base_delta_x,
            base_delta_z,
        ) = self._create_body(block_specs[0])

        wheel_collision, wheel_visual = self._create_wheels()

        (
            link_masses,
            link_collision_indices,
            link_visual_indices,
            link_positions,
            link_orientations,
            link_inertial_positions,
            link_inertial_orientations,
            link_parent_indices,
            link_joint_types,
            link_joint_axes,
        ) = self._create_wheel_links(wheel_collision, wheel_visual)

        identity_orientation = p.getQuaternionFromEuler([0, 0, 0])

        if len(block_specs) > 1:
            self._add_body_blocks(
                block_specs[1:],
                base_delta_x,
                base_delta_z,
                identity_orientation,
                link_masses,
                link_collision_indices,
                link_visual_indices,
                link_positions,
                link_orientations,
                link_inertial_positions,
                link_inertial_orientations,
                link_parent_indices,
                link_joint_types,
                link_joint_axes,
            )

        car_body = p.createMultiBody(
            baseMass=2,
            baseCollisionShapeIndex=car_body_collision,
            baseVisualShapeIndex=car_body_visual,
            basePosition=[0, 0, 0.3],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collision_indices,
            linkVisualShapeIndices=link_visual_indices,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_positions,
            linkInertialFrameOrientations=link_inertial_orientations,
            linkParentIndices=link_parent_indices,
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes,
        )

        p.changeDynamics(car_body, -1, lateralFriction=1.0)
        for link_index in range(p.getNumJoints(car_body)):
            p.changeDynamics(car_body, link_index, lateralFriction=1.0)

        for joint_index in range(2):
            p.setJointMotorControl2(
                car_body,
                joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0,
            )

        return car_body

    def _create_body(self, base_block):
        base_delta_x, base_delta_z = base_block[:2]
        base_color = list(base_block[-1]) + [1.0]
        base_half_extents = [
            base_block[2] / 2.0,
            base_block[3] / 2.0,
            base_block[4] / 2.0,
        ]

        car_body_collision = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=base_half_extents
        )
        car_body_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=base_half_extents, rgbaColor=base_color
        )

        return car_body_collision, car_body_visual, base_delta_x, base_delta_z

    def _create_wheels(self):
        wheel_radius = 0.321
        wheel_width = 0.350

        wheel_collision = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_width
        )
        wheel_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=wheel_radius,
            length=wheel_width,
            rgbaColor=[0.2, 0.2, 0.2, 1],
        )

        return wheel_collision, wheel_visual

    def _create_wheel_links(self, wheel_collision, wheel_visual):
        front_wheels_distance_to_center = 1.66
        back_wheels_distance_to_center = -1.46
        front_wheels_distance_to_center_laterally = 0.717
        back_wheels_distance_to_center_laterally = 0.717
        height_of_front_wheels = 0
        height_back_wheels = 0

        wheel_offsets = [
            [
                front_wheels_distance_to_center,
                front_wheels_distance_to_center_laterally,
                height_of_front_wheels,
            ],  # Front-right
            [
                front_wheels_distance_to_center,
                -front_wheels_distance_to_center_laterally,
                height_of_front_wheels,
            ],  # Front-left
            [
                back_wheels_distance_to_center,
                back_wheels_distance_to_center_laterally,
                height_back_wheels,
            ],  # Rear-right
            [
                back_wheels_distance_to_center,
                -back_wheels_distance_to_center_laterally,
                height_back_wheels,
            ],  # Rear-left
        ]

        wheel_orientation = p.getQuaternionFromEuler([math.pi / 2, 0, 0])

        link_joint_axes = [
            [0, 1, 0],
            [0, 1, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]

        link_masses = [0.5] * 4
        link_collision_indices = [wheel_collision] * 4
        link_visual_indices = [wheel_visual] * 4
        link_positions = [offset[:] for offset in wheel_offsets]
        link_orientations = [wheel_orientation] * 4
        link_inertial_positions = [[0, 0, 0] for _ in range(4)]
        link_inertial_orientations = [wheel_orientation] * 4
        link_parent_indices = [0, 0, 0, 0]
        link_joint_types = [
            p.JOINT_REVOLUTE,
            p.JOINT_REVOLUTE,
            p.JOINT_FIXED,
            p.JOINT_FIXED,
        ]

        return (
            link_masses,
            link_collision_indices,
            link_visual_indices,
            link_positions,
            link_orientations,
            link_inertial_positions,
            link_inertial_orientations,
            link_parent_indices,
            link_joint_types,
            link_joint_axes,
        )

    def _add_body_blocks(
        self,
        block_specs,
        base_delta_x,
        base_delta_z,
        identity_orientation,
        link_masses,
        link_collision_indices,
        link_visual_indices,
        link_positions,
        link_orientations,
        link_inertial_positions,
        link_inertial_orientations,
        link_parent_indices,
        link_joint_types,
        link_joint_axes,
    ):
        for block in block_specs:
            half_extents = [block[2] / 2.0, block[3] / 2.0, block[4] / 2.0]
            block_collision = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half_extents
            )

            block_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                rgbaColor=list(block[-1]) + [1.0],
            )

            link_masses.append(0.1)
            link_collision_indices.append(block_collision)
            link_visual_indices.append(block_visual)
            link_positions.append(
                [block[0] - base_delta_x, 0.0, block[1] - base_delta_z]
            )
            link_orientations.append(identity_orientation)
            link_inertial_positions.append([0, 0, 0])
            link_inertial_orientations.append(identity_orientation)
            link_parent_indices.append(0)
            link_joint_types.append(p.JOINT_FIXED)
            link_joint_axes.append([0, 0, 0])

    def update_vehicle_state(self, turn, brake, throttle, dt):
        max_speed = 10.0
        acceleration_rate = 5.0
        brake_deceleration = 10.0
        turn_speed = 1.0

        turn = clamp(float(turn), -1.0, 1.0)
        brake = clamp(float(brake), 0.0, 1.0)
        throttle = clamp(float(throttle), 0.0, 1.0)

        target_speed = throttle * max_speed
        self.current_speed += (target_speed - self.current_speed) * acceleration_rate * dt
        self.current_speed = max(
            self.current_speed - brake * brake_deceleration * dt, 0.0
        )

        self.current_angle += turn * turn_speed * dt

        car_position, physics_orientation = p.getBasePositionAndOrientation(self.body_id)
        current_linear_velocity, current_angular_velocity = p.getBaseVelocity(
            self.body_id
        )

        roll, pitch, _ = p.getEulerFromQuaternion(physics_orientation)
        car_orientation = p.getQuaternionFromEuler([roll, pitch, self.current_angle])
        p.resetBasePositionAndOrientation(self.body_id, car_position, car_orientation)

        forward_vector = [
            math.cos(self.current_angle),
            math.sin(self.current_angle),
            0.0,
        ]
        linear_velocity = [
            forward_vector[0] * self.current_speed,
            forward_vector[1] * self.current_speed,
            current_linear_velocity[2],
        ]
        p.resetBaseVelocity(
            self.body_id,
            linearVelocity=linear_velocity,
            angularVelocity=current_angular_velocity,
        )

        update_camera(car_position, self.current_angle, dt)

        return self.current_angle, self.current_speed

def get_next_move():
    """Return the steering, brake and throttle values for the next step."""
    if MOVE_DECISION == "DEFAULT":
        return [0.3, 0.0, 0.3]

    if MOVE_DECISION == "AI":
        return get_ai_decision(get_sensors())

    if MOVE_DECISION == "KEYBOARD":
        return get_keyboard_move()

    return [0.0, 0.0, 0.0]


def get_ai_decision(_sensors):
    # TODO: Replace the placeholder with an actual AI controller.
    return [0.3, 0.0, 0.3]


def get_sensors():
    # TODO: Collect meaningful sensor information from the simulation.
    return [0, 1, 0, 0, 2]


def get_keyboard_move():
    keys = p.getKeyboardEvents()

    def is_down(key_code):
        return keys.get(key_code, 0) & p.KEY_IS_DOWN

    throttle = 1.0 if (is_down(ord("z")) or is_down(p.B3G_UP_ARROW)) else 0.0
    brake = 1.0 if (is_down(ord("s")) or is_down(p.B3G_DOWN_ARROW)) else 0.0

    steering = 0.0
    if is_down(ord("q")) or is_down(p.B3G_LEFT_ARROW):
        steering += 1.0
    if is_down(ord("d")) or is_down(p.B3G_RIGHT_ARROW):
        steering -= 1.0

    return [clamp(steering, -1.0, 1.0), brake, throttle]


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def initialize_simulation():
    p.connect(p.GUI, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=1.0')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    if CAMERA_TYPE == "FIXED":
        p.resetDebugVisualizerCamera(
            cameraDistance=5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0],
        )


def setup_environment():
    p.setGravity(0, 0, -9.81)

    ground_half_extents = [500.0, 500.0, 0.1]
    ground_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=ground_half_extents
    )
    ground_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=ground_half_extents, rgbaColor=[0.1, 0.9, 0.1, 1]
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=ground_collision,
        baseVisualShapeIndex=ground_visual,
        basePosition=[0, 0, -ground_half_extents[2]],
    )

    for height, lateral_shift in ((6, 10), (12, 20), (18, 30)):
        create_ramp(height, lateral_shift)


def create_ramp(ramp_height, lateral_delta):
    ramp_length = 30.0
    ramp_width = 6.0
    ramp_thickness = 0.5
    ramp_start_offset_x = 15.0
    ramp_start_depth = -2.0  # Negative value lowers the ramp entrance below ground level

    half_length = ramp_length / 2.0
    half_width = ramp_width / 2.0
    half_thickness = ramp_thickness / 2.0

    ramp_angle = -math.atan2(ramp_height, ramp_length)

    cos_angle = math.cos(ramp_angle)
    sin_angle = math.sin(ramp_angle)

    # Position the ramp so the lower edge sits on the ground away from the origin.
    bottom_local_x = -half_length
    bottom_local_z = -half_thickness
    ramp_center_x = ramp_start_offset_x - (
        bottom_local_x * cos_angle + bottom_local_z * sin_angle
    )
    ramp_center_z = ramp_start_depth - (
        -bottom_local_x * sin_angle + bottom_local_z * cos_angle
    )

    ramp_half_extents = [half_length, half_width, half_thickness]
    ramp_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=ramp_half_extents
    )
    ramp_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=ramp_half_extents, rgbaColor=[0.5, 0.5, 0.5, 1]
    )

    ramp_center_y = lateral_delta

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=ramp_collision,
        baseVisualShapeIndex=ramp_visual,
        basePosition=[ramp_center_x, ramp_center_y, ramp_center_z],
        baseOrientation=p.getQuaternionFromEuler([0, ramp_angle, 0]),
    )



def update_camera(car_position, yaw, dt):
    global camera_rotation_angle

    if CAMERA_TYPE == "ROTATE_AROUND_CAR":
        angular_speed = math.radians(30.0)
        camera_rotation_angle = (camera_rotation_angle + angular_speed * dt) % (
            2 * math.pi
        )

        orbit_radius = 7.0
        orbit_height = 3.0
        camera_distance = math.sqrt(orbit_radius**2 + orbit_height**2)
        camera_yaw = math.degrees(camera_rotation_angle)
        camera_pitch = -math.degrees(math.atan2(orbit_height, orbit_radius))
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=car_position,
        )
        return

    if CAMERA_TYPE != "FOLLOW_CAR":
        return

    follow_distance = 5.0
    follow_height = 2.0
    camera_distance = math.sqrt(follow_distance**2 + follow_height**2)
    camera_yaw = math.degrees(yaw) - 70
    camera_pitch = -math.degrees(math.atan2(follow_height, follow_distance))
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=car_position,
    )
def run_simulation(car):
    dt = 1 / 240

    while True:
        turn, brake, throttle = get_next_move()
        car.apply_steering(turn)
        car.update_vehicle_state(turn, brake, throttle, dt)

        p.stepSimulation()
        time.sleep(dt)


def main():
    initialize_simulation()
    setup_environment()
    car = Car()
    car.apply_steering(0.0)

    try:
        run_simulation(car)
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()