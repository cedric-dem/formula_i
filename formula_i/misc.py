import math
import time
import random

import pybullet as p

from formula_i.config import *



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
    p.connect(p.GUI, options='--background_color_red='+str(SKY_COLOR[0])+' --background_color_green='+str(SKY_COLOR[1])+' --background_color_blue='+str(SKY_COLOR[2]))
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

    if CURRENT_TRACK == "TESTING_GRID":
        _setup_testing_grid_environment()
    elif CURRENT_TRACK == "STRAIGHT_LINE":
        _setup_straight_line_environment()
    elif CURRENT_TRACK == "OVAL_RACE_TRACK":
        _setup_oval_race_track_environment()
    else:
        raise ValueError(f"Unsupported track configuration: {CURRENT_TRACK}")

    if ADD_DEBUG_CUBES:
        cube_half_extents = [1.0, 1.0, 1.0]
        cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=cube_half_extents)
        cube_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=cube_half_extents,
            rgbaColor=[1.0, 0.0, 0.0, 1.0],
        )

        for _ in range(10):
            random_position = [random.uniform(-500.0, 500.0), random.uniform(-500.0, 500.0), 1.0]
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=cube_collision,
                baseVisualShapeIndex=cube_visual,
                basePosition=random_position,
            )



def _setup_testing_grid_environment():
    _create_ground_plane()
    index = 0
    for height, lateral_shift in ((6, 10), (12, 20), (18, 30)):
        index +=1
        create_track_portion(
            ramp_height=height,
            lateral_offset=lateral_shift,
            color=get_track_color(index),
        )

def _setup_oval_race_track_environment():
    segment_width = 10.0
    segment_thickness = 0.5

    major_radius = 110.0
    minor_radius = 60.0

    num_segments = 160
    angles = [2.0 * math.pi * index / num_segments for index in range(num_segments)]
    angle_phase = -math.pi / 2.0

    for index, angle in enumerate(angles):
        param_angle = angle + angle_phase

        center_x = major_radius * math.cos(param_angle)
        center_y = minor_radius * math.sin(param_angle) + minor_radius

        next_angle = angles[(index + 1) % num_segments] + angle_phase
        next_x = major_radius * math.cos(next_angle)
        next_y = minor_radius * math.sin(next_angle) + minor_radius

        segment_vector_x = next_x - center_x
        segment_vector_y = next_y - center_y
        segment_distance = math.hypot(segment_vector_x, segment_vector_y)

        tangent_yaw = math.atan2(
            minor_radius * math.cos(param_angle), -major_radius * math.sin(param_angle)
        )

        create_track_portion(
            length=max(segment_distance * 1.25, segment_width * 0.75),
            width=segment_width,
            thickness=segment_thickness,
            center_x=center_x,
            lateral_offset=center_y,
            color=get_track_color(index),
            orientation_yaw=tangent_yaw,
        )

def get_track_color(index):
    if (TRACK_COLOR_SEGMENTS == "RANDOM"):
        color = [random.randint(0, 10) / 10, random.randint(0, 10) / 10, random.randint(0, 10) / 10]
    elif (TRACK_COLOR_SEGMENTS == "ALTERNATING"):
        color = ([ROAD_COLOR[0], ROAD_COLOR[1], ROAD_COLOR[2]],[ROAD_COLOR[0] * 0.9, ROAD_COLOR[1] * 0.9, ROAD_COLOR[2] * 0.9])[index % 2]
    elif (TRACK_COLOR_SEGMENTS == "SAME"):
        color = ROAD_COLOR
    return color



def _setup_straight_line_environment():
    segment_length = 30.0
    segment_width = 6.0
    segment_thickness = 0.5

    for index in range(20):
        center_x = index * (segment_length-0.1)
        create_track_portion(
            length=segment_length,
            width=segment_width,
            thickness=segment_thickness,
            center_x=center_x,
            color=get_track_color(index),
        )


def _create_ground_plane():
    ground_half_extents = [1000.0, 1000.0, 0.1]
    ground_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=ground_half_extents
    )
    ground_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=ground_half_extents,
        rgbaColor=[GROUND_COLOR[0], GROUND_COLOR[1], GROUND_COLOR[2], 1],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=ground_collision,
        baseVisualShapeIndex=ground_visual,
        basePosition=[0, 0, -ground_half_extents[2]],
    )


def create_track_portion(
    length=30.0,
    width=6.0,
    thickness=0.5,
    *,
    center_x=0.0,
    lateral_offset=0.0,
    base_height_offset=0.0,
    color=None,
    ramp_height=None,
    ramp_start_offset_x=15.0,
    ramp_start_depth=-2.0,
    orientation_yaw=0.0,
):
    """Create a track portion with optional ramp or yaw characteristics."""

    half_length = length / 2.0
    half_width = width / 2.0
    half_thickness = thickness / 2.0

    color = ROAD_COLOR if color is None else color

    if ramp_height is not None:
        if orientation_yaw != 0.0:
            raise ValueError("Ramps cannot be rotated around the vertical axis")
        ramp_angle = -math.atan2(ramp_height, length)
        cos_angle = math.cos(ramp_angle)
        sin_angle = math.sin(ramp_angle)

        bottom_local_x = -half_length
        bottom_local_z = -half_thickness

        center_x = ramp_start_offset_x - (
            bottom_local_x * cos_angle + bottom_local_z * sin_angle
        )
        center_z = ramp_start_depth - (
            -bottom_local_x * sin_angle + bottom_local_z * cos_angle
        )
        orientation_quaternion = p.getQuaternionFromEuler(
            [0, ramp_angle, orientation_yaw]
        )
    else:
        center_z = base_height_offset - half_thickness
        orientation_quaternion = p.getQuaternionFromEuler([0, 0, orientation_yaw])

    portion_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[half_length, half_width, half_thickness],
    )
    portion_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[half_length, half_width, half_thickness],
        rgbaColor=[color[0], color[1], color[2], 1],
    )

    portion_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=portion_collision,
        baseVisualShapeIndex=portion_visual,
        basePosition=[center_x, lateral_offset, center_z],
        baseOrientation=orientation_quaternion,
    )

    p.changeDynamics(
        portion_id,
        -1,
        contactStiffness=1_000,
        contactDamping=5,
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



def display_data(values):
    if DISPLAY_DATA == "ON_TERMINAL":
        display_on_terminal(values)
    elif DISPLAY_DATA == "ON_GUI":
        display_on_gui(values)


def display_on_terminal(values_dict):
    for value in values_dict:
        print(value, values_dict[value], end=", ")
    print()


def display_on_gui(values_dict):
    # TODO
    pass

def get_input_data(car, current_ticks):
    car_position, car_orientation = p.getBasePositionAndOrientation(car.body_id)
    linear_velocity, _ = p.getBaseVelocity(car.body_id)
    current_angle_x, current_angle_y, current_angle_z = p.getEulerFromQuaternion(
        car_orientation
    )
    overall_speed_ms = math.sqrt(
        linear_velocity[0] ** 2
        + linear_velocity[1] ** 2
        + linear_velocity[2] ** 2
    )

    overall_speed_kmh = overall_speed_ms * 3.6

    last_position = getattr(car, "_last_display_position", None)

    if last_position is not None:
        distance_increment = math.sqrt((car_position[0] - last_position[0]) ** 2+ (car_position[1] - last_position[1]) ** 2+ (car_position[2] - last_position[2]) ** 2        )
        car.total_distance_traveled += distance_increment

    car._last_display_position = car_position

    values = {
        "ticks": current_ticks,

        "position_x": round(float(car_position[0]),2),
        "position_y": round(float(car_position[1]),2),
        "position_z": round(float(car_position[2]),2),
        "speed_x": round(float(linear_velocity[0]),2),
        "speed_y": round(float(linear_velocity[1]),2),
        "speed_z": round(float(linear_velocity[2]),2),
        "angle_x": round(float(current_angle_x),2),
        "angle_y": round(float(current_angle_y),2),
        "angle_z": round(float(current_angle_z),2),

        "distance_traveled": round(float(car.total_distance_traveled),2),
        "overall_speed_kmh": round(float(overall_speed_kmh),2),
    }
    return values

def get_str_padded_data(input_data):
    result = {}
    for key in input_data:
        tmp = str(input_data[key])
        result[key] = ((7-len(tmp)) * " " )+ tmp
    return result

def run_simulation(car):
    dt = 1 / 240

    current_ticks = 0
    while True:
        current_ticks += 1
        turn, brake, throttle = get_next_move()
        car.apply_steering(turn)
        car.update_vehicle_state(turn, brake, throttle, dt, current_ticks)

        if DISPLAY_DATA != "NO":
            if current_ticks % REFRESH_EVERY_FRAME == 0:
                input_data = get_input_data(car, current_ticks) #probably will have to move that to be used also in the input of the nn
                display_data(get_str_padded_data(input_data))

        p.stepSimulation()
        time.sleep(dt)

