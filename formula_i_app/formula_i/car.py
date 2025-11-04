import math

from .misc import *
import pybullet as p

class Car:
	def __init__(self):
		self.body_id = self._create_car()
		self.current_speed_ms = 0.0
		_, orientation = p.getBasePositionAndOrientation(self.body_id)
		_, _, self.current_angle = p.getEulerFromQuaternion(orientation)
		self.current_steering = 0.0
		self.total_distance_traveled = 0.0
		self._identity_orientation = p.getQuaternionFromEuler([0, 0, 0])
		self.sensors = self._initialize_sensors()
		self.track_limits = get_track_layout_as_matrix()

	def apply_steering(self, steering_input):
		MAX_STEERING_ANGLE = 10.0  # only for visual
		steering_angle = clamp(float(steering_input), -1.0, 1.0) * math.radians(MAX_STEERING_ANGLE)
		self.current_steering = steering_angle

		for joint_index in (0, 1):
			self._update_wheel_joint(joint_index, steering_angle)

	def _update_wheel_joint(self, joint_index, steering_angle):
		p.resetJointState(self.body_id, joint_index, steering_angle)
		p.setJointMotorControl2(
			self.body_id,
			joint_index,
			controlMode = p.POSITION_CONTROL,
			targetPosition = steering_angle,
			force = 100,
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
			baseMass = 2,
			baseCollisionShapeIndex = car_body_collision,
			baseVisualShapeIndex = car_body_visual,
			basePosition = list(CAR_INITIAL_POSITION),
			baseOrientation = p.getQuaternionFromEuler([0, 0, math.radians(INITIAL_ANGLE_OF_CAR)]),
			linkMasses = link_masses,
			linkCollisionShapeIndices = link_collision_indices,
			linkVisualShapeIndices = link_visual_indices,
			linkPositions = link_positions,
			linkOrientations = link_orientations,
			linkInertialFramePositions = link_inertial_positions,
			linkInertialFrameOrientations = link_inertial_orientations,
			linkParentIndices = link_parent_indices,
			linkJointTypes = link_joint_types,
			linkJointAxis = link_joint_axes,
		)

		p.changeDynamics(
			car_body,
			-1,
			lateralFriction = 1.0,
			contactStiffness = CAR_CONTACT_STIFFNESS,
			contactDamping = CAR_CONTACT_DAMPING,
		)
		for link_index in range(p.getNumJoints(car_body)):
			p.changeDynamics(
				car_body,
				link_index,
				lateralFriction = 1.0,
				contactStiffness = CAR_CONTACT_STIFFNESS,
				contactDamping = CAR_CONTACT_DAMPING,
			)

		for joint_index in range(2):
			p.setJointMotorControl2(
				car_body,
				joint_index,
				controlMode = p.VELOCITY_CONTROL,
				targetVelocity = 0,
				force = 0,
			)

		return car_body

	def _create_sensor_marker(self):
		marker_half_extents = [0.2, 0.2, 0.2]
		marker_visual = p.createVisualShape(
			p.GEOM_BOX, halfExtents = marker_half_extents, rgbaColor = [0.0, 1.0, 0.0, 1.0]
		)

		marker_id = p.createMultiBody(
			baseMass = 0,
			baseCollisionShapeIndex = -1,
			baseVisualShapeIndex = marker_visual,
			basePosition = [0.0, 0.0, -10.0],
			baseOrientation = self._identity_orientation,
		)

		return marker_id

	def _initialize_sensors(self):
		return [self._create_sensor_definition(angle_deg) for angle_deg in SENSORS_ANGLE_LIST]

	def _create_sensor_definition(self, angle_deg):
		return {
			"name": self._sensor_name(angle_deg),
			"angle_offset": math.radians(angle_deg),
			"distance": (MAX_SENSOR_DISTANCE + MIN_SENSOR_DISTANCE) / 2,
			"marker_id": self._create_sensor_marker() if DISPLAY_SENSORS else None,
		}

	def _sensor_name(self, angle_deg):
		if angle_deg == 0:
			return "front_0"
		if angle_deg > 0:
			return f"front_left_{angle_deg}"
		if angle_deg < 0:
			return f"front_right_{angle_deg}"
		return ""

	def _create_body(self, base_block):
		base_delta_x, base_delta_z = base_block[:2]
		base_color = list(base_block[-1]) + [1.0]
		base_half_extents = [
			base_block[2] / 2.0,
			base_block[3] / 2.0,
			base_block[4] / 2.0,
		]

		car_body_collision = p.createCollisionShape(
			p.GEOM_BOX, halfExtents = base_half_extents
		)
		car_body_visual = p.createVisualShape(
			p.GEOM_BOX, halfExtents = base_half_extents, rgbaColor = base_color
		)

		return car_body_collision, car_body_visual, base_delta_x, base_delta_z

	def _create_wheels(self):
		wheel_radius = 0.321
		wheel_width = 0.350

		wheel_collision = p.createCollisionShape(
			p.GEOM_CYLINDER, radius = wheel_radius, height = wheel_width
		)
		wheel_visual = p.createVisualShape(
			p.GEOM_CYLINDER,
			radius = wheel_radius,
			length = wheel_width,
			rgbaColor = [WHEELS_COLOR[0], WHEELS_COLOR[1], WHEELS_COLOR[2], 1],
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
				p.GEOM_BOX, halfExtents = half_extents
			)

			block_visual = p.createVisualShape(
				p.GEOM_BOX,
				halfExtents = half_extents,
				rgbaColor = list(block[-1]) + [1.0],
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

	def update_vehicle_state(self, turn, brake, throttle, dt, current_timestamp):
		turn, brake, throttle = self._clamp_controls(turn, brake, throttle)
		self._update_speed(brake, throttle, dt)
		self._update_heading(turn, dt)

		car_position, physics_orientation = p.getBasePositionAndOrientation(self.body_id)
		current_linear_velocity, current_angular_velocity = p.getBaseVelocity(self.body_id)

		car_orientation = self._compute_orientation(physics_orientation)
		p.resetBasePositionAndOrientation(self.body_id, car_position, car_orientation)

		planar_forward_vector = self._planar_forward_vector()
		if current_timestamp % REFRESH_SENSORS_EVERY_FRAME == 0:
			self._update_sensors(car_position, planar_forward_vector)

		vehicle_forward_vector = self._vehicle_forward_vector(car_orientation)
		self._apply_gravity_effect(vehicle_forward_vector, dt)

		residual_velocity = self._residual_velocity(current_linear_velocity, vehicle_forward_vector)
		linear_velocity = self._compose_linear_velocity(vehicle_forward_vector, residual_velocity)

		p.resetBaseVelocity(
			self.body_id,
			linearVelocity = linear_velocity,
			angularVelocity = current_angular_velocity,
		)

		update_camera(car_position, self.current_angle, dt)

		return self.current_angle, self.current_speed_ms

	def _clamp_controls(self, turn, brake, throttle):
		turn = clamp(float(turn), -1.0, 1.0)
		brake = clamp(float(brake), 0.0, 1.0)
		throttle = clamp(float(throttle), 0.0, 1.0)
		return turn, brake, throttle

	def _update_speed(self, brake, throttle, dt):
		acceleration_rate = 0.3
		brake_deceleration = 50.0

		target_speed_mps = throttle * MAX_SPEED_MPS
		speed_delta_mps = (target_speed_mps - self.current_speed_ms) * acceleration_rate * dt
		self.current_speed_ms += speed_delta_mps
		self.current_speed_ms = max(self.current_speed_ms - brake * brake_deceleration * dt, 0.0)

	def _update_heading(self, turn, dt):
		turn_speed = 1.0
		if self.current_speed_ms > 0.01:
			self.current_angle += turn * turn_speed * dt

	def _compute_orientation(self, physics_orientation):
		roll, pitch, _ = p.getEulerFromQuaternion(physics_orientation)
		return p.getQuaternionFromEuler([roll, pitch, self.current_angle])

	def _planar_forward_vector(self):
		return [
			math.cos(self.current_angle),
			math.sin(self.current_angle),
			0.0,
		]

	def _vehicle_forward_vector(self, car_orientation):
		orientation_matrix = p.getMatrixFromQuaternion(car_orientation)
		return [
			orientation_matrix[0],
			orientation_matrix[3],
			orientation_matrix[6],
		]

	def _apply_gravity_effect(self, vehicle_forward_vector, dt):
		gravity_vector = [0, 0, -GRAV]
		gravity_along_forward = sum(
			gravity_component * forward_component
			for gravity_component, forward_component in zip(gravity_vector, vehicle_forward_vector)
		)
		self.current_speed_ms = max(
			min(self.current_speed_ms + gravity_along_forward * dt, MAX_SPEED_MPS),
			0.0,
		)

	def _residual_velocity(self, current_linear_velocity, vehicle_forward_vector):
		current_speed_along_forward = sum(
			current_linear_velocity[i] * vehicle_forward_vector[i] for i in range(3)
		)
		return [
			current_linear_velocity[i]
			- current_speed_along_forward * vehicle_forward_vector[i]
			for i in range(3)
		]

	def _compose_linear_velocity(self, vehicle_forward_vector, residual_velocity):
		return [
			vehicle_forward_vector[i] * self.current_speed_ms + residual_velocity[i]
			for i in range(3)
		]

	def _update_sensors(self, car_position, forward_vector):

		sensor_origin = [car_position[0] + forward_vector[0], car_position[1] + forward_vector[1], car_position[2]]

		for sensor in self.sensors:
			direction = [
				math.cos(self.current_angle + sensor["angle_offset"]),
				math.sin(self.current_angle + sensor["angle_offset"]),
				0.0,
			]

			detected_point, detected_distance = self.point_in_that_direction(sensor_origin, direction[:2], sensor["distance"])
			# detected_distance = get_distance(detected_point, sensor_origin)

			sensor["distance"] = detected_distance

			if DISPLAY_SENSORS:
				p.resetBasePositionAndOrientation(
					sensor["marker_id"],
					detected_point,
					self._identity_orientation
				)

	def point_in_that_direction(self, starting_coord, sensor_direction, old_distance):

		norm = math.sqrt(sensor_direction[0] ** 2 + sensor_direction[1] ** 2)
		if norm == 0:
			raise ValueError("cant be null.")

		delta = [sensor_direction[0] / norm, sensor_direction[1] / norm]

		current_delta = 0
		found = None
		marker_dist = 0

		while ((old_distance + current_delta <= MAX_SENSOR_DISTANCE) or (old_distance - current_delta >= MIN_SENSOR_DISTANCE)) and (found is None):

			for current_direction_progression in [+1, -1]:
				old_dst = old_distance + current_direction_progression * (current_delta - 1)
				new_dst = old_distance + current_direction_progression * current_delta

				old_point = [starting_coord[0] + delta[0] * old_dst, starting_coord[1] + delta[1] * old_dst]
				new_point = [starting_coord[0] + delta[0] * new_dst, starting_coord[1] + delta[1] * new_dst]

				old_state = self.get_layout_state_at_point(old_point)
				new_state = self.get_layout_state_at_point(new_point)

				if old_state != new_state:
					found = new_point
					marker_dist = new_dst

			current_delta += 1

		if found is None:
			# if first value is false
			min_dist_point = [starting_coord[0] + delta[0] * MAX_SENSOR_DISTANCE, starting_coord[1] + delta[1] * MAX_SENSOR_DISTANCE]
			min_dist_state = self.get_layout_state_at_point(min_dist_point)
			if not min_dist_state:
				found = min_dist_point
				marker_dist = MAX_SENSOR_DISTANCE

			# could add else here for more optimization, put the code below in else

			# if last value is true
			max_dist_point = [starting_coord[0] + delta[0] * MAX_SENSOR_DISTANCE, starting_coord[1] + delta[1] * MAX_SENSOR_DISTANCE]
			max_dist_state = self.get_layout_state_at_point(max_dist_point)
			if max_dist_state:
				found = max_dist_point
				marker_dist = MIN_SENSOR_DISTANCE

			if found is None:
				# found = [starting_coord[0], starting_coord[1]]  # to avoid crash ? not sure
				raise ValueError("not found sensor")

		marker_pos = found + [starting_coord[2]]
		return marker_pos, marker_dist  # just for debug show correct height

	def get_layout_state_at_point(self, point):
		# todo discover why the minus point[1] is necessary, guess its related to the handling of xyz axes
		return self.track_limits[int(point[0]) + HALF_MAP_SIZE][-int(point[1]) + HALF_MAP_SIZE]
