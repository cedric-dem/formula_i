import sys
from pathlib import Path
import numpy as np
from PIL import Image
import open3d as o3d
import random

from .config import *

random.seed(123456)

def get_model():
	MODEL_PATH = Path(path_str)

	if not MODEL_PATH.exists():
		print(f"Error: file not found: {MODEL_PATH.resolve()}")
		sys.exit(1)

	model = o3d.io.read_triangle_model(MODEL_PATH)

	if model is None:
		print("Error: failed to load model (unsupported/corrupted GLB).")
		sys.exit(1)

	return get_resized_model(model)

def create_markers_list():
	print("==> layout image path : ", image_layout_path)

	path = image_layout_path
	img = Image.open(path).convert("RGB")
	width, height = img.size

	if width != height:
		raise Exception("not square image")

	im_size = width
	pixels = img.load()

	start, end = detect_start_and_end_point(im_size, pixels, HALF_SIZE)
	print("====> start,end", start, end)

	trail_composition = get_trail_shape(im_size, pixels, HALF_SIZE)
	random.shuffle(trail_composition)
	print('====> trail_composition size', len(trail_composition))

	if SUBSET_TRAIL_SIZE == None:
		resulting_layout = merge_start_trail_end(start, trail_composition, end)
	else:
		resulting_layout = merge_start_trail_end(start, trail_composition[:SUBSET_TRAIL_SIZE], end)

	print("=> start : ", resulting_layout[:10])
	print("=> mid : ", resulting_layout[len(resulting_layout) // 2])
	print("=> end : ", resulting_layout[-10:])
	print("====> Final result layout : ", len(resulting_layout))  # , ": ", resulting_layout)

	return resulting_layout

def adapt_markers_to_model(model, trackers_information):
	triangles_set = get_complete_list_of_triangles(model)

	if SUBSET_TRIANGLES_SIZE == None:
		print("===> triangles qty", len(triangles_set))
	else:
		print("===> triangles qty", len(triangles_set), "but will reduce to ", SUBSET_TRIANGLES_SIZE)
		triangles_set = triangles_set[:SUBSET_TRIANGLES_SIZE]

	track_layout_markers = get_track_layout_markers(trackers_information, triangles_set)
	return track_layout_markers

def display_map_and_markers(model, track_layout_markers):
	if DISPLAY_RESULT == "IN_3D_WINDOW":
		display_result_in_3d_window(model, track_layout_markers)

	elif DISPLAY_RESULT == "SUMMARIZE_IN_TERMINAL":
		summarize_result_in_terminal(model, track_layout_markers)

def create_cube(size, color, position):
	new_cube = o3d.geometry.TriangleMesh.create_box(width = size[0], height = size[1], depth = size[2])
	new_cube.compute_vertex_normals()
	new_cube.paint_uniform_color(color)
	position_adapted = [position[0] - (size[0] // 2), position[1] - (size[1] // 2), position[2] - (size[2] // 2)]
	new_cube.translate(position_adapted)
	# new_cube.translate(position)
	return new_cube

def detect_start_and_end_point(im_size, pixels, delta_coord):
	red_pixel = None
	green_pixel = None

	y = 0
	while y < im_size - 1 and (red_pixel is None or green_pixel is None):
		x = 0
		while x < im_size - 1 and (red_pixel is None or green_pixel is None):
			r, g, b = pixels[x, y]
			if r > 200 and g < 80 and b < 80:
				red_pixel = (x - delta_coord, y - delta_coord)
			elif r < 80 and g > 200 and b < 80:
				green_pixel = (x - delta_coord, y - delta_coord)
			x += 1
		y += 1
	return [red_pixel, green_pixel]

def get_trail_shape(im_size, pixels, delta_coord):
	result = [
		[x - delta_coord, y - delta_coord]
		for y in range(im_size)
		for x in range(im_size)
		# if ((pixels[x, y][0] < 50 and pixels[x, y][1] < 50 and pixels[x, y][2] < 50) or (pixels[x, y][0] > 200 and pixels[x, y][1] < 80 and pixels[x, y][2] < 80) or (pixels[x, y][0] < 80 and pixels[x, y][1] > 200 and pixels[x, y][2] < 80))
		if ((pixels[x, y][0] < 125 and pixels[x, y][1] < 128 and pixels[x, y][2] < 128) or (pixels[x, y][0] > 128 and pixels[x, y][1] < 128 and pixels[x, y][2] < 128) or (pixels[x, y][0] < 128 and pixels[x, y][1] > 128 and pixels[x, y][2] < 128))
	]
	return result

def merge_start_trail_end(start, trail_composition, end):
	result = [["START", start[0], start[1]]]

	unexplored_pixels = [[start[0], start[1]]]
	explored_pixels = []

	current_index = 0
	continue_exploration = True

	while continue_exploration:
		# print('====> new step : ',len(unexplored_pixels), len(explored_pixels), unexplored_pixels[0])

		elem_to_treat = unexplored_pixels.pop(0)
		explored_pixels.append(elem_to_treat)
		result.append([current_index, elem_to_treat[0], elem_to_treat[1]])

		if CUT_BFS:  # for debug
			unexplored_pixels.append(trail_composition[0])
			random.shuffle(trail_composition)
			random.shuffle(unexplored_pixels)

		else:
			for neighbour in [[0, 1], [0, -1], [1, 0], [-1, 0]]:
				# for neighbour in [[0, 1], [0, -1], [1, 0], [-1, 0], [1,1], [1,-1], [-1,1], [-1,-1]]:
				# for neighbour in [[0, 1], [0, -1], [1, 0], [-1, 0], [1,1], [1,-1], [-1,1], [-1,-1], [-2,0 ], [2,0], [0, -2], [0,2]]:
				next_coord = [elem_to_treat[0] + neighbour[0], elem_to_treat[1] + neighbour[1]]
				# print("=====> ",explored_pixels[0],trail_composition[0], unexplored_pixels[0], next_coord)
				if (next_coord not in unexplored_pixels) and (next_coord not in explored_pixels) and (next_coord in trail_composition):
					unexplored_pixels.append(next_coord)

		current_index += 1

		if STOP_EXPLORATION_EARLY == None:
			continue_exploration = len(unexplored_pixels) > 0
		else:
			continue_exploration = current_index < STOP_EXPLORATION_EARLY

	result.append(["END", end[0], end[1]])

	return result

def get_index_of_triangle_intersecting_with_cube_on_y_axis(triangles_set, tracker_info):
	# to improve precision : could find intersection in the triangle but for now closes t point will do the job
	best_distance = None
	best_index = None

	for current_triangle_index in range(len(triangles_set)):
		for current_point in triangles_set[current_triangle_index]:
			this_distance = get_distance(current_point, tracker_info[1:])
			if best_distance == None or this_distance < best_distance:
				best_distance = this_distance
				best_index = current_triangle_index
	# print("'==> ",best_index, best_distance)
	return best_index

def get_distance(mesh_3d_point, tracker_coord_2d):
	# return random.randint(0,12)
	return ((((tracker_coord_2d[0] - mesh_3d_point[0]) ** 2)) + ((tracker_coord_2d[1] - mesh_3d_point[2]) ** 2)) ** 0.5

def display_all_triangles(triangles_set, markers_objects_scene):
	display_triangles_all = "none"  # "only_center", "three_points", "none"
	cube_size = 10
	for corresponding_triangle in triangles_set:

		if display_triangles_all == "only_center":
			color = [0, 1, 0]
			avg = [(corresponding_triangle[0][0] + corresponding_triangle[1][0] + corresponding_triangle[2][0]) / 3, (corresponding_triangle[0][1] + corresponding_triangle[1][1] + corresponding_triangle[2][1]) / 3,
				   (corresponding_triangle[0][2] + corresponding_triangle[1][2] + corresponding_triangle[2][2]) / 3]

			markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, avg))

		elif display_triangles_all == "three_points":
			color = [random.randint(0, 100) / 100, random.randint(0, 100) / 100, random.randint(0, 100) / 100]

			markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, corresponding_triangle[0]))
			markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, corresponding_triangle[1]))
			markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, corresponding_triangle[2]))

		elif display_triangles_all == "none":
			pass

def display_augmented_trackers(trackers_information, triangles_set, markers_objects_scene):
	cube_size = 10

	for current_tracker in trackers_information:
		triangle_index = get_index_of_triangle_intersecting_with_cube_on_y_axis(triangles_set, current_tracker)

		corresponding_triangle = triangles_set[triangle_index]

		# color = [random.randint(0, 100) / 100, random.randint(0, 100) / 100, random.randint(0, 100) / 100]
		color = [0, 1, 0]  # green = corresponding triangle

		# markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, corresponding_triangle[0]))
		# markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, corresponding_triangle[1]))
		# markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, corresponding_triangle[2]))

		avg = [(corresponding_triangle[0][0] + corresponding_triangle[1][0] + corresponding_triangle[2][0]) / 3, (corresponding_triangle[0][1] + corresponding_triangle[1][1] + corresponding_triangle[2][1]) / 3,
			   (corresponding_triangle[0][2] + corresponding_triangle[1][2] + corresponding_triangle[2][2]) / 3]

		markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, avg))

		# point_intersection_with_the_ground = [current_tracker[1], 12, current_tracker[2]]
		point_intersection_with_the_ground = [current_tracker[1], avg[1], current_tracker[2]]
		# print("'=====> ",current_tracker)

		# add cube to the gnd
		cube_landed = create_cube([cube_size, cube_size, cube_size], [1, 0, 0], point_intersection_with_the_ground)  # red : coord of trackers
		markers_objects_scene.append(cube_landed)

def add_4_corners_trackers(markers_objects_scene):
	color = [0, 0, 1]
	cube_size = 50
	d = HALF_SIZE
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [d, 0, d]))
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [-d, 0, d]))
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [d, 0, -d]))
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [-d, 0, -d]))

def augment_layout_markers_using_triangles_list(trackers_information, triangles_set):
	markers_objects_scene = []
	print("triangles set :", len(triangles_set))

	print("===> trackers", len(trackers_information))

	# display all triangles, or not
	display_all_triangles(triangles_set, markers_objects_scene)

	# display trackers maybe on the ground
	display_augmented_trackers(trackers_information, triangles_set, markers_objects_scene)

	## add markers at 4 corners,
	add_4_corners_trackers(markers_objects_scene)

	return markers_objects_scene

def get_track_layout_markers(trackers_information, triangles_list):
	markers_position_3d = augment_layout_markers_using_triangles_list(trackers_information, triangles_list)
	print("===> Markers in 3D : ", len(markers_position_3d))

	return markers_position_3d

def get_complete_list_of_triangles(model):  # to be merged later with upper function
	lst = get_upper_map_list_of_triangles(model)
	random.shuffle(lst)
	# {print('======> size', len(lst))
	return lst  # for now, using a subset to debug faster

def get_initial_list_of_triangles(model, i):
	mesh = model.meshes[i].mesh

	vertices = np.asarray(mesh.vertices)
	faces = np.asarray(mesh.triangles)

	triangles = [vertices[tri].tolist() for tri in faces]
	return triangles

def get_upper_map_list_of_triangles(model):
	return get_initial_list_of_triangles(model, 1)

def get_original_map_vertices(model):
	# triangles_list = get_initial_list_of_triangles(model, 0) #-3464.262451171875 3464.262451171875 -3464.27783203125 3464.27783203125
	triangles_list = get_initial_list_of_triangles(model, 1)  # -3464.262451171875 3464.262451171875 -3464.27783203125 3464.27783203125

	max_x = -99999
	max_z = -99999
	min_x = 99999
	min_z = 99999

	for current_triangle in triangles_list:
		for vertice in current_triangle:
			this_x = vertice[0]
			this_z = vertice[2]

			if this_x > max_x:
				max_x = this_x
			if this_x < min_x:
				min_x = this_x
			if this_z > max_z:
				max_z = this_z
			if this_z < min_z:
				min_z = this_z

	print("=======================> vertices ", min_x, max_x, min_z, max_z)
	return [min_x, max_x, min_z, max_z]

def get_resized_model(model):
	min_x, max_x, min_z, max_z = get_original_map_vertices(model)

	current_range_x = max_x - min_x
	current_range_z = max_z - min_z

	if current_range_x == 0 or current_range_z == 0:
		raise ValueError("Invalid map dimensions: zero width or height detected.")

	target_min = -HALF_SIZE
	target_max = HALF_SIZE
	target_span = target_max - target_min

	scale_x = target_span / current_range_x
	scale_z = target_span / current_range_z

	for geom in model.meshes:
		mesh = geom.mesh
		mesh_vertices = np.asarray(mesh.vertices)

		transformed_vertices = mesh_vertices.copy()
		transformed_vertices[:, 0] = (mesh_vertices[:, 0] - min_x) * scale_x + target_min
		transformed_vertices[:, 2] = (mesh_vertices[:, 2] - min_z) * scale_z + target_min

		mesh.vertices = o3d.utility.Vector3dVector(transformed_vertices)

	# display new vertices
	get_original_map_vertices(model)

	return model

def display_result_in_3d_window(model, track_layout_markers):
	try:
		app = o3d.visualization.gui.Application.instance
		app.initialize()

		win = o3d.visualization.O3DVisualizer(title = "view", width = 1280, height = 800)

		col = np.array((0, 0, 0, 1), dtype = np.float32)

		if hasattr(win, "set_background"):
			win.set_background(col, o3d.geometry.Image())

		win.add_model("title_here", model)

		# Frame camera
		if hasattr(win, "reset_camera_to_default"):
			win.reset_camera_to_default()

		app.add_window(win)
		app.run()

	except Exception as e:
		print(f"[Info] O3DVisualizer had issues ({e}). Falling back to simple viewer.")
		try:
			o3d.visualization.draw([model] + track_layout_markers)
		except Exception as e2:
			print(f"Fallback viewer failed as well: {e2}")
			sys.exit(1)

def summarize_result_in_terminal(model, track_layout_markers):
	# display min height, max height, track length etc
	pass
