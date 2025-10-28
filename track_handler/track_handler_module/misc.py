import sys
from pathlib import Path
import numpy as np
from PIL import Image
import open3d as o3d
import random
import csv

from .config import *

random.seed(123456)

def in_csv_file(data, filename):
	with open(filename, mode = 'w', newline = '', encoding = 'utf-8') as file:
		writer = csv.writer(file)
		writer.writerows(data)

def read_csv_file(filename):
	data = []
	with open(filename, mode = 'r', newline = '', encoding = 'utf-8') as file:
		reader = csv.reader(file)
		for row in reader:
			converted_row = []
			for i, value in enumerate(row):
				if i in (0, 2):
					converted_row.append(value)
				elif i in (1, 3):
					converted_row.append(int(value))
			data.append(converted_row)
	return data

def get_model():
	print('===> Reading model')
	MODEL_PATH = Path(path_str)

	if not MODEL_PATH.exists():
		print(f"Error: file not found: {MODEL_PATH.resolve()}")
		sys.exit(1)

	model = o3d.io.read_triangle_model(MODEL_PATH)

	if model is None:
		print("Error: failed to load model (unsupported/corrupted GLB).")
		sys.exit(1)

	print('===> finished reading model, resizing it')
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

def adapt_markers_to_model(model, FILENAME_TEMP_LAYOUT):
	triangles_set = get_list_of_triangles(model)
	if SUBSET_TRIANGLES_SIZE == None:
		print("===> triangles qty", len(triangles_set))
	else:
		print("===> triangles qty", len(triangles_set), "but will reduce to ", SUBSET_TRIANGLES_SIZE)
		triangles_set = triangles_set[:SUBSET_TRIANGLES_SIZE]

	current_layout_markers_adapted = read_csv_file(FILENAME_TEMP_LAYOUT)

	continue_exploration = True
	current_loop = 0
	total = len(current_layout_markers_adapted)

	while continue_exploration:
		# small step progress in adapting layout
		augment_layout_markers_using_triangles_list(current_layout_markers_adapted, triangles_set, ADAPT_QUANTITY)

		# write in csv file
		in_csv_file(current_layout_markers_adapted, FILENAME_TEMP_LAYOUT)

		# check if needed to continue
		current_loop += 1

		remaining = count_remaining(current_layout_markers_adapted)

		continue_exploration = remaining > 0

		print(">>>> current loop this execution : ", current_loop, " total quantity :", total, " remaining", remaining, " continue ", continue_exploration)

	print('===> Finished exploration')

def count_remaining(current_trackers):
	count = 0
	for row in current_trackers:
		if isinstance(row[2], str) and row[2].startswith("?"):
			count += 1
	return count

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
	unexplored_pixels = [[start[0], start[1]]]
	explored_pixels = []

	current_index = 0
	continue_exploration = True

	while continue_exploration:
		# print('====> new step : ',len(unexplored_pixels), len(explored_pixels), unexplored_pixels[0])

		if CUT_BFS:  # for debug
			unexplored_pixels.append(trail_composition[0])
			random.shuffle(trail_composition)
			random.shuffle(unexplored_pixels)

		else:
			elem_to_treat = unexplored_pixels.pop(0)
			explored_pixels.append(elem_to_treat)

			for neighbour in [[0, 1], [0, -1], [1, 0], [-1, 0]]:
				next_coord = [elem_to_treat[0] + neighbour[0], elem_to_treat[1] + neighbour[1]]
				# print("=====> ",explored_pixels[0],trail_composition[0], unexplored_pixels[0], next_coord)
				if (next_coord not in explored_pixels) and (next_coord not in unexplored_pixels) and (next_coord in trail_composition):
					unexplored_pixels.append(next_coord)

		current_index += 1

		if STOP_EXPLORATION_LAYOUT_PATH_EARLY == None:
			continue_exploration = len(unexplored_pixels) > 0
		else:
			continue_exploration = current_index < STOP_EXPLORATION_LAYOUT_PATH_EARLY

	result = [["START", start[0], "?", start[1]]]
	for i in range(len(explored_pixels)):
		result.append([i, explored_pixels[i][0], "?", explored_pixels[i][1]])
	result.append(["END", end[0], "?", end[1]])

	return result

def get_distance(mesh_3d_point, tracker_coord_2d):
	# print('====> dist ', mesh_3d_point, tracker_coord_2d)
	# return random.randint(0,12)
	return ((((tracker_coord_2d[0] - mesh_3d_point[0]) ** 2)) + ((tracker_coord_2d[1] - mesh_3d_point[2]) ** 2)) ** 0.5

def add_4_corners_trackers(markers_objects_scene):
	color = [0, 0, 1]
	cube_size = 50
	d = HALF_SIZE
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [d, 0, d]))
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [-d, 0, d]))
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [d, 0, -d]))
	markers_objects_scene.append(create_cube([cube_size, cube_size, cube_size], color, [-d, 0, -d]))

def augment_layout_markers_using_triangles_list(current_adapted_track_layout, triangles_set, quantity):
	current_index = 0
	current_found = 0
	continue_exploration = True
	while continue_exploration:

		if isinstance(current_adapted_track_layout[current_index][2], str) and current_adapted_track_layout[current_index][2].startswith("?"):
			# adapt that one
			height = adapt_one_marker_to_model(current_adapted_track_layout[current_index], triangles_set)
			current_adapted_track_layout[current_index][2] = height

			current_index += 1
			current_found += 1

		else:
			current_index += 1

		continue_exploration = (current_found < quantity) and not (current_index >= len(current_adapted_track_layout))

def adapt_one_marker_to_model(marker_to_adapt, triangles_set):
	# to improve precision : could find intersection in the triangle but for now closes t point will do the job

	# get_index_of_triangle_intersecting_with_cube_on_y_axis
	best_distance = None
	best_point = None

	for current_triangle_index in range(len(triangles_set)):
		for current_point in triangles_set[current_triangle_index]:
			this_distance = get_distance(current_point, [marker_to_adapt[1], marker_to_adapt[3]])
			if best_distance == None or this_distance < best_distance:
				best_distance = this_distance
				best_point = current_point
	# print("'==> ",best_point, best_distance)

	# if best distance is too far, could also take the middle of the triangle
	return round(best_point[1], 3)

def get_list_of_triangles(model):  # to be merged later with upper function
	list_of_triangles = get_upper_map_list_of_triangles(model)
	random.shuffle(list_of_triangles)
	return list_of_triangles

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

def get_scene_cube_markers(track_layout_markers):
	cubes_list = []

	# cubes not adapted : red
	cubes_list += get_cubes_scene_not_adapted(track_layout_markers)

	# cubes adapted : green
	cubes_list += get_cubes_scene_adapted(track_layout_markers)

	# markers border scene  : blue
	cubes_list += get_border_scene_cube_markers()

	return cubes_list

def get_cubes_scene_not_adapted(track_layout_markers):
	cube_size = 7
	cube_height = 100
	proportion_to_show = 0.01
	cubes_list = []
	for current_marker_index in range(len(track_layout_markers)):
		if isinstance(track_layout_markers[current_marker_index][2], str) and track_layout_markers[current_marker_index][2].startswith("?"):
			if random.random() < proportion_to_show:
				this_position = [track_layout_markers[current_marker_index][1], cube_height, track_layout_markers[current_marker_index][3]]
				new_cube = create_cube([cube_size, cube_size, cube_size], [1, 0, 0], this_position)
				cubes_list.append(new_cube)
	return cubes_list

def get_cubes_scene_adapted(track_layout_markers):
	cube_size = 7
	proportion_to_show = 0.01
	cubes_list = []
	for current_marker_index in range(len(track_layout_markers)):
		if not(isinstance(track_layout_markers[current_marker_index][2], str) and track_layout_markers[current_marker_index][2].startswith("?")):
			if random.random() < proportion_to_show:
				this_position = [track_layout_markers[current_marker_index][1], float(track_layout_markers[current_marker_index][2]), track_layout_markers[current_marker_index][3]]
				new_cube = create_cube([cube_size, cube_size, cube_size], [0, 1, 0], this_position)
				cubes_list.append(new_cube)
	return cubes_list

def get_border_scene_cube_markers():
	cube_size = 50
	second_marker_height = 1000
	return [
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_SIZE, 0, HALF_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_SIZE, 0, HALF_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_SIZE, 0, -HALF_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_SIZE, 0, -HALF_SIZE]),

		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_SIZE, second_marker_height, HALF_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_SIZE, second_marker_height, HALF_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_SIZE, second_marker_height, -HALF_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_SIZE, second_marker_height, -HALF_SIZE]),
	]

def display_result_in_3d_window(model, track_layout_markers):
	scene_cubes = get_scene_cube_markers(track_layout_markers)

	try:
		app = o3d.visualization.gui.Application.instance
		app.initialize()

		o3d.visualization.draw([model] + scene_cubes)

	except:
		print("Error 404")

def summarize_result_in_terminal(model, track_layout_markers):
	# display min height, max height, track length etc
	pass
