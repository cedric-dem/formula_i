from pathlib import Path
from PIL import Image
from statistics import median, mean

import trimesh
import numpy as np
import open3d as o3d
import random
import csv
import os
import sys
import time

from .config import *

random.seed(123456)

def save_in_csv_file(data, filename):
	with open(filename, mode = 'w', newline = '', encoding = 'utf-8') as file:
		writer = csv.writer(file)
		writer.writerows(data)

def read_layout_file(filename):
	resulting_layout = []
	with open(filename, mode = 'r', newline = '', encoding = 'utf-8') as file:
		reader = csv.reader(file)
		for row in reader:
			converted_row = []
			for col_index, value in enumerate(row):
				if col_index in (0, 2):
					converted_row.append(value)
				elif col_index in (1, 3):
					converted_row.append(int(value))
			resulting_layout.append(converted_row)
	return resulting_layout

def get_model():
	print('==> Reading model')
	model_path = Path(model_path_string)

	if not model_path.exists():
		print("Error: file not found: " + str(model_path))
		sys.exit(1)

	model = o3d.io.read_triangle_model(model_path)

	if model is None:
		print("Error: failed to load model (unsupported/corrupted GLB).")
		sys.exit(1)

	return get_resized_model(model)

def get_track_layout():
	t0 = time.time()
	print("==> layout image path : ", image_layout_path)

	img = Image.open(image_layout_path).convert("RGB")
	width, height = img.size

	if width != height:
		raise Exception("not square image")

	pixels = img.load()

	start, end = detect_start_and_end_point(width, pixels)
	print("==> detected start and end", start, end)

	matrix_presence = get_matrix_presence(width, pixels)
	print('==> finished retrieved matrix presence')

	print("==> beginning merge in order")
	ordered_layout = get_ordered_layout(start, matrix_presence, end)

	print("==> start trail: ", ordered_layout[:10])
	print("==> mid point: ", ordered_layout[len(ordered_layout) // 2])
	print("==> end trail: ", ordered_layout[-10:])
	print("==> Final result layout size : ", len(ordered_layout))
	t1 = time.time()
	print('==> finished get layout information. time taken', round(t1 - t0, 2))

	return ordered_layout

def adapt_layout_to_model(layout):
	t0 = time.time()
	model = get_model()

	triangles_set = get_list_of_triangles(model)
	print("==> triangles quantity", len(triangles_set))

	adapt_layout_using_triangles_list(layout, triangles_set)

	t1 = time.time()
	print('==> finished adapt layout to model time taken', round(t1 - t0, 2))
	return layout

def display_map_and_layout():
	model = get_model()

	# track_layout = read_layout_file(RAW_LAYOUT_CSV_FILE)
	track_layout = read_layout_file(SMOOTHEN_LAYOUT_CSV_FILE)

	summarize_result_in_terminal(model, track_layout)
	display_result_in_3d_window(model, track_layout)

def create_cube(size, color, position):
	new_cube = o3d.geometry.TriangleMesh.create_box(width = size[0], height = size[1], depth = size[2])
	new_cube.compute_vertex_normals()
	new_cube.paint_uniform_color(color)
	position_adapted = [position[0] - (size[0] // 2), position[1] - (size[1] // 2), position[2] - (size[2] // 2)]
	new_cube.translate(position_adapted)
	# new_cube.translate(position)
	return new_cube

def detect_start_and_end_point(im_size, pixels):
	red_pixel = None
	green_pixel = None

	y = 0
	while y < im_size - 1 and (red_pixel is None or green_pixel is None):
		x = 0
		while x < im_size - 1 and (red_pixel is None or green_pixel is None):
			r, g, b = pixels[x, y]
			if r > 128 and g < 128 and b < 128:
				red_pixel = (x, y)
			elif r < 128 and g > 128 and b < 128:
				green_pixel = (x, y)
			x += 1
		y += 1
	return [green_pixel, red_pixel]

def get_matrix_presence(image_size, pixels):
	return [
		[
			((pixels[x, y][0] < 125 and pixels[x, y][1] < 128 and pixels[x, y][2] < 128) or (pixels[x, y][0] > 128 > pixels[x, y][1] and pixels[x, y][2] < 128) or (pixels[x, y][0] < 128 < pixels[x, y][1] and pixels[x, y][2] < 128))
			for y in range(image_size)
		]
		for x in range(image_size)
	]

def get_layout_size(mat):
	layout_size = 0
	for line in mat:
		layout_size += sum(line)
	return layout_size

def get_interval_between_print(max_value):
	return int(max_value / PRINT_QUANTITY_PER_LOOP)

def get_ordered_layout(start, trail_composition, end):
	pixels_list_next = [[start[0], start[1]]]
	matrix_exploration_state = [[False for _ in range(HALF_MAP_SIZE * 2)] for _ in range(HALF_MAP_SIZE * 2)]
	explored_pixels_list_history = []

	current_index = 0
	layout_size = get_layout_size(trail_composition)

	print('===> layout size ', layout_size)

	print_interval = get_interval_between_print(layout_size)

	while len(pixels_list_next) > 0:
		elem_to_treat = pixels_list_next.pop(0)
		matrix_exploration_state[elem_to_treat[0]][elem_to_treat[1]] = True
		explored_pixels_list_history.append(elem_to_treat)

		for neighbour in [[0, 1], [0, -1], [1, 0], [-1, 0]]:
			next_coord = [elem_to_treat[0] + neighbour[0], elem_to_treat[1] + neighbour[1]]

			if (next_coord not in pixels_list_next) and (not matrix_exploration_state[next_coord[0]][next_coord[1]]) and (trail_composition[next_coord[0]][next_coord[1]]):
				pixels_list_next.append(next_coord)

		if current_index % print_interval == 0:
			print('====> Current proportion ', current_index, '/', layout_size)

		current_index += 1

	return get_formatted_layout_file_content(start, explored_pixels_list_history, end)

def get_formatted_layout_file_content(start, explored_pixels_history, end):
	formatted_list = [get_unadapted_layout_element("START", start[0], start[1])]
	for current_index in range(len(explored_pixels_history)):
		formatted_list.append(
			get_unadapted_layout_element(
				current_index,
				explored_pixels_history[current_index][0],
				explored_pixels_history[current_index][1])
		)
	formatted_list.append(get_unadapted_layout_element("END", end[0], end[1]))
	return formatted_list

def get_unadapted_layout_element(index, x, y):
	return [index, x - HALF_MAP_SIZE, None, y - HALF_MAP_SIZE]

def adapt_layout_using_triangles_list(current_adapted_track_layout, triangles_list):
	current_index = 0

	previous_triangle_index = 0

	size = len(current_adapted_track_layout)
	here_print_every = get_interval_between_print(size)

	while current_index < len(current_adapted_track_layout):

		previous_triangle_index, current_adapted_track_layout[current_index][2] = get_adapted_height_of_layout_element(current_adapted_track_layout[current_index], triangles_list, previous_triangle_index)

		if current_index % here_print_every == 0:
			print('====> Current proportion :', current_index, " / ", size)

		current_index += 1

def get_adapted_height_of_layout_element(layout_element, triangles_list, previous_index):
	intersecting_triangle_index = get_index_of_intersecting_triangle(layout_element, triangles_list, previous_index)

	found_triangle = triangles_list[intersecting_triangle_index]

	new_height = get_height_of_point_using_triangle([layout_element[1], layout_element[3]], found_triangle)

	return intersecting_triangle_index, round(new_height, 3)

def get_height_of_point_using_triangle(layout_element_to_adapt, found_triangle):
	# TODO, to be exact, should look for the position of point exactly on the triangle

	avg_point = [(found_triangle[0][0] + found_triangle[1][0] + found_triangle[2][0]) / 3, (found_triangle[0][1] + found_triangle[1][1] + found_triangle[2][1]) / 3, (found_triangle[0][2] + found_triangle[1][2] + found_triangle[2][2]) / 3]

	list_of_points_of_interest = found_triangle + [avg_point]

	index_closest_point = get_index_of_closest_point(list_of_points_of_interest, layout_element_to_adapt)

	return list_of_points_of_interest[index_closest_point][1]

def get_index_of_closest_point(list_of_points_of_interest, goal_position):
	current_best_index = None
	current_best_distance = None
	for point_index in range(len(list_of_points_of_interest)):
		this_point = list_of_points_of_interest[point_index]  # [x,y,z]
		this_distance = get_distance_2d([this_point[0], this_point[2]], goal_position)

		if current_best_distance is None or this_distance < current_best_distance:
			current_best_distance = this_distance
			current_best_index = point_index

	return current_best_index

def get_distance_2d(point_a, point_b):
	return (((point_a[0] - point_b[0]) ** 2) + ((point_a[1] - point_a[1]) ** 2)) ** 0.5

def get_index_of_intersecting_triangle(layout_element_to_adapt, triangles_list, first_index_to_try):
	found = False
	current_triangle_index = 0
	if is_inside_triangle([layout_element_to_adapt[1], layout_element_to_adapt[3]], triangles_list[first_index_to_try]):
		found = True
		current_triangle_index = first_index_to_try
	else:
		while current_triangle_index < len(triangles_list) and not found:
			if is_inside_triangle([layout_element_to_adapt[1], layout_element_to_adapt[3]], triangles_list[current_triangle_index]):
				found = True
			current_triangle_index += 1
		current_triangle_index -= 1

	if not found:
		# for now, never happened, if that happens, compute the distance to every triangle and just take the closest
		raise ValueError("No triangle found")
	return current_triangle_index

def cross(x1, y1, x2, y2, x3, y3):
	return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)

def nz(v, eps):
	return 0.0 if abs(v) < eps else v

def is_inside_triangle(two_dim_pos, this_triangle):
	Ax, Ay = this_triangle[0][0], this_triangle[0][2]
	Bx, By = this_triangle[1][0], this_triangle[1][2]
	Cx, Cy = this_triangle[2][0], this_triangle[2][2]
	Px, Py = two_dim_pos[0], two_dim_pos[1]

	eps = 1e-12
	if abs(cross(Ax, Ay, Bx, By, Cx, Cy)) < eps:
		result = False
	else:
		c1 = nz(cross(Ax, Ay, Bx, By, Px, Py), eps)
		c2 = nz(cross(Bx, By, Cx, Cy, Px, Py), eps)
		c3 = nz(cross(Cx, Cy, Ax, Ay, Px, Py), eps)

		result = not (((c1 > 0) or (c2 > 0) or (c3 > 0)) and ((c1 < 0) or (c2 < 0) or (c3 < 0)))
	return result

def get_list_of_triangles(model):
	list_of_triangles = get_initial_list_of_triangles(model, 1)
	random.shuffle(list_of_triangles)
	return list_of_triangles

def get_initial_list_of_triangles(model, i):
	mesh = model.meshes[i].mesh

	vertices = np.asarray(mesh.vertices)
	faces = np.asarray(mesh.triangles)

	return [vertices[tri].tolist() for tri in faces]

def get_original_map_borders(model):
	# triangles_list = get_initial_list_of_triangles(model, 0) #-3464.262451171875 3464.262451171875 -3464.27783203125 3464.27783203125
	triangles_list = get_initial_list_of_triangles(model, 1)  # -3464.262451171875 3464.262451171875 -3464.27783203125 3464.27783203125

	max_x = triangles_list[0][0][0]
	min_x = max_x
	min_z = triangles_list[0][0][2]
	max_z = min_z

	for current_triangle in triangles_list:
		for current_point in current_triangle:
			this_x = current_point[0]
			this_z = current_point[2]

			min_x = min(min_x, this_x)
			max_x = max(max_x, this_x)
			min_z = min(min_z, this_z)
			max_z = max(max_z, this_z)

	print("==> vertices border ", min_x, max_x, min_z, max_z)
	return [min_x, max_x, min_z, max_z]

def get_resized_model(model):
	min_x, max_x, min_z, max_z = get_original_map_borders(model)

	current_range_x = max_x - min_x
	current_range_z = max_z - min_z

	if current_range_x == 0 or current_range_z == 0:
		raise ValueError("Invalid map dimensions: zero width or height detected.")

	target_span = 2 * HALF_MAP_SIZE

	scale_x = target_span / current_range_x
	scale_z = target_span / current_range_z

	print("===> Resize scale : ", scale_x, scale_z)

	for current_geometry in model.meshes:
		mesh = current_geometry.mesh
		mesh_vertices = np.asarray(mesh.vertices)

		transformed_vertices = mesh_vertices.copy()
		transformed_vertices[:, 0] = (mesh_vertices[:, 0] - min_x) * scale_x - HALF_MAP_SIZE
		transformed_vertices[:, 1] = mesh_vertices[:, 1] * scale_z
		transformed_vertices[:, 2] = (mesh_vertices[:, 2] - min_z) * scale_z - HALF_MAP_SIZE

		mesh.vertices = o3d.utility.Vector3dVector(transformed_vertices)

	# display new vertices
	get_original_map_borders(model)

	return model

def get_layout_cubes_scene(track_layout):  # cubes adapted : green
	cube_size = 1
	cubes_list = []
	for current_track_layout_element_index in range(0, len(track_layout), 20):
		# color = [0, 1, 0]
		proportion = current_track_layout_element_index / len(track_layout)
		color = [0, proportion, 1 - proportion]

		this_position = [track_layout[current_track_layout_element_index][1], float(track_layout[current_track_layout_element_index][2]), track_layout[current_track_layout_element_index][3]]
		new_cube = create_cube([cube_size, cube_size, cube_size], color, this_position)
		cubes_list.append(new_cube)
	return cubes_list

def get_border_scene_cube_markers():  # markers border scene  : blue
	cube_size = 50
	second_marker_height = 1000
	return [
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_MAP_SIZE, 0, HALF_MAP_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_MAP_SIZE, 0, HALF_MAP_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_MAP_SIZE, 0, -HALF_MAP_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_MAP_SIZE, 0, -HALF_MAP_SIZE]),

		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_MAP_SIZE, second_marker_height, HALF_MAP_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_MAP_SIZE, second_marker_height, HALF_MAP_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [HALF_MAP_SIZE, second_marker_height, -HALF_MAP_SIZE]),
		create_cube([cube_size, cube_size, cube_size], [0, 0, 1], [-HALF_MAP_SIZE, second_marker_height, -HALF_MAP_SIZE]),
	]

def display_result_in_3d_window(model, track_layout):
	scene_cubes = get_layout_cubes_scene(track_layout) + get_border_scene_cube_markers()

	try:
		app = o3d.visualization.gui.Application.instance
		app.initialize()

		o3d.visualization.draw([model] + scene_cubes)

	except:
		print("Error 404")

def summarize_result_in_terminal(model, track_layout):
	# display min height, max height, track length etc

	max_x = float(track_layout[0][1])
	min_x = max_x

	max_y = float(track_layout[0][2])
	min_y = max_y

	max_z = float(track_layout[0][3])
	min_z = max_z

	for track_layout_element in track_layout:
		current_x = float(track_layout_element[1])
		current_y = float(track_layout_element[2])
		current_z = float(track_layout_element[3])

		max_x = max(max_x, current_x)
		min_x = min(min_x, current_x)

		max_y = max(max_y, current_y)
		min_y = min(min_y, current_y)

		max_z = max(max_z, current_z)
		min_z = min(min_z, current_z)

	print("==> x from ", min_x, " to ", max_x)
	print("==> lowest point : ", min_y, " highest point : ", max_y)
	print("==> z from ", min_z, " to ", max_z)
	print("==> Start and end : ", track_layout[0], " - ", track_layout[-1])

def sliding_window_mean(data):
	result = []
	quantity = len(data)
	here_print_every = get_interval_between_print(quantity)
	for current_index in range(quantity):
		if current_index % here_print_every == 0:
			print("====> current proportion: ", current_index, "/", quantity)

		window = data[
				 max(0, current_index - SLIDING_WINDOW_RADIUS):
				 min(quantity, current_index + SLIDING_WINDOW_RADIUS + 1)
				 ]

		if SLIDING_WINDOW_TECHNIQUE == "mean":
			result.append(round(mean(window), 3))
		elif SLIDING_WINDOW_TECHNIQUE == "median":
			result.append(round(median(window), 3))
	return result

def smoothen_result(layout):
	t0 = time.time()
	heights = [float(layout_element[2]) for layout_element in layout]
	new_heights = sliding_window_mean(heights)

	for current_index in range(len(new_heights)):
		layout[current_index][2] = new_heights[current_index]

	t1 = time.time()
	print('==> finished smoothen result. time taken', round(t1 - t0, 2))
	return layout

def get_map_as_matrix(layout):
	top_map_as_matrix = [[None for _ in range(HALF_MAP_SIZE * 2)] for _ in range(HALF_MAP_SIZE * 2)]

	for layout_element in layout:
		top_map_as_matrix[int(layout_element[1]) + HALF_MAP_SIZE][int(layout_element[3]) + HALF_MAP_SIZE] = (layout_element[0], float(layout_element[2]))

	return top_map_as_matrix

def convert_to_final_obj_model(layout):
	t0 = time.time()

	top_map_as_matrix = get_map_as_matrix(layout)

	triangles_list = get_triangles_list_from_matrix(top_map_as_matrix)

	print('==> Creating glb file')
	triangles_to_glb(triangles_list)

	print('==> Creating converting glb to obj file')
	convert_glb_to_obj()

	print('==> removing glb file')
	os.remove(TEMP_GLB_FILE)

	t1 = time.time()
	print('==> finished convert to obj model. time taken', round(t1 - t0, 2))

def get_triangles_list_from_matrix(top_map_as_matrix):
	triangles_list = []

	for current_i in range(1, (2 * HALF_MAP_SIZE) - 1):
		for current_j in range(1, (2 * HALF_MAP_SIZE) - 1):
			if top_map_as_matrix[current_i][current_j]:  # if current position is occupied

				triangles_list += detect_neighbours_and_retrieve_triangles(top_map_as_matrix, current_i, current_j)

	return triangles_list

def detect_neighbours_and_retrieve_triangles(top_map_as_matrix, current_i, current_j):
	number_neighbours = get_number_of_neighbours(current_i, current_j, top_map_as_matrix)

	higher_x_cube = get_coord_from_position(top_map_as_matrix, current_i + 1, current_j)
	lower_x_cube = get_coord_from_position(top_map_as_matrix, current_i - 1, current_j)
	higher_z_cube = get_coord_from_position(top_map_as_matrix, current_i, current_j + 1)
	lower_z_cube = get_coord_from_position(top_map_as_matrix, current_i, current_j - 1)

	this_coord = get_coord_from_position(top_map_as_matrix, current_i, current_j)

	new_triangles_list = []

	if number_neighbours == 4:  # 4 neighbours, most common case
		new_triangles_list = get_triangles_list_from_4_neighbour(higher_x_cube, lower_x_cube, higher_z_cube, lower_z_cube, this_coord)

	elif number_neighbours == 3:  # could happen often
		new_triangles_list = get_triangles_list_from_3_neighbour(higher_x_cube, lower_x_cube, higher_z_cube, lower_z_cube, this_coord)

	elif number_neighbours == 2:  # will happen often
		new_triangles_list = get_triangles_list_from_2_neighbour(higher_x_cube, lower_x_cube, higher_z_cube, lower_z_cube, this_coord)

	elif number_neighbours == 1:  # could happen rarely, but will add nothing
		pass

	elif number_neighbours == 0:  # should not happen
		print("=> Error triangle 0")

	else:
		print("==> Error  triangle 1", number_neighbours)
	return new_triangles_list

def get_triangles_list_from_4_neighbour(higher_x_cube, lower_x_cube, higher_z_cube, lower_z_cube, this_cube):
	return [
		[higher_x_cube, lower_z_cube, this_cube],
		[lower_x_cube, higher_z_cube, this_cube],
		[higher_z_cube, higher_x_cube, this_cube],
		[lower_z_cube, lower_x_cube, this_cube]
	]

def get_triangles_list_from_3_neighbour(higher_x_cube, lower_x_cube, higher_z_cube, lower_z_cube, this_cube):
	new_neighbours = []
	if higher_x_cube is None:
		new_neighbours = [
			[lower_x_cube, higher_z_cube, this_cube],
			[lower_z_cube, lower_x_cube, this_cube]
		]
	elif lower_x_cube is None:
		new_neighbours = [
			[higher_x_cube, lower_z_cube, this_cube],
			[higher_z_cube, higher_x_cube, this_cube],
		]
	elif higher_z_cube is None:
		new_neighbours = [
			[higher_x_cube, lower_z_cube, this_cube],
			[lower_z_cube, lower_x_cube, this_cube]
		]
	elif lower_z_cube is None:
		new_neighbours = [
			[lower_x_cube, higher_z_cube, this_cube],
			[higher_z_cube, higher_x_cube, this_cube],
		]
	else:
		print("error triangle 3")
	return new_neighbours

def get_triangles_list_from_2_neighbour(higher_x_cube, lower_x_cube, higher_z_cube, lower_z_cube, this_cube):
	new_neighbours = []

	if higher_x_cube is None and higher_z_cube is None:  # 4 first cases could happen, will add to the triangles
		new_neighbours = [[lower_z_cube, lower_x_cube, this_cube]]
	elif higher_x_cube is None and lower_z_cube is None:
		new_neighbours = [[lower_x_cube, higher_z_cube, this_cube]]
	elif lower_x_cube is None and higher_z_cube is None:
		new_neighbours = [[higher_x_cube, lower_z_cube, this_cube]]
	elif lower_x_cube is None and lower_z_cube is None:
		new_neighbours = [[higher_z_cube, higher_x_cube, this_cube]]

	elif higher_x_cube is None and lower_x_cube is None:  # those two should never happen, current cube is in between opposite cubes, so add nothing
		print("error triangle 4")
	elif higher_z_cube is None and lower_z_cube is None:
		print("error triangle 5")
	else:
		print("error triangle 6")
	return new_neighbours

def get_coord_from_position(top_map_as_matrix, i, j):
	if top_map_as_matrix[i][j]:
		height = [i - HALF_MAP_SIZE, top_map_as_matrix[i][j][1], j - HALF_MAP_SIZE]
	else:
		height = None
	return height

def get_number_of_neighbours(current_i, current_j, new_matrix):
	neighbours = [
		new_matrix[current_i - 1][current_j] is not None,
		new_matrix[current_i + 1][current_j] is not None,
		new_matrix[current_i][current_j - 1] is not None,
		new_matrix[current_i][current_j + 1] is not None
	]
	return sum(neighbours)

def triangles_to_glb(triangles):
	gray = 0.1
	vert_map = {}
	vertices = []
	faces = []
	for triangle in triangles:
		indexes = []
		for point in triangle:
			key = (float(point[0]), float(point[1]), float(point[2]))
			if key not in vert_map:
				vert_map[key] = len(vertices)
				vertices.append(key)
			indexes.append(vert_map[key])
		faces.append(indexes)

	vertices = np.asarray(vertices, dtype = np.float64)
	faces = np.asarray(faces, dtype = np.int32)

	mesh = o3d.geometry.TriangleMesh(
		vertices = o3d.utility.Vector3dVector(vertices),
		triangles = o3d.utility.Vector3iVector(faces),
	)

	mesh.remove_duplicated_vertices()
	mesh.remove_degenerate_triangles()
	mesh.compute_vertex_normals()

	mesh.vertex_colors = o3d.utility.Vector3dVector(np.tile(np.array([[gray, gray, gray]], dtype = np.float64), (len(vertices), 1)))

	ok = o3d.io.write_triangle_mesh(
		TEMP_GLB_FILE, mesh,
		write_vertex_normals = True,
		write_vertex_colors = True,
		print_progress = False
	)
	if not ok:
		raise RuntimeError("error")

def convert_glb_to_obj():
	mesh = trimesh.load(TEMP_GLB_FILE, force = 'mesh')
	mesh.export(OUTPUT_OBJ_FILE)

def create_obj_track_file():
	print('////////////////////////////////////////////////////////// get layout information ///')
	unadapted_layout = get_track_layout()

	print('////////////////////////////////////////////////////////// Adapt layout to model ///')
	adapted_layout = adapt_layout_to_model(unadapted_layout)

	print('////////////////////////////////////////////////////////// Smoothen Result ///')
	smoothened_layout = smoothen_result(adapted_layout)

	print('////////////////////////////////////////////////////////// SAVE CSV FILE ///')
	save_in_csv_file(smoothened_layout, SMOOTHEN_LAYOUT_CSV_FILE)  # no real need for that, in fact i could delete both read and write function

	print('////////////////////////////////////////////////////////// Convert to final obj model ///')
	convert_to_final_obj_model(unadapted_layout)
