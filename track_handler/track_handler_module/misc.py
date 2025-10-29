from pathlib import Path
from PIL import Image
from statistics import median, mean

import numpy as np
import open3d as o3d
import random
import csv
import sys
import time

from .config import *

random.seed(123456)

def create_csv_layout_file():
	t0 = time.time()
	trackers_information = create_markers_list()
	t1 = time.time()
	print('==> time taken', round(t1 - t0, 2))

	in_csv_file(trackers_information, FILENAME_TEMP_LAYOUT)

def in_csv_file(data, filename):
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
	MODEL_PATH = Path(path_str)

	if not MODEL_PATH.exists():
		print("Error: file not found: " + str(MODEL_PATH))
		sys.exit(1)

	model = o3d.io.read_triangle_model(MODEL_PATH)

	if model is None:
		print("Error: failed to load model (unsupported/corrupted GLB).")
		sys.exit(1)

	print('==> finished reading model, resizing it')
	return get_resized_model(model)

def create_markers_list():
	print("==> layout image path : ", image_layout_path)

	img = Image.open(image_layout_path).convert("RGB")
	width, height = img.size

	if width != height:
		raise Exception("not square image")

	pixels = img.load()

	start, end = detect_start_and_end_point(width, pixels, HALF_SIZE)
	print("==> start,end", start, end)

	trail_composition = get_trail_shape(width, pixels, HALF_SIZE)
	random.shuffle(trail_composition)
	print('==> trail_composition size', len(trail_composition))

	print("==> beginning merge")
	resulting_layout = merge_start_trail_end(start, trail_composition, end)

	print("==> start trail: ", resulting_layout[:10])
	print("==> mid point: ", resulting_layout[len(resulting_layout) // 2])
	print("==> end trail: ", resulting_layout[-10:])
	print("==> Final result layout size : ", len(resulting_layout))  # , ": ", resulting_layout)

	return resulting_layout

def adapt_markers_to_model():
	model = get_model()

	triangles_set = get_list_of_triangles(model)
	print("==> triangles qty", len(triangles_set))

	current_layout_markers_adapted = read_layout_file(FILENAME_TEMP_LAYOUT)
	print('==> finish read')

	continue_exploration = True
	current_loop = 0
	total = len(current_layout_markers_adapted)
	remaining = count_remaining(current_layout_markers_adapted)

	while continue_exploration:
		print("====> current loop this execution : ", current_loop, " total quantity :", total, " remaining", remaining, " continue ", continue_exploration)

		# small step progress in adapting layout
		augment_layout_markers_using_triangles_list(current_layout_markers_adapted, triangles_set, ADAPT_QUANTITY_PER_BATCH)

		# write in csv file
		in_csv_file(current_layout_markers_adapted, FILENAME_TEMP_LAYOUT)

		# check if needed to continue
		current_loop += 1

		remaining = count_remaining(current_layout_markers_adapted)

		continue_exploration = remaining > 0

	print('==> Finished exploration')

def count_remaining(current_trackers):
	count = 0
	for row in current_trackers:
		if isinstance(row[2], str) and row[2].startswith("?"):
			count += 1
	return count

def display_map_and_markers():
	model = get_model()

	track_layout_markers = read_layout_file(FILENAME_TEMP_LAYOUT)

	summarize_result_in_terminal(model, track_layout_markers)
	display_result_in_3d_window(model, track_layout_markers)

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
			if r > 128 and g < 128 and b < 128:
				red_pixel = (x - delta_coord, y - delta_coord)
			elif r < 128 and g > 128 and b < 128:
				green_pixel = (x - delta_coord, y - delta_coord)
			x += 1
		y += 1
	return [green_pixel, red_pixel]

def get_trail_shape(im_size, pixels, delta_coord):
	return [
		[x - delta_coord, y - delta_coord]
		for y in range(im_size)
		for x in range(im_size)
		if ((pixels[x, y][0] < 125 and pixels[x, y][1] < 128 and pixels[x, y][2] < 128) or (pixels[x, y][0] > 128 and pixels[x, y][1] < 128 and pixels[x, y][2] < 128) or (pixels[x, y][0] < 128 and pixels[x, y][1] > 128 and pixels[x, y][2] < 128))
	]

def merge_start_trail_end(start, trail_composition, end):
	unexplored_pixels = [[start[0], start[1]]]
	explored_pixels = []

	current_index = 0
	continue_exploration = True

	t0 = time.time()
	while continue_exploration:
		elem_to_treat = unexplored_pixels.pop(0)
		explored_pixels.append(elem_to_treat)

		for neighbour in [[0, 1], [0, -1], [1, 0], [-1, 0]]:
			next_coord = [elem_to_treat[0] + neighbour[0], elem_to_treat[1] + neighbour[1]]
			if (next_coord not in explored_pixels) and (next_coord not in unexplored_pixels) and (next_coord in trail_composition):
				unexplored_pixels.append(next_coord)

		current_index += 1

		if STOP_EXPLORATION_LAYOUT_PATH_EARLY == None:
			continue_exploration = len(unexplored_pixels) > 0
		else:
			continue_exploration = current_index < STOP_EXPLORATION_LAYOUT_PATH_EARLY

		if current_index % SHOW_TRAIL_BFS_STATE_EVERY == 0:
			t1 = time.time()
			print('====> Current index :', current_index, ". time taken for those ", SHOW_TRAIL_BFS_STATE_EVERY, ": ", round(t1 - t0, 2), " so average per elem ", round((t1 - t0) / SHOW_TRAIL_BFS_STATE_EVERY, 2))
			t0 = t1

	return get_formatted_layout_file_content(start, explored_pixels, end)

def get_formatted_layout_file_content(start, explored_pixels, end):
	formatted_list = [["START", start[0], "?", start[1]]]
	for i in range(len(explored_pixels)):
		formatted_list.append([i, explored_pixels[i][0], "?", explored_pixels[i][1]])
	formatted_list.append(["END", end[0], "?", end[1]])
	return formatted_list

def augment_layout_markers_using_triangles_list(current_adapted_track_layout, triangles_set, quantity):
	current_index = 0
	current_found = 0
	continue_exploration = True

	t0 = time.time()
	while continue_exploration:

		if isinstance(current_adapted_track_layout[current_index][2], str) and current_adapted_track_layout[current_index][2].startswith("?"):
			# adapt that one
			current_adapted_track_layout[current_index][2] = get_adapted_height_of_marker(current_adapted_track_layout[current_index], triangles_set)
			current_found += 1

		current_index += 1

		continue_exploration = (current_found < quantity) and not (current_index >= len(current_adapted_track_layout))

	if current_found % quantity == 0:  # if finished by reaching end
		t1 = time.time()
		print('====> Current index :', current_index, ". time taken for those last ", quantity, ": ", round(t1 - t0, 2), " so average per elem ", round((t1 - t0) / quantity, 2))

def get_adapted_height_of_marker(marker_to_adapt, triangles_set):
	intersecting_triangle_index = get_index_of_intersecting_triangle(marker_to_adapt, triangles_set)

	found_triangle = triangles_set[intersecting_triangle_index]

	new_height = get_height_of_point_using_triangle([marker_to_adapt[1], marker_to_adapt[3]], found_triangle)

	return round(new_height, 3)

def get_height_of_point_using_triangle(marker_to_adapt, found_triangle):
	# solution 0, quickest , just take the height of a random point of the triangle
	# solution 1, just take the height of the center of the triangle
	# solution 2, list 3 points of the triangle + its center, and take the closest point, returning its height
	# solution 3, to be exact, position of point exactly on the triangle

	avg_point = [(found_triangle[0][0] + found_triangle[1][0] + found_triangle[2][0]) / 3, (found_triangle[0][1] + found_triangle[1][1] + found_triangle[2][1]) / 3, (found_triangle[0][2] + found_triangle[1][2] + found_triangle[2][2]) / 3]

	list_of_points_of_interest = found_triangle + [avg_point]

	index_closest_point = get_index_of_closest_point(list_of_points_of_interest, marker_to_adapt)

	# height = found_triangle[0][1]  # solution 0
	# height = (found_triangle[0][1] + found_triangle[1][1] + found_triangle[2][1]) / 3 #solution 1
	height = list_of_points_of_interest[index_closest_point][1]  # solution 2
	return height

def get_index_of_closest_point(list_of_points_of_interest, goal_pos):
	current_best_index = None
	current_best_distance = None
	for point_index in range(len(list_of_points_of_interest)):
		this_point = list_of_points_of_interest[point_index]  # [x,y,z]
		this_distance = get_distance_2d([this_point[0], this_point[2]], goal_pos)

		if current_best_distance is None or this_distance < current_best_distance:
			current_best_distance = this_distance
			current_best_index = point_index

	return current_best_index

def get_distance_2d(point_a, point_b):
	return (((point_a[0] - point_b[0]) ** 2) + ((point_a[1] - point_a[1]) ** 2)) ** 0.5

def get_index_of_intersecting_triangle(marker_to_adapt, triangles_set):
	found = False
	current_triangle_index = 0
	while current_triangle_index < len(triangles_set) and not found:
		if is_inside_triangle([marker_to_adapt[1], marker_to_adapt[3]], triangles_set[current_triangle_index]):
			found = True
		current_triangle_index += 1
	if not found:
		# could do plan b, only if not found : look for closest : old code :
		"""
		def get_distance(mesh_3d_point, tracker_coord_2d):
			return (((tracker_coord_2d[0] - mesh_3d_point[0]) ** 2) + ((tracker_coord_2d[1] - mesh_3d_point[2]) ** 2)) ** 0.5
		for current_point in triangles_set[current_triangle_index]:
			this_distance = get_distance(current_point, [marker_to_adapt[1], marker_to_adapt[3]])
			if best_distance == None or this_distance < best_distance:
				best_distance = this_distance
				best_point = current_point
		"""
		raise ValueError("No triangle found")
	return current_triangle_index - 1

def is_inside_triangle(two_dim_pos, this_triangle):  # TODO clean this
	return ((this_triangle[0][0] <= two_dim_pos[0] <= this_triangle[1][0]) or (this_triangle[0][0] <= two_dim_pos[0] <= this_triangle[2][0]) or (this_triangle[1][0] <= two_dim_pos[0] <= this_triangle[2][0]) or (
			this_triangle[0][0] >= two_dim_pos[0] >= this_triangle[1][0]) or (this_triangle[0][0] >= two_dim_pos[0] >= this_triangle[2][0]) or (this_triangle[1][0] >= two_dim_pos[0] >= this_triangle[2][0])) and (
			(this_triangle[0][2] <= two_dim_pos[1] <= this_triangle[1][2]) or (this_triangle[0][2] <= two_dim_pos[1] <= this_triangle[2][2]) or (this_triangle[1][2] <= two_dim_pos[1] <= this_triangle[2][2]) or (
			this_triangle[0][2] >= two_dim_pos[1] >= this_triangle[1][2]) or (this_triangle[0][2] >= two_dim_pos[1] >= this_triangle[2][2]) or (this_triangle[1][2] >= two_dim_pos[1] >= this_triangle[2][2]))

def get_list_of_triangles(model):  # to be merged later with upper function
	list_of_triangles = get_initial_list_of_triangles(model, 1)
	random.shuffle(list_of_triangles)
	return list_of_triangles

def get_initial_list_of_triangles(model, i):
	mesh = model.meshes[i].mesh

	vertices = np.asarray(mesh.vertices)
	faces = np.asarray(mesh.triangles)

	triangles = [vertices[tri].tolist() for tri in faces]
	return triangles

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

	target_span = 2 * HALF_SIZE

	scale_x = target_span / current_range_x
	scale_z = target_span / current_range_z

	for geom in model.meshes:
		mesh = geom.mesh
		mesh_vertices = np.asarray(mesh.vertices)

		transformed_vertices = mesh_vertices.copy()
		transformed_vertices[:, 0] = (mesh_vertices[:, 0] - min_x) * scale_x - HALF_SIZE
		transformed_vertices[:, 2] = (mesh_vertices[:, 2] - min_z) * scale_z - HALF_SIZE

		mesh.vertices = o3d.utility.Vector3dVector(transformed_vertices)

	# display new vertices
	get_original_map_borders(model)

	return model

def get_cubes_scene_not_adapted(track_layout_markers):  # cubes not adapted : red
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

def get_cubes_scene_adapted(track_layout_markers):  # cubes adapted : green
	cube_size = 1
	proportion_to_show = 0.08
	cubes_list = []
	for current_marker_index in range(len(track_layout_markers)):
		if 9000 < current_marker_index < 11000:
			if not (isinstance(track_layout_markers[current_marker_index][2], str) and track_layout_markers[current_marker_index][2].startswith("?")):
				# color = [0, 1, 0]
				proportion = current_marker_index / len(track_layout_markers)
				color = [0, proportion, 1 - proportion]

				this_position = [track_layout_markers[current_marker_index][1], float(track_layout_markers[current_marker_index][2]), track_layout_markers[current_marker_index][3]]
				new_cube = create_cube([cube_size, cube_size, cube_size], color, this_position)
				cubes_list.append(new_cube)
	return cubes_list

def get_border_scene_cube_markers():  # markers border scene  : blue
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
	scene_cubes = get_cubes_scene_not_adapted(track_layout_markers) + get_cubes_scene_adapted(track_layout_markers) + get_border_scene_cube_markers()

	try:
		app = o3d.visualization.gui.Application.instance
		app.initialize()

		o3d.visualization.draw([model] + scene_cubes)

	except:
		print("Error 404")

def summarize_result_in_terminal(model, track_layout_markers):
	# display min height, max height, track length etc

	max_height = float(track_layout_markers[0][2])
	min_height = max_height

	for track_layout_marker in track_layout_markers:
		if not track_layout_marker[2].startswith("?"):
			current_height = float(track_layout_marker[2])
			if current_height > max_height:
				max_height = current_height

			if current_height < min_height:
				min_height = current_height

	print("==> lowest point : ", min_height, " highest point : ", max_height)
	print("==> Start and end : ", track_layout_markers[0], " - ", track_layout_markers[-1])

def sliding_window_mean(data):
	window_radius = 350
	result = []
	quantity = len(data)
	for current_index in range(quantity):
		start = max(0, current_index - window_radius)
		end = min(quantity, current_index + window_radius + 1)
		window = data[start:end]
		# result.append(round(median(window),3))
		result.append(round(mean(window), 3))
	return result

def smoothen_result():
	layout_content = read_layout_file(FILENAME_TEMP_LAYOUT)
	heights = [float(elem[2]) for elem in layout_content]
	new_heights = sliding_window_mean(heights)

	for current_index in range(len(new_heights)):
		layout_content[current_index][2] = new_heights[current_index]

	in_csv_file(layout_content, FILENAME_TEMP_LAYOUT)
