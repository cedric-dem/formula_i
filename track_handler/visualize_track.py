import sys
from pathlib import Path
import numpy as np
import open3d as o3d

################

building_presence = 0
gp_index = 8

################

model_type = ["glb_files_no_buildings", "glb_files_buildings"][building_presence]

gp_name = [
	"01_australian_gp", "02_chinese_gp", "03_japansese_gp", "04_bahrain_gp", "05_saudi_arabian_gp", "06_miami_gp",
	"07_imola_gp", "08_monaco_gp", "09_spanish_gp", "10_canadian_gp", "11_austrian_gp", "12_british_gp",
	"13_belgian_gp", "14_hungarian_gp", "15_dutch_gp", "16_italian_gp", "17_azerbaidjan_gp", "18_singapore_gp",
	"19_united_states_gp", "20_mexican_gp", "21_brazilian_gp", "22_las_vegas_gp", "23_qatar_gp", "24_abu_dhabi_gp"
][gp_index - 1]

################

path_str = "f1_tracks/" + model_type + "/" + gp_name + ".glb"

MODEL_PATH = Path(path_str)

################

def create_cube(size, color, position):
	new_cube = o3d.geometry.TriangleMesh.create_box(width = size[0], height = size[1], depth = size[2])
	new_cube.compute_vertex_normals()
	new_cube.paint_uniform_color(color)
	new_cube.translate([-0.5, -0.5, -0.5])
	new_cube.translate(position)
	return new_cube

def combine_triangle_model_meshes(model):
	combined_mesh = o3d.geometry.TriangleMesh()

	if hasattr(model, "meshes"):
		for mesh_model in model.meshes:
			mesh = getattr(mesh_model, "mesh", None)
			if mesh is None:
				continue
			combined_mesh += mesh

	return combined_mesh

def determine_surface_height(model_mesh, cube_size, start_position):
	if model_mesh is None or model_mesh.is_empty():
		return None

	vertices = np.asarray(model_mesh.vertices)
	if vertices.size == 0:
		return None

	half_x = cube_size[0] / 2
	half_z = cube_size[2] / 2
	min_x = start_position[0] - half_x
	max_x = start_position[0] + half_x
	min_z = start_position[2] - half_z
	max_z = start_position[2] + half_z

	mask = (
			(vertices[:, 0] >= min_x)
			& (vertices[:, 0] <= max_x)
			& (vertices[:, 2] >= min_z)
			& (vertices[:, 2] <= max_z)
	)

	if np.any(mask):
		return float(vertices[mask, 1].max())

	return float(vertices[:, 1].max())

def drop_position_onto_model(start_position, cube_size, model_mesh, step_size = 1.0):
	position_y = start_position[1]

	surface_height = determine_surface_height(model_mesh, cube_size, start_position)
	if surface_height is None:
		return start_position

	target_y = surface_height + cube_size[1] / 2

	if position_y <= target_y:
		return start_position

	while position_y - step_size > target_y:
		position_y -= step_size

	return [start_position[0], target_y, start_position[2]]

def main():
	if not MODEL_PATH.exists():
		print(f"Error: file not found: {MODEL_PATH.resolve()}")
		sys.exit(1)

	model = o3d.io.read_triangle_model(MODEL_PATH)
	if model is None:
		print("Error: failed to load model (unsupported/corrupted GLB).")
		sys.exit(1)

	model_mesh = combine_triangle_model_meshes(model)

	markers = []

	markers.append(create_cube([100, 10, 100], [1, 0, 0], [0, 10, 0]))
	markers.append(create_cube([100, 10, 100], [0, 1, 0], [0, 200, 0]))

	blue_start = [-2500, 2000, -2500]
	blue_size = [50, 50, 50]
	blue_position = drop_position_onto_model(blue_start, blue_size, model_mesh)
	markers.append(create_cube(blue_size, [0, 0, 1], blue_position))

	try:
		app = o3d.visualization.gui.Application.instance
		app.initialize()

		win = o3d.visualization.O3DVisualizer(title = f"GLB Viewer – {MODEL_PATH.name}", width = 1280, height = 800)

		rgba = (0, 0, 0, 1)
		col = np.array(rgba, dtype = np.float32)

		if hasattr(win, "set_background"):
			win.set_background(col, o3d.geometry.Image())

		win.add_model("spa_v2", model)

		# Frame camera
		if hasattr(win, "reset_camera_to_default"):
			win.reset_camera_to_default()

		app.add_window(win)
		app.run()

	except Exception as e:
		print(f"[Info] O3DVisualizer had issues ({e}). Falling back to simple viewer.")
		try:
			o3d.visualization.draw([model] + markers)
		except Exception as e2:
			print(f"Fallback viewer failed as well: {e2}")
			sys.exit(1)

if __name__ == "__main__":
	main()
