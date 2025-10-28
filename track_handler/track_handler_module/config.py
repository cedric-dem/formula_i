################

building_presence = 0
gp_index = 8  # 13

################

## global
HALF_SIZE = 2500

## script 1
CUT_BFS = False  # will add random points if true, else will do exact breath first search
SUBSET_TRAIL_SIZE = None  # options : size or None
STOP_EXPLORATION_LAYOUT_PATH_EARLY = 1000  # stop loop after that many steps (or none to go until the end)

## script 2
ADAPT_QUANTITY = 1000
SUBSET_TRIANGLES_SIZE = 10000  # options : int to retreive only that amount of triangles or None to go with full list of triangles

## script 3
DISPLAY_RESULT = "IN_3D_WINDOW"  # options : "SUMMARIZE_IN_TERMINAL", "IN_3D_WINDOW"

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

# image_layout_path = "f1_tracks/img_layout/" + gp_name + ".jpg"
image_layout_path = "f1_tracks/img_layout_resized/" + gp_name + ".jpg"

FILENAME_TEMP_LAYOUT = "temp_layout.csv"

################
