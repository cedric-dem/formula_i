################

building_presence = 0
gp_index = 13  # 13

################

## global
HALF_SIZE = 2500

PRINT_STATUS_EVERY = 1000

#smoothen
SLIDING_WINDOW_RADIUS = 350
SLIDING_WINDOW_TECHNIQUE = "mean"

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

SMOOTHEN_LAYOUT_CSV_FILE = "f1_tracks/layout_smoothen_csv_files/" + gp_name + ".csv"
TEMP_GLB_FILE = "temp.glb"  # "f1_tracks/output_glb_layout/" + gp_name + ".glb"
OUTPUT_OBJ_FILE = "f1_tracks/output_obj_layout/" + gp_name + ".obj"

################
