################################## Global config

CAMERA_TYPE = "FOLLOW_CAR"  # Options: "FIXED", "FOLLOW_CAR", "ROTATE_AROUND_CAR"
MOVE_DECISION = "KEYBOARD"  # Options: "DEFAULT", "AI", "KEYBOARD"
SIMPLIFIED_MODEL = False
DISPLAY_DATA = "ON_TERMINAL"  # Options : "ON_GUI", "ON_TERMINAL", "NO"
REFRESH_EVERY_FRAME = 240
ADD_DEBUG_CUBES = False
REFRESH_SENSORS_EVERY_FRAME = 3
DISPLAY_SENSORS = True

################################### Car Behaviour

MAX_SPEED_KMH = 370
MAX_SPEED_MPS = MAX_SPEED_KMH / 3.6

CAR_INITIAL_POSITION = (-430, 1084, 106)  # (-420,1066,163) (135,-199,149) 150,-290,162) other points on the track

INITIAL_ANGLE_OF_CAR = 120

GRAV = 15

HALF_MAP_SIZE = 2500

################################### Look

SKY_COLOR = (0.615, 0.850, 0.956)
GROUND_COLOR = (0.443, 0.603, 0.435)
ROAD_COLOR = (0.737, 0.768, 0.713)

CAR_COLOR = (0.847, 0.458, 0.396)
HELMET_COLOR = (0.1, 0.1, 0.1)
WHEELS_COLOR = (0.2, 0.2, 0.2)

CAR_BODY_BLOCKS = [
	# (delta_x, delta_z, size_x, size_y, size_z, color_rgb)
	(0, 0, 1.7, 1.5, 0.5, CAR_COLOR),  # Main chassis
	(0.5, 0.0, 3.8, 0.4, 0.4, CAR_COLOR),  # chassis lengthwise
	(2.3, -0.24, 0.5, 1.4, 0.1, CAR_COLOR),  # Front wing assembly
	(-1.8, 0.24, 0.5, 1.0, 0.4, CAR_COLOR),  # Rear wing assembly
	(-0.6, 0.3, 0.7, 0.2, 0.4, CAR_COLOR),  # Center cell upper
	(0, 0.3, 0.3, 0.2, 0.4, HELMET_COLOR),  # helmet
]

################################### track
gp_index = 13
gp_name = [
	"01_australian_gp", "02_chinese_gp", "03_japansese_gp", "04_bahrain_gp", "05_saudi_arabian_gp", "06_miami_gp",
	"07_imola_gp", "08_monaco_gp", "09_spanish_gp", "10_canadian_gp", "11_austrian_gp", "12_british_gp",
	"13_belgian_gp", "14_hungarian_gp", "15_dutch_gp", "16_italian_gp", "17_azerbaidjan_gp", "18_singapore_gp",
	"19_united_states_gp", "20_mexican_gp", "21_brazilian_gp", "22_las_vegas_gp", "23_qatar_gp", "24_abu_dhabi_gp"
][gp_index - 1]

FOLDER_OBJ_FILES = "f1_tracks/obj_files"
FOLDER_CSV_FILES = "f1_tracks/csv_files"

TRACK_LAYOUT_CSV_FILE = FOLDER_CSV_FILES + "/" + gp_name + ".csv"

################################### initialize

camera_rotation_angle = 0.0

################################### terrain

GROUND_COLLISION_MARGIN = 0.0
GROUND_CONTACT_STIFFNESS = 3000.0
GROUND_CONTACT_DAMPING = 300.0

CAR_CONTACT_STIFFNESS = 3000.0
CAR_CONTACT_DAMPING = 300.0
