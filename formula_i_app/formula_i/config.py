################################## Global config

CAMERA_TYPE = "FOLLOW_CAR"  # Options: "FIXED", "FOLLOW_CAR", "ROTATE_AROUND_CAR", "KEYBOARD_CONTROLLED"
MOVE_DECISION = "KEYBOARD"  # Options: "DEFAULT", "AI", "KEYBOARD"
SIMPLIFIED_MODEL = False
DISPLAY_DATA = "ON_TERMINAL" # Options : "ON_GUI", "ON_TERMINAL", "NO"
REFRESH_EVERY_FRAME = 240
ADD_DEBUG_CUBES = False
CURRENT_TRACK = "OBJ_MODEL"  # Options: "TESTING_GRID", "OVAL_RACE_TRACK", "STRAIGHT_LINE", "OBJ_MODEL"
TRACK_COLOR_SEGMENTS = "SAME" # Options: "ALTERNATING", "RANDOM", "SAME"
REFRESH_SENSORS_EVERY_FRAME = 3
DISPLAY_SENSORS = False

################################### Car Behaviour

MAX_SPEED_KMH = 370
MAX_SPEED_MPS = MAX_SPEED_KMH / 3.6

CAR_INITIAL_POSITION = (-430,1084,164) #(-420,1066,163) (135,-199,149) 150,-290,162) other points on the track

INITIAL_ANGLE_OF_CAR = 120

################################### Look

SKY_COLOR = (0.615,0.850,0.956)
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
    (-1.8, 0.24, 0.5, 1.0, 0.4, CAR_COLOR) ,  # Rear wing assembly
    (-0.6, 0.3, 0.7, 0.2, 0.4, CAR_COLOR),  # Center cell upper
    (0, 0.3, 0.3, 0.2, 0.4,HELMET_COLOR),  # helmet
]

################################### initialize

camera_rotation_angle = 0.0
