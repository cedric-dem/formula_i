from track_handler_module.misc import *

if __name__ == "__main__":
	# load model
	model = get_model()

	# load layout
	track_layout_markers = read_csv_file(FILENAME_TEMP_LAYOUT)

	display_map_and_markers(model, track_layout_markers)
