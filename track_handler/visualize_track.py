from track_handler_module.misc import *

if __name__ == "__main__":
	model = get_model()

	trackers_information = create_markers_list()

	track_layout_markers = adapt_markers_to_model(model, trackers_information)

	display_map_and_markers(model, track_layout_markers)
