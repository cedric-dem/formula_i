from track_handler_module.misc import *

if __name__ == "__main__":
	model = get_model()

	adapt_markers_to_model(model, FILENAME_TEMP_LAYOUT)
