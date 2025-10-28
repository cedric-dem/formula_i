from track_handler_module.misc import *
import time

if __name__ == "__main__":
	t0 = time.time()
	trackers_information = create_markers_list()
	t1 = time.time()
	print('====> time taken', round(t1-t0,2))

	in_csv_file(trackers_information, FILENAME_TEMP_LAYOUT)
