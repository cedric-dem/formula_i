from track_handler_module.misc import *

import trimesh

mesh = trimesh.load(OUTPUT_GLB_FILE, force='mesh')

mesh.export("result.obj")
