from pythreejs import *
from IPython.display import display
import trimesh

mesh = trimesh.load('model.glb')
geometry = BufferGeometry.from_trimesh(mesh)
material = MeshStandardMaterial(color='white', metalness=0.3, roughness=0.7)
mesh_view = Mesh(geometry=geometry, material=material)

scene = Scene(children=[mesh_view, AmbientLight(intensity=0.8)])
renderer = Renderer(camera=PerspectiveCamera(position=[3, 3, 3]),
                    scene=scene, controls=[OrbitControls()],
                    width=800, height=600)
display(renderer)

