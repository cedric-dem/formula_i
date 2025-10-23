import sys
from pathlib import Path
import numpy as np
import open3d as o3d

MODEL_PATH = Path("f1_tracks/glb_files/spa_francorchamps.glb")

def set_background(win, rgba=(0, 0, 0, 1)):
    col = np.array(rgba, dtype=np.float32)
    if hasattr(win, "set_background_color"):
        try:
            win.set_background_color(col)
            return
        except Exception:
            pass
    if hasattr(win, "set_background"):
        try:
            try:
                win.set_background(col, o3d.geometry.Image())
            except TypeError:
                win.set_background(col)
            return
        except Exception:
            pass

def set_toggle(win, names, value=True):

    # 1) Try direct callable like win.show_foo(True)
    for n in names:
        attr = getattr(win, n, None)
        if callable(attr):
            try:
                attr(value)
                return True
            except Exception:
                pass
    # 2) Try setter like win.set_show_foo(True)
    for n in names:
        setter = getattr(win, ("set_" + n), None)
        if callable(setter):
            try:
                setter(value)
                return True
            except Exception:
                pass
    # 3) Try property assignment: win.show_foo = True
    for n in names:
        if hasattr(win, n):
            try:
                setattr(win, n, value)
                return True
            except Exception:
                pass
    return False

def main():
    if not MODEL_PATH.exists():
        print(f"Error: file not found: {MODEL_PATH.resolve()}")
        sys.exit(1)

    model = o3d.io.read_triangle_model(str(MODEL_PATH))
    if model is None:
        print("Error: failed to load model (unsupported/corrupted GLB).")
        sys.exit(1)

    try:
        app = o3d.visualization.gui.Application.instance
        app.initialize()

        win = o3d.visualization.O3DVisualizer(
            title=f"GLB Viewer – {MODEL_PATH.name}",
            width=1280,
            height=800
        )

        set_background(win, (0, 0, 0, 1))
        set_toggle(win, ["show_skybox"])
        set_toggle(win, ["show_axes"])

        set_toggle(win, ["show_ground", "show_ground_plane"])

        win.add_model("spa_v2", model)

        # Frame camera
        if hasattr(win, "reset_camera_to_default"):
            win.reset_camera_to_default()

        app.add_window(win)
        app.run()

    except Exception as e:
        print(f"[Info] O3DVisualizer had issues ({e}). Falling back to simple viewer.")
        try:
            o3d.visualization.draw(model)
        except Exception as e2:
            print(f"Fallback viewer failed as well: {e2}")
            sys.exit(1)

if __name__ == "__main__":
    main()
