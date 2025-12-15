from tactigon_shapes.modules.ros2.nodes.camera_ws.node import CameraWs
from tactigon_shapes.modules.ros2.nodes.camera_ws.models import CameraWsConfig

if __name__ == "__main__":
    cfg = CameraWsConfig("/camera/image_raw", "127.0.0.1", 5555)

    with CameraWs(cfg) as p:
        input("premere un tasto per chiudere")