from flask import current_app

from typing import Optional

from tactigon_shapes.modules.ros2.extension import Ros2Interface

def get_ros2_interface() -> Optional[Ros2Interface]:
    if Ros2Interface.__name__ in current_app.extensions and isinstance(current_app.extensions[Ros2Interface.__name__], Ros2Interface):
        return current_app.extensions[Ros2Interface.__name__]
    
    return None