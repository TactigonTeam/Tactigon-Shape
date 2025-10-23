from flask import current_app

from typing import Optional

from tactigon_shapes.modules.zion.extension import ZionInterface

def get_zion_interface() -> Optional[ZionInterface]:
    if ZionInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[ZionInterface.__name__], ZionInterface):
        return current_app.extensions[ZionInterface.__name__]
    
    return None