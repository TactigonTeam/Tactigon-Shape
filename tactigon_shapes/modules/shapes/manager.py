from flask import current_app

from typing import Optional

from tactigon_shapes.modules.shapes.extension import ShapesApp

def get_shapes_app() -> Optional[ShapesApp]:
    if ShapesApp.__name__ in current_app.extensions and isinstance(current_app.extensions[ShapesApp.__name__], ShapesApp):
        return current_app.extensions[ShapesApp.__name__]
    
    return None