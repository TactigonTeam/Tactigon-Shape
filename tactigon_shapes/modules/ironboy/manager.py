from flask import current_app

from typing import Optional

from tactigon_shapes.modules.ironboy.extension import IronBoyInterface

def get_ironboy_interface() -> Optional[IronBoyInterface]:
    if IronBoyInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[IronBoyInterface.__name__], IronBoyInterface):
        return current_app.extensions[IronBoyInterface.__name__]
    
    return None