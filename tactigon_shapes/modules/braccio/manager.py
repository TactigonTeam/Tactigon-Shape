from flask import current_app

from typing import Optional

from tactigon_shapes.modules.braccio.extension import BraccioInterface

def get_braccio_interface() -> Optional[BraccioInterface]:
    if BraccioInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[BraccioInterface.__name__], BraccioInterface):
        return current_app.extensions[BraccioInterface.__name__]
    
    return None