from flask import current_app

from tactigon_shapes.modules.chords.extension import ChordsLLMInterface

def get_chords_interface() -> ChordsLLMInterface | None:
    if ChordsLLMInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[ChordsLLMInterface.__name__], ChordsLLMInterface):
        return current_app.extensions[ChordsLLMInterface.__name__]
    return None