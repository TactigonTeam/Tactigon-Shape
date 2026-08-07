from flask import current_app

from tactigon_shapes.modules.chords.extension import ChordsLLMInterface, ChordsMLInterface

def get_chords_llm_interface() -> ChordsLLMInterface | None:
    if ChordsLLMInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[ChordsLLMInterface.__name__], ChordsLLMInterface):
        return current_app.extensions[ChordsLLMInterface.__name__]
    return None


def get_chords_ml_interface() -> ChordsMLInterface | None:
    if ChordsMLInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[ChordsMLInterface.__name__], ChordsMLInterface):
        return current_app.extensions[ChordsMLInterface.__name__]
    return None