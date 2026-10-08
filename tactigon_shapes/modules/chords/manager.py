from flask import current_app

from tactigon_shapes.modules.chords.extension import ChordLLMInterface, ChordMLInterface

def get_chord_llm_interface() -> ChordLLMInterface | None:
    if ChordLLMInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[ChordLLMInterface.__name__], ChordLLMInterface):
        return current_app.extensions[ChordLLMInterface.__name__]
    return None


def get_chord_ml_interface() -> ChordMLInterface | None:
    if ChordMLInterface.__name__ in current_app.extensions and isinstance(current_app.extensions[ChordMLInterface.__name__], ChordMLInterface):
        return current_app.extensions[ChordMLInterface.__name__]
    return None