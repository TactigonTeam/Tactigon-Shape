import sys

def get_python_version() -> list[int]:
    return [int(v) for v in sys.version.split(" ")[0].split(".")]

def supports_voice() -> bool:
    try:
        import tactigon_speech # type: ignore
        return True
    except:
        return False

def get_tactigon_speech_version() -> None | str:
    try:
        from tactigon_speech import __version__ as tactigon_speech_version # type: ignore
    except:
        tactigon_speech_version = None

    return tactigon_speech_version