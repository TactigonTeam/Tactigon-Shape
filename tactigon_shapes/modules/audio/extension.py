import os
import time
from werkzeug.datastructures import FileStorage

from tactigon_shapes.modules.tskin.models import TSkin
from tactigon_shapes.modules.file_manager.extension import FileManager
class AudioRecorder:
    DIRECTORY = "Audio_Recordings"
    EXTENSION = ".wav"
    SHARED_DIR = "/app/audio" # volume condiviso con il container speech
    
    def __init__(self, tskin: TSkin):
        self.tskin = tskin
        
    @property
    def full_filename(self, filename: str) -> str:
        name = os.path.basename(filename)
        return (name if not name.lower().endswith(self.EXTENSION) 
                else name + self.EXTENSION
                )

    def __init__(self, tskin: TSkin):
        self.tskin = tskin

    def check_filename(self, filename: str) -> str:
        name = os.path.basename(filename)
        if not name.lower().endswith(self.EXTENSION):
            name += self.EXTENSION
        return name

    def record(self, filename: str, duration: float) -> bool:
        from tactigon_shapes.modules.file_manager.extension import FileManager, ItemAlreadyExists
        if not self.tskin.can_listen:
            return False

        name = self.check_filename(filename)
        full_path = os.path.join(self.SHARED_DIR, name)

        if not self.tskin.record(name, duration):
            return False

        while self.tskin.is_recording:
            time.sleep(self.tskin.TICK)
            
        if not os.path.exists(full_path):
            return False

        try:
            FileManager.add_directory(self.DIRECTORY)
        except ItemAlreadyExists:
            pass

        with open(full_path, "rb") as f:
            FileManager.add_file(
                directory=self.DIRECTORY,
                file_path=name,
                content=FileStorage(stream=f, filename=name),
            )

        os.remove(full_path)
        return True