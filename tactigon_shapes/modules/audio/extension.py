import os
import time
import logging
from werkzeug.datastructures import FileStorage

from tactigon_shapes.modules.tskin.models import TSkin
from tactigon_shapes.modules.file_manager.extension import FileManager, ItemAlreadyExists
# adatta il percorso all'ubicazione reale di SocketCommand nella tua libreria
from tactigon_skin.models.socket import SocketCommand

logger = logging.getLogger(__name__)


class AudioRecorder:
    DIRECTORY = "Audio_Recordings"
    EXTENSION = ".wav"
    SHARED_DIR = "/app/audio"  # volume condiviso con il container speech

    def __init__(self, tskin: TSkin):
        self.tskin = tskin

    def check_filename(self, filename: str) -> str:
        name = os.path.basename(filename)
        if not name.lower().endswith(self.EXTENSION):
            name += self.EXTENSION
        return name

    def record(self, filename: str, duration: float) -> bool:
        if not self.tskin.can_listen:
            return False

        name = self.check_filename(filename)
        full_path = os.path.join(self.SHARED_DIR, name)

        if not self.tskin.record(name, duration):
            return False

        deadline = time.monotonic() + duration
        while self.tskin.is_recording and time.monotonic() < deadline:
            time.sleep(self.tskin.TICK)

        if self.tskin.is_recording:
            self.tskin.stop()
            self.tskin.on_response(SocketCommand.STOP, {})

        logger.info("Recording finished: %s", full_path)

        if not os.path.exists(full_path):
            logger.error("Recorded file not found: %s", full_path)
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