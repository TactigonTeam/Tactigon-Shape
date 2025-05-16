import os
import time
from tactigon_shapes.modules.ironBoy.extension import IronBoyInterface, IronBoyCommand
if __name__ == "__main__":

    ironboy = IronBoyInterface(r"C:\Users\LucaGabello\Desktop\Tactigon-Shape\config\ironBoy")

    ironboy.start()
    time.sleep(5)

    cmd=ironboy.command(IronBoyCommand.WAVE)
    print(cmd)
