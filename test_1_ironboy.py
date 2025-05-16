import time
from tactigon_ironboy import IronBoy,IronBoyCommand,IronBoyConfig


cfg = IronBoyConfig("TactigonIronBoy", "C8:0A:39:4F:C7:B4")
ironboy=IronBoy(cfg)
try:
    
    with IronBoy(cfg) as ironboy:
        while True:
            if not ironboy.connected:
                print("Connecting...")
                time.sleep(1)
                continue
            
            print(f"Connected to: {ironboy.client}")

            cmd = ironboy.send_command(IronBoyCommand.WAVE)
            print(cmd)
            
            timeout = 10
            start_time = time.time()
            while ironboy.executing:
                if time.time() - start_time > timeout:
                    print("Timeout")
                    break
                time.sleep(0.1)

except KeyboardInterrupt:
    print("\nUser stopped the program.")
except Exception as e:
    print(f"Error: {e}")