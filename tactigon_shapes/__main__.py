import argparse
import signal
from . import TactigonShapes

def run():
    parser = argparse.ArgumentParser("Tactigon Shapes")
    parser.add_argument("-A", "--address", help="Server address", type=str, default="0.0.0.0")
    parser.add_argument("-P", "--port", help="Server port", type=int, default=5123)
    args = parser.parse_args()

    server = TactigonShapes(args.address, args.port, True)

    signal.signal(signal.SIGTERM, lambda s, h: server.stop())
    signal.signal(signal.SIGINT, lambda s, h: server.stop())

    try:
        server.serve()
    except KeyboardInterrupt:
        server.stop()

if __name__ == "__main__":
    run()