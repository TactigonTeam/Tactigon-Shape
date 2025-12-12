import argparse
from tactigon_shapes import DockerServer

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Tactigon Shapes")
    parser.add_argument("-A", "--address", help="Server address", type=str, default="0.0.0.0")
    parser.add_argument("-P", "--port", help="Server port", type=int, default=5123)
    args = parser.parse_args()

    server = DockerServer(args.address, args.port, True)
    server.serve()