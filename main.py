import argparse
from tactigon_shapes import Server

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Tactigon Shapes")
    parser.add_argument("-A", "--address", help="Server address", type=str, default="127.0.0.1")
    parser.add_argument("-P", "--port", help="Server port", type=int, default=5123)
    args = parser.parse_args()

    server = Server("0.0.0.0", args.port, True)
    server = Server(args.address, args.port, True)
    server.serve()