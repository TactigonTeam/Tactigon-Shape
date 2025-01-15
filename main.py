import argparse
from tactigon_shapes import Server
from tactigon_shapes.utility import is_deep_speech_supported, install_package

if __name__ == "__main__":
    voice_packages = {
        "deepspeech-tflite==0.9.3": True,
        "tactigon_speech==5.0.8.post1": True,
    }

    if is_deep_speech_supported():
        for package, no_deps in voice_packages.items():
            install_package(package, no_deps)


    parser = argparse.ArgumentParser("Tactigon Shapes")
    parser.add_argument("-A", "--address", help="Server address", type=str, default="127.0.0.1")
    parser.add_argument("-P", "--port", help="Server port", type=int, default=5123)
    args = parser.parse_args()

    server = Server(args.address, args.port, True)
    server.serve()