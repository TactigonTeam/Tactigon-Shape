import logging
import types

from tactigon_shapes.modules.bianconiglio.extension import BianconiglioInterface
from tactigon_shapes.modules.bianconiglio.models import BianconiglioConfig


def main():
    # cfg = BianconiglioConfig(
    #     url="http://192.168.1.34:8000",
    #     chord_url="http://192.168.1.222:8080"
    # )

    b = BianconiglioInterface("./config/bianconiglio")
    # TODO: implementare la gestione del context

    resp = b.stream_chat_with_rag("Ciao Mamma")

    if isinstance(resp, types.GeneratorType):
        for l in resp:
            print(l, type(l))



if __name__ == "__main__":
    main()
