import logging

from tactigon_shapes.modules.chords.extension import ChordsLLMInterface
from tactigon_shapes.modules.chords.models import ChordsLLMConfig, ChordsLLMChatMessage, ChordsLLMChat

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

def main():
    i = ChordsLLMInterface()
    chat = i.new_chat()

    print(chat)
    if not chat:
        return
    
    input("Premere per continuare")

    res = i.stream(chat.chat_id, chat.user_id, "Cosa mi sai dire della pappa?")


if __name__ == "__main__":
    main()