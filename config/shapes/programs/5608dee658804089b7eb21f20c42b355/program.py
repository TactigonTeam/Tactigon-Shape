# Shapes by Next Industries

import time
import random
from numbers import Number
from datetime import datetime
from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue
from tactigon_shapes.modules.braccio.extension import BraccioInterface, CommandStatus, Wrist, Gripper
from tactigon_shapes.modules.zion.extension import ZionInterface, Scope, AlarmSearchStatus, AlarmSeverity
from tactigon_shapes.modules.tskin.models import TSkin, Gesture, Touch, OneFingerGesture, TwoFingerGesture, HotWord, TSpeechObject, TSpeech
from pynput.keyboard import Controller as KeyboardController, HotKey, KeyCode
from typing import List, Optional, Union, Any

def dai_carta():
    global prima_mano, puo_giocare, nuova_carta, mazziere_vuole_carte, carte_mazziere, mie_carte
    nuova_carta = random.randint(1, 13)
    if nuova_carta > 10:
        nuova_carta = 10
    return nuova_carta


prima_mano = True
puo_giocare = False
mazziere_vuole_carte = False
mie_carte = 0
nuova_carta = 0
carte_mazziere = 0

# This is the main function that runs your code. Any
# code blocks you add to this section will be executed.


def check_gesture(gesture: Optional[Gesture], gesture_to_find: str) -> bool:
    if not gesture:
        return False

    return gesture.gesture == gesture_to_find

def check_touch(touch: Optional[Touch], finger_gesture: str, actions: List[ShapesPostAction]) -> bool:
    if not touch:
        return False
    _g_one = None
    try:
        _g_one = OneFingerGesture[finger_gesture]
        if touch.one_finger == _g_one:
            return True
    except:
        pass
    _g_two = None
    try:
        _g_two = TwoFingerGesture[finger_gesture]
        if touch.two_finger == _g_two:
            return True
    except:
        pass
    return False

def check_speech(tskin: TSkin, logging_queue: LoggingQueue, hotwords: List[Union[HotWord, List[HotWord]]]):
    def build_tspeech(hws: List[Union[HotWord, List[HotWord]]]) -> Optional[TSpeechObject]:
        if not hws:
            return None

        hw, *rest = hws

        return TSpeechObject(
            [
                TSpeech(hw, build_tspeech(rest))
            ]
        )

    tspeech = build_tspeech(hotwords)

    if tspeech and tskin.can_listen:
        debug(logging_queue, f"Waiting for command...")
        r = tskin.listen(tspeech)
        if r:
            debug(logging_queue, "Listening....")
            text_so_far = ""
            t = None
            while True:
                t = tskin.transcription

                if t:
                    break

                if text_so_far != tskin.text_so_far:
                    text_so_far = tskin.text_so_far
                    debug(logging_queue, f"Listening: {text_so_far}")
                time.sleep(tskin.TICK)

            if t and t.path is not None:
                debug(logging_queue, f"Command found: {[hw.word for hw in t.path]}")
                return [hw.word for hw in t.path]

    debug(logging_queue, "Cannot listen...")
    return []

def record_audio(tskin: TSkin, filename: str, seconds: float):
    tskin.record(filename, seconds)

    while tskin.is_recording:
        time.sleep(tskin.TICK)

def keyboard_press(keyboard: KeyboardController, commands: List[KeyCode]):
    for k in commands:
        _k = k.char if isinstance(k, KeyCode) and k.char else k
        keyboard.press(_k)
    for k in commands[::-1]:
        _k = k.char if isinstance(k, KeyCode) and k.char else k
        keyboard.release(_k)

def braccio_move(braccio: Optional[BraccioInterface], logging_queue: LoggingQueue, x: float, y: float, z: float):
    if braccio:
        res = braccio.move(x, y, z)
        if res:
            if res[0]:
                debug(logging_queue, f"Braccio command executed in {round(res[2], 2)}s.")
            else:
                debug(logging_queue, f"Braccio command error: {res[1].name}")
        else:
            debug(logging_queue, "Braccio not connected")
    else:
        debug(logging_queue, "Braccio not configured")

def braccio_wrist(braccio: Optional[BraccioInterface], logging_queue: LoggingQueue, wrist: Wrist):
    if braccio:
        res = braccio.wrist(wrist)
        if res:
            if res[0]:
                debug(logging_queue, f"Braccio command executed in {round(res[2], 2)}s.")
            else:
                debug(logging_queue, f"Braccio command error: {res[1].name}")
        else:
            debug(logging_queue, "Braccio not connected")
    else:
        debug(logging_queue, "Braccio not configured")

def braccio_gripper(braccio: Optional[BraccioInterface], logging_queue: LoggingQueue, gripper: Gripper):
    if braccio:
        res = braccio.gripper(gripper)
        if res:
            if res[0]:
                debug(logging_queue, f"Braccio command executed in {round(res[2], 2)}s.")
            else:
                debug(logging_queue, f"Braccio command error: {res[1].name}")
        else:
            debug(logging_queue, "Braccio not connected")
    else:
        debug(logging_queue, "Braccio not configured")

def zion_device_last_telemetry(zion: Optional[ZionInterface], device_id: str, keys: str) -> dict:
    if not zion:
        return {}

    data = zion.device_last_telemetry(device_id, keys)

    if not data:
        return {}

    return data

def zion_device_attr(zion: Optional[ZionInterface], device_id: str, scope: Scope, keys: str) -> dict:
    if not zion:
        return {}

    data = zion.device_attr(device_id, scope, keys)

    if not data:
        return {}

    return data

def zion_device_alarm(zion: Optional[ZionInterface], device_id: str, severity: AlarmSeverity, search_status: AlarmSearchStatus) -> List[dict]:
    if not zion:
        return []

    data = zion.device_alarm(device_id, severity, search_status)

    if not data:
        return []

    return data

def zion_send_device_last_telemetry(zion: Optional[ZionInterface], device_id: str, key: str, data) -> bool:
    if not zion:
        return False

    payload = {}
    payload[key] = data

    return zion.send_device_last_telemetry(device_id, payload)

def zion_send_device_attr(zion: Optional[ZionInterface], device_id: str, scope: Scope, key: str, data) -> bool:
    if not zion:
        return False

    payload = {}
    payload[key] = data

    return zion.send_device_attr(device_id, payload, scope)

def zion_send_device_alarm(zion: Optional[ZionInterface], device_id: str, name: str) -> bool:
    if not zion:
        return False

    return zion.upsert_device_alarm(device_id, name, name)

def debug(logging_queue: LoggingQueue, msg: Optional[Any]):
    logging_queue.debug(str(msg))

def reset_touch(tskin: TSkin):
        if tskin.touch_preserve:
            _ = tskin.touch

# This is the main function that runs your code. Any
# code blocks you add to this section will be executed.
def app(tskin: TSkin, keyboard: KeyboardController, braccio: Optional[BraccioInterface], zion: Optional[ZionInterface], actions: List[ShapesPostAction], logging_queue: LoggingQueue):
    global mie_carte
    global nuova_carta
    global puo_giocare
    global mazziere_vuole_carte
    global carte_mazziere
    global prima_mano

    gesture = tskin.gesture
    touch = tskin.touch
    if puo_giocare:
        if check_gesture(gesture, "swipe_r"):
            if prima_mano:
                debug(logging_queue, 'Dammi le carte')
                for count in range(2):
                    mie_carte = (mie_carte if isinstance(mie_carte, Number) else 0) + dai_carta()
                prima_mano = False
                debug(logging_queue, ('Hai in mano:' + str(mie_carte)))
            else:
                mie_carte = (mie_carte if isinstance(mie_carte, Number) else 0) + dai_carta()
                debug(logging_queue, ('Hai in mano:' + str(mie_carte)))
        elif check_gesture(gesture, "swipe_l"):
            debug(logging_queue, 'Ho finito')
            puo_giocare = False
            mazziere_vuole_carte = True
    elif mazziere_vuole_carte:
        for count2 in range(2):
            carte_mazziere = (carte_mazziere if isinstance(carte_mazziere, Number) else 0) + dai_carta()
        if carte_mazziere < 15:
            carte_mazziere = (carte_mazziere if isinstance(carte_mazziere, Number) else 0) + dai_carta()
        else:
            debug(logging_queue, (''.join([str(x) for x in ['Le tue carte: ', mie_carte, ' carte del mazziere: ', carte_mazziere]])))
            if mie_carte > 21:
                debug(logging_queue, 'Ho perso....')
            elif carte_mazziere > 21:
                debug(logging_queue, 'Ho vinto io!')
            else:
                if carte_mazziere > mie_carte:
                    debug(logging_queue, 'Vince il banco...')
                else:
                    debug(logging_queue, 'Ho vinto io!')
            puo_giocare = False
            mazziere_vuole_carte = False
    else:
        if check_gesture(gesture, "up"):
            debug(logging_queue, 'Nuovo gioco!')
            puo_giocare = True
            mazziere_vuole_carte = False
