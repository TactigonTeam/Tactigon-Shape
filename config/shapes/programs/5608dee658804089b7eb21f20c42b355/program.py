
# Shapes by Next Industries

import time
import random
import types
import json
import os
from numbers import Number
from datetime import datetime
from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue
from tactigon_shapes.modules.braccio.extension import BraccioInterface, CommandStatus, Wrist, Gripper
from tactigon_shapes.modules.zion.extension import ZionInterface, Scope, AlarmSearchStatus, AlarmSeverity
from tactigon_shapes.modules.ros2.extension import Ros2Interface
from tactigon_shapes.modules.ros2 import models as ros2_models
from tactigon_shapes.modules.tskin.models import TSkin, Gesture, Touch, OneFingerGesture, TwoFingerGesture, TSpeechObject, TSpeech, HotWord
from tactigon_shapes.modules.ironboy.extension import IronBoyInterface, IronBoyCommand
from tactigon_shapes.modules.ginos.extension import GinosInterface
from tactigon_shapes.modules.ginos.models import LLMPromptRequest
from tactigon_shapes.modules.mqtt.extension import MQTTClient
from pynput.keyboard import Controller as KeyboardController, HotKey, KeyCode
from typing import Union, Any
from pathlib import Path


def check_gesture(gesture: Gesture | None, gesture_to_find: str) -> bool:
    if not gesture:
        return False
    
    return gesture.gesture == gesture_to_find

def check_touch(touch: Touch | None, finger_gesture: str) -> bool:
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

def check_speech(tskin: TSkin, logging_queue: LoggingQueue, hotwords: list[Union[HotWord, list[HotWord]]]):
    def build_tspeech(hws: list[Union[HotWord, list[HotWord]]]) -> TSpeechObject | None:
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
            debug(logging_queue, "listening....")
            t = None
            while True:
                t = tskin.transcription

                if t:
                    break

                text_so_far = tskin.text_so_far
                if text_so_far:
                    debug(logging_queue, f"listening: {text_so_far}")
                    
                time.sleep(tskin.TICK)

            if t and t.path is not None:
                debug(logging_queue, f"Command found: {[hw.word for hw in t.path]}")
                return [hw.word for hw in t.path]

    debug(logging_queue, "Cannot listen...")
    return []

def keyboard_press(keyboard: KeyboardController, commands: list[KeyCode]):
    for k in commands:
        _k = k.char if isinstance(k, KeyCode) and k.char else k
        keyboard.press(_k)
    for k in commands[::-1]:
        _k = k.char if isinstance(k, KeyCode) and k.char else k
        keyboard.release(_k)

def braccio_move(braccio: BraccioInterface | None, logging_queue: LoggingQueue, x: float, y: float, z: float):
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

def braccio_wrist(braccio: BraccioInterface | None, logging_queue: LoggingQueue, wrist: Wrist):
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

def braccio_gripper(braccio: BraccioInterface | None, logging_queue: LoggingQueue, gripper: Gripper):
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

def zion_device_last_telemetry(zion: ZionInterface | None, device_id: str, keys: str) -> dict:
    if not zion:
        return {}
    
    data = zion.device_last_telemetry(device_id, keys)

    if not data:
        return {}

    return data

def zion_device_attr(zion: ZionInterface | None, device_id: str, scope: Scope, keys: str) -> dict:
    if not zion:
        return {}
    
    data = zion.device_attr(device_id, scope, keys)

    if not data:
        return {}

    return data

def zion_device_alarm(zion: ZionInterface | None, device_id: str, severity: AlarmSeverity, search_status: AlarmSearchStatus) -> list[dict]:
    if not zion:
        return []
    
    data = zion.device_alarm(device_id, severity, search_status)

    return [alarm.toJSON() for alarm in data]

def zion_send_device_last_telemetry(zion: ZionInterface | None, device_id: str, key: str, data) -> bool:
    if not zion:
        return False

    payload = {}
    payload[key] = data

    return zion.send_device_last_telemetry(device_id, payload)

def zion_delete_device_attr(zion: ZionInterface | None, device_id: str, scope: Scope, keys: str) -> bool:
    if not zion:
        return False

    return zion.delete_device_attr(device_id, scope, keys)

def zion_send_device_attr(zion: ZionInterface | None, device_id: str, scope: Scope, key: str, data) -> bool:
    if not zion:
        return False

    payload = {}
    payload[key] = data

    return zion.send_device_attr(device_id, payload, scope)    

def zion_send_device_alarm(zion: ZionInterface | None, device_id: str, name: str) -> bool:
    if not zion:
        return False

    return zion.upsert_device_alarm(device_id, name, name) 

def debug(logging_queue: LoggingQueue, msg: Any):

    if isinstance(msg,(float)):
        rounded=round(msg,4)
        logging_queue.debug(str(rounded))
    elif isinstance(msg, types.GeneratorType):
        for line in msg:
            logging_queue.prompt(line)
    else:
        logging_queue.debug(str(msg).replace("\n","<br>"))

def iron_boy_command(ironboy: IronBoyInterface | None, logging_queue: LoggingQueue, cmd: IronBoyCommand, reps: int = 1):
    if ironboy:
        command = ironboy.command(cmd,reps)

        if not command:
            debug(logging_queue, "command error")
    else:
        debug(logging_queue, "ironboy not configured")

def ginos_ai_prompt(ginos: GinosInterface | None, prompt: str, context: str = ""):
    if not ginos:
        return

    prompt_object = LLMPromptRequest(
        model=ginos.model,
        prompt=prompt,
    )

    return ginos.prompt(prompt_object)

def ginos_load_dataframe(ginos: GinosInterface | None, directory: str, file_path: str) -> bool:
    if not ginos:
        return False

    return ginos.add_file_to_context(os.path.join(directory, file_path))

def ros2_run(ros2: Ros2Interface | None, command: str):
    if not ros2:
        return

    ros2.run(command)

def ros2_publish(ros2: Ros2Interface | None, topic: str, message: ros2_models.RosMessageTypes):
    if not ros2:
        return
    
    ros2.publish(topic, message)

def mqtt_publish(mqtt: MQTTClient | None, topic: str, payload: Any):
    if not mqtt:
        return
    
    mqtt.publish(topic, payload)

def mqtt_register(mqtt: MQTTClient | None):
    if not mqtt:
        return
    
    mqtt.register()

def mqtt_unregister(mqtt: MQTTClient | None):
    if not mqtt:
        return
    
    mqtt.unregister()


# ---------- Generated code ---------------

from numbers import Number
import random

prima_mano = None
puo_giocare = None
nuova_carta = None
mazziere_vuole_carte = None
carte_mazziere = None
mie_carte = None

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
def tactigon_shape_function(
        tskin: TSkin,
        keyboard: KeyboardController,
        braccio: BraccioInterface | None,
        zion: ZionInterface | None,
        ros2: Ros2Interface | None,
        ironboy: IronBoyInterface | None,
        ginos: GinosInterface | None,
        mqtt: MQTTClient | None,
        logging_queue: LoggingQueue):

    global mie_carte, nuova_carta, puo_giocare, mazziere_vuole_carte, carte_mazziere, prima_mano
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

    return True
