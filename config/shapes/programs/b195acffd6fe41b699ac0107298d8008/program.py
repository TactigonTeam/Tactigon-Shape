
# Shapes by Next Industries

import time
import random
import types
from numbers import Number
from datetime import datetime
from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue
from tactigon_shapes.modules.braccio.extension import BraccioInterface, CommandStatus, Wrist, Gripper
from tactigon_shapes.modules.zion.extension import ZionInterface, Scope, AlarmSearchStatus, AlarmSeverity
from tactigon_shapes.modules.tskin.models import TSkin, Gesture, Touch, OneFingerGesture, TwoFingerGesture, HotWord, TSpeechObject, TSpeech
from tactigon_shapes.modules.ironboy.extension import IronBoyInterface, IronBoyCommand
from tactigon_shapes.modules.ginos.extension import GinosInterface
from tactigon_shapes.modules.ginos.models import LLMPromptRequest
from tactigon_shapes.modules.mqtt.extension import MQTTClient
from pynput.keyboard import Controller as KeyboardController, HotKey, KeyCode
from typing import List, Optional, Union, Any
from pathlib import Path


def check_gesture(gesture: Optional[Gesture], gesture_to_find: str) -> bool:
    if not gesture:
        return False
    
    return gesture.gesture == gesture_to_find

def check_touch(touch: Optional[Touch], finger_gesture: str) -> bool:
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

def zion_delete_device_attr(zion: Optional[ZionInterface], device_id: str, scope: Scope, keys: str) -> bool:
    if not zion:
        return False

    return zion.delete_device_attr(device_id, scope, keys)

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

    if isinstance(msg,(float)):
        rounded=round(msg,4)
        logging_queue.debug(str(rounded))
    elif isinstance(msg, types.GeneratorType):
        for line in msg:
            logging_queue.prompt(line)
    else:
        logging_queue.debug(str(msg).replace("\n","<br>"))

def reset_touch(tskin: TSkin):
        if tskin.touch_preserve:
            _ = tskin.touch

def iron_boy_command(ironboy: Optional[IronBoyInterface], logging_queue: LoggingQueue, cmd: IronBoyCommand, reps: int = 1):
    if ironboy:
        command = ironboy.command(cmd,reps)

        if not command:
            debug(logging_queue, "command error")
    else:
        debug(logging_queue, "ironboy not configured")

def ginos_ai_prompt(ginos: Optional[GinosInterface], prompt: str, context: str = ""):
    if not ginos:
        return

    prompt_object = LLMPromptRequest(
        model=ginos.model,
        prompt=prompt,
    )

    return ginos.prompt(prompt_object)

def get_doc_content(file_path):
    path = Path(file_path)
    
    if not path.exists():
        return None
        
    extension = path.suffix.lower()
    
    try:
        if extension == '.txt' or extension == '.md':
            with open(file_path, 'r', encoding='utf-8') as f:
                return f.read()
        else:
            return None
        
    except Exception as e:

        return None

def summarize_text(ginos: Optional[GinosInterface],file_path: str):

    if not ginos:
        return
    
    extracted_file_content = get_doc_content(file_path)
    
    if not extracted_file_content:
        return None

    prompt_per_riassunto = "summarize this text: " + extracted_file_content

    response = ginos_ai_prompt(ginos, prompt_per_riassunto)

    return response


def mqtt_publish(mqtt: Optional[MQTTClient], topic: str, payload: Any):
    if not mqtt:
        return
    
    mqtt.publish(topic, payload)

def mqtt_register(mqtt: Optional[MQTTClient]):
    if not mqtt:
        return
    
    mqtt.register()

def mqtt_unregister(mqtt: Optional[MQTTClient]):
    if not mqtt:
        return
    
    mqtt.unregister()

# ---------- Generated code ---------------

can_move = None
pos_x = None
pos_z = None
pos_wrist = None
pos_gripper = None
pos_y = None


can_move = True
pos_x = 0
pos_y = 50
pos_z = 100
pos_wrist = True
pos_gripper = True

# This is the main function that runs your code. Any
# code blocks you add to this section will be executed.
def tactigon_shape_function(
        tskin: TSkin,
        keyboard: KeyboardController,
        braccio: Optional[BraccioInterface],
        zion: Optional[ZionInterface],
        ironboy: Optional[IronBoyInterface],
        ginos: Optional[GinosInterface],
        mqtt: Optional[MQTTClient],
        logging_queue: LoggingQueue):

    global pos_x, pos_y, pos_z, can_move, pos_wrist, pos_gripper
    gesture = tskin.gesture
    touch = tskin.touch
    if check_gesture(gesture, "up"):
        pos_z = 100
        can_move = True
    elif check_gesture(gesture, "down"):
        pos_z = -20
        can_move = True
    elif check_gesture(gesture, "twist"):
        pos_wrist = not pos_wrist
        if pos_wrist:
            braccio_wrist(braccio, logging_queue, Wrist['HORIZONTAL'])
        else:
            braccio_wrist(braccio, logging_queue, Wrist['VERTICAL'])
    elif check_touch(touch, "SINGLE_TAP"):
        pos_gripper = not pos_gripper
        if pos_gripper:
            braccio_gripper(braccio, logging_queue, Gripper['OPEN'])
        else:
            braccio_gripper(braccio, logging_queue, Gripper['CLOSE'])
    if can_move:
        braccio_move(braccio, logging_queue, pos_x, pos_y, pos_z)
        can_move = False
