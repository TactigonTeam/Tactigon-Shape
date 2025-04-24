# Shapes by Next Industries

import time
from datetime import datetime
from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue
from tactigon_shapes.modules.braccio.extension import BraccioInterface, CommandStatus, Wrist, Gripper
from tactigon_shapes.modules.tskin.models import TSkin, Gesture, Touch, OneFingerGesture, TwoFingerGesture, HotWord, TSpeechObject, TSpeech
from pynput.keyboard import Controller as KeyboardController, HotKey, KeyCode
from typing import List, Optional, Union


positions = []
wrist_pos = True

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

def debug(logging_queue: LoggingQueue, msg: str):
    logging_queue.debug(msg)

def reset_touch(tskin: TSkin):
        if tskin.touch_preserve:
            _ = tskin.touch

# This is the main function that runs your code. Any
# code blocks you add to this section will be executed.
def app(tskin: TSkin, keyboard: KeyboardController, braccio: Optional[BraccioInterface], actions: List[ShapesPostAction], logging_queue: LoggingQueue):
    
    global positions
    global wrist_pos

    gesture = tskin.gesture
    touch = tskin.touch
    if check_touch(touch, "SINGLE_TAP", actions):
        positions = check_speech(tskin, logging_queue, [HotWord("pick"), HotWord("position"), [HotWord("star"), HotWord("circle"), HotWord("square")]])
        if len(positions) == 3:
            if positions[-1] == 'star':
                debug(logging_queue, 'Pick from star')
                braccio_gripper(braccio, logging_queue, Gripper['OPEN'])
                braccio_move(braccio, logging_queue, (-100), 100, 100)
                braccio_move(braccio, logging_queue, (-100), 100, (-20))
                braccio_gripper(braccio, logging_queue, Gripper['CLOSE'])
                braccio_move(braccio, logging_queue, (-100), 100, 100)
                braccio_move(braccio, logging_queue, 0, 100, 100)
                braccio_move(braccio, logging_queue, 0, 100, (-20))
                braccio_gripper(braccio, logging_queue, Gripper['OPEN'])
                braccio_move(braccio, logging_queue, 0, 100, 100)
            elif positions[-1] == 'square':
                debug(logging_queue, 'Pick from square')
                braccio_gripper(braccio, logging_queue, Gripper['OPEN'])
                braccio_move(braccio, logging_queue, 100, 100, 100)
                braccio_move(braccio, logging_queue, 100, 100, (-20))
                braccio_gripper(braccio, logging_queue, Gripper['CLOSE'])
                braccio_move(braccio, logging_queue, 100, 100, 100)
                braccio_move(braccio, logging_queue, 0, 100, 100)
                braccio_move(braccio, logging_queue, 0, 100, (-20))
                braccio_gripper(braccio, logging_queue, Gripper['OPEN'])
                braccio_move(braccio, logging_queue, 0, 100, 100)
    if check_gesture(gesture, "twist"):
        debug(logging_queue, 'Rotate wrist')
        wrist_pos = not wrist_pos
        if wrist_pos:
            braccio_wrist(braccio, logging_queue, Wrist['HORIZONTAL'])
        else:
            braccio_wrist(braccio, logging_queue, Wrist['VERTICAL'])
