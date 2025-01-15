function loadCustomBlocks(response) {
    const gestures = response ? response.gestures : []
    const modKeys = response ? response.modKeys : []
    const funcKeys = response ? response.funckeys : []
    const taps = response ? response.taps : []
    const wristOptions = response ? response.wristOptions : []
    const gripperOptions = response ? response.gripperOptions : []
    const speechs = response && response.speechs ? response.speechs : []

    loadTSkinBlocks(gestures, taps);
    loadSpeechBlocks(speechs);
    loadKeyboardBlocks(funcKeys, modKeys);
    loadBraccioBlocks(wristOptions, gripperOptions);

    Blockly.Blocks['tactigon_shape_function'] = {
        init: function () {
            this.jsonInit({
                "type": "tactigon_shape_function",
                "message0": "Tactigon Main %1",
                "args0": [
                    {
                        "type": "field_input",
                        "name": "NAME",
                        "text": "app",
                        "editable": false
                    }
                ],
                "message1": "do %1",
                "args1": [
                    {
                        "type": "input_statement",
                        "name": "BODY"
                    }
                ],
                "inputsInline": false,
                "colour": 230,
                "tooltip": "Main function",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['tactigon_shape_debug'] = {
        init: function () {
            this.jsonInit({
                "type": "tactigon_shape_debug",
                "message0": "Debug %1",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "TEXT",
                        "check": "String"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#bce261",
                "tooltip": "Send a message to the terminal",
                "helpUrl": ""
            });
        }
    };
}

//Carica i blocchi relativi a TSkin
function loadTSkinBlocks(gestures, taps, speechs, speech_api) {
    Blockly.Blocks['tskin_gesture_list'] = {
        init: function () {
            this.jsonInit({
                "type": "tskin_gesture_list",
                "message0": "%1 gesture",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "gesture",
                        "options": gestures
                    }
                ],
                "output": "Boolean",
                "colour": "#EB6152",
                "tooltip": "Possible gesture found by Tactigon Skin",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['tskin_take_angle'] = {
        init: function () {
            this.jsonInit({
                "type": "tskin_take_angle",
                "message0": "take  %1 angle",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "angle",
                        "options": [
                            [
                                "Roll",
                                "roll"
                            ],
                            [
                                "Pitch",
                                "pitch"
                            ],
                            [
                                "Yaw",
                                "yaw"
                            ]
                        ]
                    }
                ],
                "output": "Number",
                "colour": "#EB6152",
                "tooltip": "Get Tactigon Skin rotation angle",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['tskin_take_gyro'] = {
        init: function () {
            this.jsonInit({
                "type": "tskin_take_gyro",
                "message0": "take  %1 gyro",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "gyro",
                        "options": [
                            [
                                "x-axis",
                                "x"
                            ],
                            [
                                "y-axis",
                                "y"
                            ],
                            [
                                "z-axis",
                                "z"
                            ]
                        ]
                    }
                ],
                "output": "Number",
                "colour": "#EB6152",
                "tooltip": "Get Tactigon Skin gyroscopic axis",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['tskin_touch_list'] = {
        init: function () {
            this.jsonInit({
                "type": "tskin_touch_list",
                "message0": "%1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "touch",
                        "options": taps
                    }
                ],
                "output": "Boolean",
                "colour": "#EB6152",
                "tooltip": "Get Tactigon Skin touchpad gesture",
                "helpUrl": ""
            });
        }
    };
}

function loadSpeechBlocks(speechs) {
    args = []
    message = "Voice command:"

    for (var i=0; i<speechs.length; i++){

        message += " %" + (i + 1);

        if (i==0) {
            args.push({
                "type": "field_dropdown",
                "name": "FIELD_0",
                "options": speechs[i]["_init_"].map((k) => [k, k])
            });
        }
        else {
            optionMapping = {};

            Object.keys(speechs[i]).forEach(element => {
                optionMapping[element] = speechs[i][element].map((k) => [k, k]);
            });

            args.push({
                "type": "field_dependent_dropdown",
                "name": "FIELD_" + i,
                "parentName": "FIELD_" + (i - 1),
                "optionMapping": optionMapping,
                'defaultOptions': [['---', '']],
            });
        }
    }

    Blockly.Blocks['tskin_listen'] = {
        init: function(){
            this.jsonInit({
                "type": "tskin_listen",
                "message0": message,
                'args0': args,
                "output": "List",
                "colour": "#EB6152",
                "tooltip": "Use Tactigon Skin to listen for commands",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['tskin_record'] = {
        init: function () {
            this.jsonInit({
                "type": "tskin_record",
                "message0": "Record on %1 for %2 seconds",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "filename",
                        "check": "String"
                    },
                    {
                        "type": "input_value",
                        "name": "seconds",
                        "check": "Number"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#EB6152",
                "tooltip": "Use Tactigon Skin to record audio",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['tskin_play'] = {
        init: function () {
            this.jsonInit({
                "type": "tskin_play",
                "message0": "Play file audio %1",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "filename",
                        "check": "String"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#EB6152",
                "tooltip": "Use Tactigon Skin to play audio",
                "helpUrl": ""
            });
        }
    };
}

// Carica i blocchi relativi a Keyboard
function loadKeyboardBlocks(funcKeys, modKeys) {
    Blockly.Blocks['keyboard_press'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_press",
                "message0": "Press %1",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "NAME",
                        "check": "KeyboardShortcut"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#c2c2c2",
                "tooltip": "Press a key on the keyboard",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['keyboard_funckey'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_funckey",
                "message0": "Fn Key: %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "function_key",
                        "options": funcKeys
                    }
                ],
                "output": "KeyboardShortcut",
                "colour": "#c2c2c2",
                "tooltip": "Enter a single function key",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['keyboard_mod_plus_funckey'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_mod_plus_funckey",
                "message0": "Mod %1 + Fn Key %2",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "mod_key",
                        "options": modKeys
                    },
                    {
                        "type": "field_dropdown",
                        "name": "function_key",
                        "options": funcKeys
                    }
                ],
                "output": "KeyboardShortcut",
                "colour": "#c2c2c2",
                "tooltip": "Combination of a modifier key and a function key",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['keyboard_mod_plus_mod_plus_funckey'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_mod_plus_mod_plus_funckey",
                "message0": "Mod %1 + Mod %2 + Fn Key %3",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "mod_key_1",
                        "options": modKeys
                    },
                    {
                        "type": "field_dropdown",
                        "name": "mod_key_2",
                        "options": modKeys
                    },
                    {
                        "type": "field_dropdown",
                        "name": "function_key",
                        "options": funcKeys
                    }
                ],
                "output": "KeyboardShortcut",
                "colour": "#c2c2c2",
                "tooltip": "Combination of 2 modifier keys and a function key",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['keyboard_key'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_key",
                "message0": "Key: %1",
                "args0": [
                    {
                        "type": "field_input",
                        "name": "LETTER",
                        "text": "a"
                    }
                ],
                "output": "KeyboardShortcut",
                "colour": "#c2c2c2",
                "tooltip": "Enter a single keyboard letter",
                "helpUrl": ""
            })

            var field = this.getField('LETTER');
            field.setValidator(this.validateLetter);
        },
        validateLetter: function (newValue) {
            if (newValue.length === 1 && /^[a-zA-Z0-9]$/.test(newValue)) {
                return newValue.toLowerCase(); // Convert to uppercase for consistency
            }
            return null; // Invalid input
        }
    };

    Blockly.Blocks['keyboard_mod_plus_key'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_mod_plus_key",
                "message0": "Mod Key %1 + Key %2",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "mod_key",
                        "options": modKeys
                    },
                    {
                        "type": "field_input",
                        "name": "LETTER",
                        "text": "a"
                    }
                ],
                "output": "KeyboardShortcut",
                "colour": "#c2c2c2",
                "tooltip": "Combination of a modifier key and a letter",
                "helpUrl": ""
            });
            var field = this.getField('LETTER');
            field.setValidator(this.validateLetter);
        },
        validateLetter: function (newValue) {
            if (newValue.length === 1 && /^[a-zA-Z]$/.test(newValue)) {
                return newValue.toLowerCase(); // Convert to uppercase for consistency
            }
            return null; // Invalid input
        }
    };

    Blockly.Blocks['keyboard_mod_plus_mod_plus_key'] = {
        init: function () {
            this.jsonInit({
                "type": "keyboard_mod_plus_mod_plus_key",
                "message0": "Mod Key %1 + Mod Key %2 + Key %3",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "mod_key_1",
                        "options": modKeys
                    },
                    {
                        "type": "field_dropdown",
                        "name": "mod_key_2",
                        "options": modKeys
                    },
                    {
                        "type": "field_input",
                        "name": "LETTER",
                        "text": "a"
                    }
                ],
                "output": "KeyboardShortcut",
                "colour": "#c2c2c2",
                "tooltip": "Combination of 2 modifier keys and a letter",
                "helpUrl": ""
            });

            var field = this.getField('LETTER');
            field.setValidator(this.validateLetter);
        },
        validateLetter: function (newValue) {
            if (newValue.length === 1 && /^[a-zA-Z]$/.test(newValue)) {
                return newValue.toLowerCase(); // Convert to uppercase for consistency
            }
            return null; // Invalid input
        }
    };
}

function loadBraccioBlocks(wristOptions, gripperOptions) {
    Blockly.Blocks['braccio_move'] = {
        init: function () {
            this.jsonInit({
                "type": "braccio_move",
                "message0": "Move (x: %1, y: %2, z: %3)",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "x",
                        "check": "Number"
                    },
                    {
                        "type": "input_value",
                        "name": "y",
                        "check": "Number"
                    },
                    {
                        "type": "input_value",
                        "name": "z",
                        "check": "Number"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#cb6434",
                "tooltip": "Move Braccio to the given coordinates",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['braccio_wrist'] = {
        init: function () {
            this.jsonInit({
                "type": "braccio_wrist",
                "message0": "Wrist %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "wrist",
                        "options": wristOptions
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#cb6434",
                "tooltip": "Move Braccio wrist",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['braccio_gripper'] = {
        init: function () {
            this.jsonInit({
                "type": "braccio_gripper",
                "message0": "Gripper %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "gripper",
                        "options": gripperOptions
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": "#cb6434",
                "tooltip": "Move Braccio gripper",
                "helpUrl": ""
            });
        }
    };
}

function defineCustomGenerators() {
    Blockly.Python.INDENT = '    ';

    python.pythonGenerator.forBlock['tactigon_shape_function'] = function (block, generator) {
        Blockly.Python.definitions_['import_requests'] = `
# Shapes by Next Industries

import time
from datetime import datetime
from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue
from tactigon_shapes.modules.braccio.extension import BraccioInterface, CommandStatus, Wrist, Gripper
from tactigon_shapes.utility import has_voice

if has_voice():
    from tactigon_shapes.modules.tskin.models import HotWord, TSpeechObject, TSpeech

from tactigon_shapes.modules.tskin.models import TSkin, Gesture, Touch, OneFingerGesture, TwoFingerGesture
from pynput.keyboard import Controller as KeyboardController, HotKey, KeyCode
from typing import List, Optional, Union`;
        
        var libs = `
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

if has_voice():
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

def debug(logging_queue: LoggingQueue, msg: str):
    logging_queue.debug(msg)

def reset_touch(tskin: TSkin):
        if tskin.touch_preserve:
            _ = tskin.touch

# This is the main function that runs your code. Any
# code blocks you add to this section will be executed.
`;
        var statements_body = Blockly.Python.statementToCode(block, 'BODY');

        if (!statements_body) {
            statements_body = "\tpass"
        }

        let variables = block.workspace.getAllVariables().map((v) => {
            return generator.INDENT + "global " + v.name;
        }).join('\n');

        var code = libs + 'def app(tskin: TSkin, keyboard: KeyboardController, braccio: Optional[BraccioInterface], actions: List[ShapesPostAction], logging_queue: LoggingQueue):\n' + 
            variables + '\n' + "\n" +
            Blockly.Python.INDENT + "gesture = tskin.gesture\n" +
            Blockly.Python.INDENT + "touch = tskin.touch\n" +
            statements_body + '\n';
        return code;
    };


    python.pythonGenerator.forBlock['tactigon_shape_debug'] = function (block, generator) {
        var message = generator.valueToCode(block, 'TEXT', python.Order.ATOMIC);
        var code = `debug(logging_queue, ${message})\n`;
        return code;
    };

    defineTSkinGenerators()
    defineSpeechGenerators();
    defineKeyboardGenerators();
    defineBraccioGenerators();
}

function defineTSkinGenerators(){
    python.pythonGenerator.forBlock['tskin_gesture_list'] = function (block) {
        var gesture = block.getFieldValue('gesture');
        var code = `check_gesture(gesture, "${gesture}")`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_take_angle'] = function (block, generator) {
        var angle = block.getFieldValue('angle');
        var code = `tskin.angle and tskin.angle.${angle}`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_take_gyro'] = function (block, generator) {
        var gyro = block.getFieldValue('gyro');
        var code = `tskin.gyro and tskin.gyro.${gyro}`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_touch_list'] = function (block, generator) {
        var touchType = block.getFieldValue('touch');

        var code = `check_touch(touch, "${touchType}", actions)`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineSpeechGenerators(){
    python.pythonGenerator.forBlock['tskin_listen'] = function (block) {
        let args = block.inputList[0].fieldRow
            .filter((f) => f.selectedOption && f.selectedOption[1] != "")
            .map((f) => {
                if (f.selectedOption[1] == "---"){
                    return `[${f.optionMapping.position.filter((o) => o[0] != "---").map((o) => `HotWord("${o[0]}")`).join(", ")}]`
                }
                return `HotWord("${f.selectedOption[1]}")`;
            })
            .join(", ");

        var code = `check_speech(tskin, logging_queue, [${args}])`
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_record'] = function (block, generator) {
        debugger;
        let filename = generator.valueToCode(block, 'filename', python.Order.ATOMIC);
        let seconds = generator.valueToCode(block, 'seconds', python.Order.ATOMIC);

        return `record_audio(tskin, ${filename}, ${seconds})\n`
    };

    python.pythonGenerator.forBlock['tskin_play'] = function (block, generator) {
        let filename = generator.valueToCode(block, 'filename', python.Order.ATOMIC);

        return `tskin.play(${filename})\n`
    };
}

function defineKeyboardGenerators(){
    python.pythonGenerator.forBlock['keyboard_press'] = function (block, generator) {
        var message = generator.valueToCode(block, 'NAME', python.Order.ATOMIC);
        var code = `keyboard_press(keyboard, HotKey.parse(${message}))\n`;
        return code;
    };

    python.pythonGenerator.forBlock['keyboard_key'] = function (block) {
        var key = block.getFieldValue('LETTER');
        var code = `'${key}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['keyboard_mod_plus_key'] = function (block, generator) {
        var mod = block.getFieldValue('mod_key');
        var key = block.getFieldValue('LETTER');
        var code = `'${mod}${key}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['keyboard_mod_plus_mod_plus_key'] = function (block, generator) {
        var mod1 = block.getFieldValue('mod_key_1');
        var mod2 = block.getFieldValue('mod_key_2');
        var key = block.getFieldValue('LETTER');
        var code = `'${mod1}${mod2}${key}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['keyboard_funckey'] = function (block) {
        var func = block.getFieldValue('function_key');
        var code = `'${func}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['keyboard_mod_plus_funckey'] = function (block, generator) {
        var mod = block.getFieldValue('mod_key');
        var func = block.getFieldValue('function_key');
        var code = `'${mod}${func}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['keyboard_mod_plus_mod_plus_funckey'] = function (block, generator) {
        var mod1 = block.getFieldValue('mod_key_1');
        var mod2 = block.getFieldValue('mod_key_2');
        var func = block.getFieldValue('function_key');
        var code = `'${mod1}${mod2}${func}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineBraccioGenerators(){
    python.pythonGenerator.forBlock['braccio_move'] = function (block, generator) {
        var x = generator.valueToCode(block, 'x', python.Order.ATOMIC);
        var y = generator.valueToCode(block, 'y', python.Order.ATOMIC);
        var z = generator.valueToCode(block, 'z', python.Order.ATOMIC);
        var code = `braccio_move(braccio, logging_queue, ${x}, ${y}, ${z})\n`;
        return code;
    };

    python.pythonGenerator.forBlock['braccio_wrist'] = function (block, generator) {
        var x = block.getFieldValue('wrist');
        var code = `braccio_wrist(braccio, logging_queue, Wrist['${x}'])\n`;
        return code;
    };

    python.pythonGenerator.forBlock['braccio_gripper'] = function (block, generator) {
        var x = block.getFieldValue('gripper');
        var code = `braccio_gripper(braccio, logging_queue, Gripper['${x}'])\n`;
        return code;
    };
}