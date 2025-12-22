/********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Project Name:
# Tactigon Soul - Shape
# 
# Release date: 30/09/2025
# Release version: 1.0
#
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/


function loadCustomBlocks(response) {
    const gestures = response ? response.gestures : [];
    const modKeys = response ? response.modKeys : [];
    const funcKeys = response ? response.funckeys : [];
    const taps = response ? response.taps : [];
    const wristOptions = response ? response.wristOptions : [];
    const gripperOptions = response ? response.gripperOptions : [];
    const speechs = response ? response.speechs : [];
    const zion = response ? response.zion : [];
    const ros2 = response ? response.ros2 : {};
    const ironboy = response ? response.ironboy : [];
    const ginos = response ? response.ginos: {};
    
    loadTSkinBlocks(gestures, taps);
    if (speechs) {
        loadSpeechBlocks(speechs);
    }
    loadKeyboardBlocks(funcKeys, modKeys);
    loadBraccioBlocks(wristOptions, gripperOptions);
    loadZionBlocks(zion);
    loadRos2Blocks(ros2);
    loadIronBoyBlocks(ironboy);
    loadGinosAIBlocks(ginos);
    loadMQTTBlocks();
    loadDictionaryBlocks();

    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            "type": "tactigon_shape_function",
            "message0": "Loop %1 do %2",
            "args0": [
                {
                    "type": "input_dummy",
                    "name": ""
                },
                {
                    "type": "input_statement",
                    "name": "BODY"
                }
            ],
            "colour": 230,
            "tooltip": "Main function",
            "helpUrl": ""
        },
        {
            "type": "tactigon_shape_setup",
            "message0": "Setup %1 do %2",
            "args0": [
                {
                    "type": "input_dummy",
                    "name": ""
                },
                {
                    "type": "input_statement",
                    "name": "setup_code"
                }
            ],
            "inputsInline": false,
            "colour": 230,
            "tooltip": "Setup function",
            "helpUrl": ""
        },
        {
            "type": "tactigon_shape_close",
            "message0": "Close %1 do %2",
            "args0": [
                {
                    "type": "input_dummy",
                    "name": ""
                },
                {
                    "type": "input_statement",
                    "name": "setup_code"
                }
            ],
            "inputsInline": false,
            "colour": 230,
            "tooltip": "Setup function",
            "helpUrl": ""
        },
        {
            "type": "tactigon_shape_debug",
            "message0": "Debug %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "TEXT"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": "#bce261",
            "tooltip": "Send a message to the terminal",
            "helpUrl": ""
        }
    ]);
    Blockly.common.defineBlocks(blocksDefinitions);
}

function loadDictionaryBlocks(){
    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            "type": "get_dict_property",
            "message0": "Get item %1 from dictionary %2",
            "args0": [
                {
                "type": "input_value",
                "name": "key",
                "check": "String"
                },
                {
                "type": "input_value",
                "name": "dictionary"
                }
            ],
            "output": null,
            "colour": '#636363',
            "tooltip": "Get the value for a key in a dictionary",
            "helpUrl": "",
            "inputsInline": true
        },
        {
            "type": "dict_builder",
            "message0": "Create map %1",
            "args0": [
                {
                    "type": "input_statement",
                    "name": "PAIRS"
                }
            ],
            "output": "Dict",
            "colour": 230,
            "tooltip": "Create a dictionary",
        },
        {
            "type": "dict_pair",
            "message0": "Key %1 value %2",
            "args0": [
                {
                    "type": "input_value",
                    "name": "dict_key",
                    "check": "String"
                },
                {
                    "type": "input_value",
                    "name": "dict_value"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 200,
            "tooltip": "Create a key-value pair for a dictionary",
        },
        {
            "type": "dict_to_json",
            "message0": "jsonify %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "dict",
                    "check": "Dict"
                }
            ],
            "output": "JSONString",
            "colour": 200,
            "tooltip": "Convert dictionary into json string",
            "inputsInline": true
        },
        {
            "type": "json_to_dict",
            "message0": "Dict from json %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "json",
                    "check": "String"
                }
            ],
            "output": "Dict",
            "colour": 200,
            "tooltip": "Convert json string into dictionary",
            "inputsInline": true
        }
    ]);

    Blockly.common.defineBlocks(blocksDefinitions);
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
                "output": "Array",
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

function loadZionBlocks(zion){
    Blockly.Blocks['device_list'] = {
        init: function () {
            this.jsonInit({
                "type": "device_list",
                "message0": "Device %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "device",
                        "options": zion.devices
                    }
                ],
                "output": "ZionDevice",
                "colour": "#EB6152",
                "tooltip": "Devices from Zion",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['scope_list'] = {
        init: function () {
            this.jsonInit({
                "type": "scope_list",
                "message0": "Scope %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "scope",
                        "options": zion.scopes
                    }
                ],
                "output": "ZionScope",
                "colour": "#EB6152",
                "tooltip": "Attribute scope from Zion",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['alarm_severity_list'] = {
        init: function () {
            this.jsonInit({
                "type": "alarm_severity_list",
                "message0": "Alarm severity %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "severity",
                        "options": zion.alarmSeverity
                    }
                ],
                "output": "ZionAlarmSeverity",
                "colour": "#EB6152",
                "tooltip": "Alarm severity from Zion",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['alarm_search_status_list'] = {
        init: function () {
            this.jsonInit({
                "type": "alarm_search_status_list",
                "message0": "Alarm search status %1",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "search_status",
                        "options": zion.alarmSearchStatus
                    }
                ],
                "output": "ZionAlarmSeachStatus",
                "colour": "#EB6152",
                "tooltip": "Alarm search status from Zion",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['delete_device_attr'] = {
        init: function () {
            this.jsonInit({
                "type": "delete_device_attr",
                "tooltip": "delete attribute from device",
                "helpUrl": "",
                "message0": "Delete Attribute from device: %1 Scope %2 Key %3",
                "args0": [          
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "scope",
                        "check": "ZionScope"
                    },
                    {
                        "type": "input_value",
                        "name": "key",
                        "check": "String"
                    }
                ],
                "output": "Dictionary",
                "colour": '#6665DD'

            });
        }
    };

    Blockly.Blocks['device_last_telemetry'] = {
        init: function () {
            this.jsonInit({
                "type": "device_last_telemetry",
                "message0": "Get device last telemetry %1 (Optional) filter key %2",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "keys",
                        "check": "String"
                    }
                ],
                "output": "Dictionary",
                "colour": '#6665DD',
                "tooltip": "Get last telemetry from device",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['device_attr'] = {
        init: function () {
            this.jsonInit({
                "type": "device_attr",
                "message0": "Get device attribute %1 Scope %2 (Optional) filter key %3",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "scope",
                        "check": "ZionScope"
                    },
                    {
                        "type": "input_value",
                        "name": "keys",
                        "check": "String"
                    }
                ],
                "output": "Dictionary",
                "colour": '#6665DD',
                "tooltip": "Get last telemetry from device",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['device_alarm'] = {
        init: function () {
            this.jsonInit({
                "type": "device_alarm",
                "message0": "Get device alarm %1 Severity %2 Search status %3",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "severity",
                        "check": "ZionAlarmSeverity"
                    },
                    {
                        "type": "input_value",
                        "name": "search_status",
                        "check": "ZionAlarmSeachStatus"
                    }
                ],
                "output": "Array",
                "colour": '#6665DD',
                "tooltip": "Get last telemetry from device",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['send_device_last_telemetry'] = {
        init: function () {
            this.jsonInit({
                "type": "send_device_last_telemetry",
                "message0": "Update telemetry %1 Key %2 Value %3",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "key",
                        "check": "String"
                    },
                    {
                        "type": "input_value",
                        "name": "payload",
                    }
                ],
                "output": "Boolean",
                "colour": '#6665DD',
                "tooltip": "Send telemetry to device",
                "helpUrl": ""
            });
        }
    };

    Blockly.Blocks['send_device_attr'] = {
        init: function () {
            this.jsonInit({
                "type": "send_device_last_telemetry",
                "message0": "Update attribute of device %1 Scope %2 Key %3 Value %4",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "scope",
                        "check": "ZionScope"
                    },
                    {
                        "type": "input_value",
                        "name": "key",
                        "check": "String"
                    },
                    {
                        "type": "input_value",
                        "name": "payload",
                    }
                ],
                "output": "Boolean",
                "colour": '#6665DD',
                "tooltip": "Send attribute to device",
                "helpUrl": ""
            });
        }
    };
    
    Blockly.Blocks['send_device_alarm'] = {
        init: function () {
            this.jsonInit({
                "type": "send_device_last_telemetry",
                "message0": "Create or update alarm on device %1 alarm %2",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "device",
                        "check": "ZionDevice"
                    },
                    {
                        "type": "input_value",
                        "name": "name",
                        "check": "String"
                    }
                ],
                "output": "Boolean",
                "colour": '#6665DD',
                "tooltip": "Create or update alarm on device",
                "helpUrl": ""
            });
        }
    };
} 

function loadRos2Blocks(ros2blocks){
    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            "type": "ros2_command",
            "message0": "ros2 run %1",
            "args0": [
                {
                    "type": "field_dropdown",
                    "name": "command",
                    "options": ros2blocks.commands
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "tooltip": "Start a ROS 2 process",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "ros2_subscribe",
            "message0": "On topic %1 type %2 get values in %3",
            "args0": [
                {
                    "type": "input_value",
                    "name": "topic",
                    "check": "String"
                },
                {
                    "type": "input_value",
                    "name": "message_type",
                    "check": "GenericRos2MessageType"
                },
                {
                    "type": "field_variable",
                    "name": "var_name",
                    "variable": "payload"
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "function"
                }
            ],
            "tooltip": "Subscribe to a ROS 2 topic and trigger the call on message event",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "ros2_message_type",
            "message0": "Type %1",
            "args0": [
                {
                "type": "field_dropdown",
                "name": "message_type",
                "options": ros2blocks.default_types
                }
            ],
            "output": "GenericRos2MessageType",
            "tooltip": "Select a ROS 2 message type",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "ros2_publish",
            "message0": "Publish message %1 on topic %2",
            "args0": [
                {
                    "type": "input_value",
                    "name": "message_type",
                    "check": "Ros2MessageType"
                },
                {
                    "type": "input_value",
                    "name": "topic",
                    "check": "String"
                }                
            ],
            "tooltip": "Publish a message to a ROS 2 topic",
            "helpUrl": "",
            "previousStatement": null,
            "nextStatement": null,
            "colour": 225
        },
        {
            "type": "ros2_message_String",
            "message0": "String(data=%1)",
            "args0": [
                {
                    "type": "input_value",
                    "name": "data",
                    "check": "String"
                }
            ],
            "output": "Ros2MessageType",
            "tooltip": "Create a ROS 2 String message",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "ros2_message_Bool",
            "message0": "Bool(data=%1)",
            "args0": [
                {
                    "type": "input_value",
                    "name": "data",
                    "check": "Boolean"
                }
            ],
            "output": "Ros2MessageType",
            "tooltip": "Create a ROS 2 Bool message",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "ros2_message_Int64",
            "message0": "Int64(data=%1)",
            "args0": [
                {
                    "type": "input_value",
                    "name": "data",
                    "check": "Number"
                }
            ],
            "output": "Ros2MessageType",
            "tooltip": "Create a ROS 2 Int64 message",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "ros2_message_Float64",
            "message0": "Float64(data=%1)",
            "args0": [
                {
                    "type": "input_value",
                    "name": "data",
                    "check": "Number"
                }
            ],
            "output": "Ros2MessageType",
            "tooltip": "Create a ROS 2 Float64 message",
            "helpUrl": "",
            "colour": 225
        },
    ]);

    Blockly.common.defineBlocks(blocksDefinitions);
}

function loadIronBoyBlocks(ironboy) {
    Blockly.Blocks['ironboy_command'] = {
        init: function () {
            this.jsonInit({
                "type": "ironboy_command",
                "tooltip": "Send Iron Boy command",
                "helpUrl": "Send a move command to Iron Boy",
                "message0": "Send command %1 execute for %2 times",
                "args0": [
                    {
                        "type": "input_value",
                        "name": "command",
                        "check": "IronBoyCommand"
                    },
                    {
                        "type": "input_value",
                        "name": "reps",
                        "check": "Number"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 30,
                "inputsInline": true
            });
        }
        
    }
    
    Blockly.Blocks['command_list'] = {
        init: function() {
            this.jsonInit({
            "type": "command_list",
            "message0": "Command %1",
            "args0": [
                {
                "type": "field_dropdown",
                "name": "command",
                "options": ironboy.commands
                }
            ],
            "output": "IronBoyCommand",
            "colour": "#EB6152",
            "tooltip": "Select an Iron Boy movement command",
            "helpUrl": ""
            });
        }
    };
}

function loadGinosAIBlocks(ginos){
    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            "type": "ginos_summarize_text",
            "tooltip": "",
            "helpUrl": "",
            "message0": "Summarize this file: %1",
            "args0": [
                {
                "type": "input_value",
                "name": "input_path",
                "check": "String"
                }
            ],
            "output": "String",
            "colour": "#EB6152",
            },
                    
        {
            "type": "ginos_ai_prompt",
            "message0": "AI prompt %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "prompt"
                }
            ],
            "output": "String",
            "colour": "#EB6152",
            "tooltip": "AI response from Ginos",
            "helpUrl": ""
        },
        {
            "type": "ginos_ai_chat_message_role",
            "message0": "AI role %1",
            "args0": [
                {
                    "type": "field_dropdown",
                    "name": "scope",
                    "options": ginos.roles
                }
            ],
            "output": "GinosAIChatRole",
            "colour": "#EB6152",
            "tooltip": "",
            "helpUrl": ""
        },
        {
            "type": "ginos_ai_chat",
            "message0": "AI chat message %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "messages",
                }
            ],
            "output": "String",
            "colour": "#EB6152",
            "tooltip": "",
            "helpUrl": ""
        },
        {
            "type": "ginos_ai_chat_message",
            "message0": "AI chat message %1 %2",
            "args0": [
                {
                    "type": "input_value",
                    "name": "role",
                    "check": "GinosAIChatRole"
                },
                {
                    "type": "input_value",
                    "name": "content",
                }
            ],
            "output": "GinosAIChatMessage",
            "colour": "#EB6152",
            "tooltip": "",
            "helpUrl": ""
        },
        {
            "type": "read_static_file",
            "tooltip": "read from file.csv",
            "helpUrl": "read from file.csv",
            "message0": "Input file : %1",
            "args0": [
                {
                    "type": "input_value",
                    "name": "path",
                    "check": "String"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 15,
            "inputsInline": false
        }
    ]);

    Blockly.common.defineBlocks(blocksDefinitions);
}

function loadMQTTBlocks(){
    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            "type": "mqtt_subscribe",
            "message0": "On message from %1 get values in %2",
            "args0": [
                {
                    "type": "input_value",
                    "name": "topic",
                    "check": "String"
                },
                {
                    "type": "field_variable",
                    "name": "var_name",
                    "variable": "payload"
                }
            ],
            "message1": "do %1",
            "args1": [
                {
                    "type": "input_statement",
                    "name": "function"
                }
            ],
            "tooltip": "Subscribe to a topic and trigger the call on message event",
            "helpUrl": "",
            "colour": 225
        },
        {
            "type": "mqtt_publish",

            "message0": "publish payload %1 on topic %2",
            "args0": [
                {
                    "type": "input_value",
                    "name": "payload"
                },
                {
                    "type": "input_value",
                    "name": "topic",
                    "check": "String"
                }
            ],
            "tooltip": "publish a message to a topic",
            "helpUrl": "",
            "previousStatement": null,
            "nextStatement": null,
            "colour": 225
        },
        {
            "type": "mqtt_register",
            "tooltip": "",
            "helpUrl": "",
            "message0": "Register the device",
            "args0": [],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 225
        },
        {
            "type": "mqtt_unregister",
            "tooltip": "",
            "helpUrl": "",
            "message0": "Unregister the device",
            "args0": [],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 225
        }

    ]);

    Blockly.common.defineBlocks(blocksDefinitions);

}

function defineImportsAndLibraries(){
    return `
# Shapes by Next Industries

import time
import random
import types
from numbers import Number
from datetime import datetime
from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue
from tactigon_shapes.modules.braccio.extension import BraccioInterface, CommandStatus, Wrist, Gripper
from tactigon_shapes.modules.zion.extension import ZionInterface, Scope, AlarmSearchStatus, AlarmSeverity
from tactigon_shapes.modules.ros2.extension import Ros2Interface
from tactigon_shapes.modules.ros2 import models as ros2_models
from tactigon_shapes.modules.tskin.models import TSkin, Gesture, Touch, OneFingerGesture, TwoFingerGesture
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
        logging_queue.debug(str(msg).replace("\\n","<br>"))

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

def ros2_run(ros2: Optional[Ros2Interface], command: str):
    if not ros2:
        return

    ros2.run(command)

def ros2_publish(ros2: Optional[Ros2Interface], topic: str, message: ros2_models.RosMessageTypes):
    if not ros2:
        return
    
    ros2.publish(topic, message)

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

`;
}

function defineCustomGenerators() {
    Blockly.Python.INDENT = '    ';

    python.pythonGenerator.forBlock['tactigon_shape_setup'] = function (block, generator) {
        var statements_body = Blockly.Python.statementToCode(block, 'setup_code');
        
        if (!statements_body) {
            statements_body = Blockly.Python.INDENT + "pass"
        }

        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0){
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        var code = 'def tactigon_shape_setup(\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'tskin: TSkin,\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'keyboard: KeyboardController,\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'braccio: Optional[BraccioInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'zion: Optional[ZionInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ros2: Optional[Ros2Interface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ironboy: Optional[IronBoyInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ginos: Optional[GinosInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'mqtt: Optional[MQTTClient],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'logging_queue: LoggingQueue):\n\n' +
            variables +
            statements_body;
        return code;
    };

    python.pythonGenerator.forBlock['tactigon_shape_close'] = function (block, generator) {
        var statements_body = Blockly.Python.statementToCode(block, 'setup_code');
        
        if (!statements_body) {
            statements_body = Blockly.Python.INDENT + "pass"
        }

        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0){
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        var code = 'def tactigon_shape_close(\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'tskin: TSkin,\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'keyboard: KeyboardController,\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'braccio: Optional[BraccioInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'zion: Optional[ZionInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ros2: Optional[Ros2Interface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ironboy: Optional[IronBoyInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ginos: Optional[GinosInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'mqtt: Optional[MQTTClient],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'logging_queue: LoggingQueue):\n\n' +
            variables +
            statements_body;
        return code;
    };

    python.pythonGenerator.forBlock['tactigon_shape_function'] = function (block, generator) {
        var statements_body = Blockly.Python.statementToCode(block, 'BODY');
        
        if (!statements_body) {
            statements_body = Blockly.Python.INDENT + "pass"
        }

        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0){
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        var code = 'def tactigon_shape_function(\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'tskin: TSkin,\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'keyboard: KeyboardController,\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'braccio: Optional[BraccioInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'zion: Optional[ZionInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ros2: Optional[Ros2Interface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ironboy: Optional[IronBoyInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'ginos: Optional[GinosInterface],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'mqtt: Optional[MQTTClient],\n' +
            Blockly.Python.INDENT + Blockly.Python.INDENT + 'logging_queue: LoggingQueue):\n\n' +
            variables +
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
    
    defineTSkinGenerators();
    defineSpeechGenerators();
    defineKeyboardGenerators();
    defineBraccioGenerators();
    defineDictionaryGenerators();
    defineZionGenerators();
    defineRos2Generators();
    defineIronBoyGenerators();
    defineGinosAIGenerators();
    defineMQTTGenerators();
}

function defineTSkinGenerators(){
    python.pythonGenerator.forBlock['tskin_gesture_list'] = function (block) {
        var gesture = block.getFieldValue('gesture');
        var code = `check_gesture(gesture, "${gesture}")`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_take_angle'] = function (block, generator) {
        var angle = block.getFieldValue('angle');
        var code = `tskin.angle.${angle} if tskin.angle else 0`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_take_gyro'] = function (block, generator) {
        var gyro = block.getFieldValue('gyro');
        var code = `tskin.gyro.${gyro} if tskin.gyro else 0`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['tskin_touch_list'] = function (block, generator) {
        var touchType = block.getFieldValue('touch');

        var code = `check_touch(touch, "${touchType}")`

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
        const mod1 = block.getFieldValue('mod_key_1');
        const mod2 = block.getFieldValue('mod_key_2');
        const func = block.getFieldValue('function_key');
        const code = `'${mod1}${mod2}${func}'`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineBraccioGenerators(){
    python.pythonGenerator.forBlock['braccio_move'] = function (block, generator) {
        const x = generator.valueToCode(block, 'x', python.Order.ATOMIC);
        const y = generator.valueToCode(block, 'y', python.Order.ATOMIC);
        const z = generator.valueToCode(block, 'z', python.Order.ATOMIC);
        const code = `braccio_move(braccio, logging_queue, ${x}, ${y}, ${z})\n`;
        return code;
    };

    python.pythonGenerator.forBlock['braccio_wrist'] = function (block, generator) {
        const x = block.getFieldValue('wrist');
        const code = `braccio_wrist(braccio, logging_queue, Wrist['${x}'])\n`;
        return code;
    };

    python.pythonGenerator.forBlock['braccio_gripper'] = function (block, generator) {
        const x = block.getFieldValue('gripper');
        const code = `braccio_gripper(braccio, logging_queue, Gripper['${x}'])\n`;
        return code;
    };
}

function defineDictionaryGenerators() {
    python.pythonGenerator.forBlock['get_dict_property'] = function (block, generator) {
        const dict = Blockly.Python.valueToCode(block, 'dictionary', Blockly.Python.ORDER_ATOMIC) || "{}";
        const key = Blockly.Python.valueToCode(block, 'key', Blockly.Python.ORDER_ATOMIC) || "''";

        const code = `${dict}.get(${key}, None)`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['dict_builder'] = function(block) {
        const pairBlock = block.getInputTargetBlock('PAIRS');
        const pairCode = Blockly.Python.blockToCode(pairBlock).slice(0, -2); // remove trailing comma and space
        const code = "{" + pairCode + "}";

        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['dict_pair'] = function (block) {
        const key = Blockly.Python.valueToCode(block, 'dict_key', Blockly.Python.ORDER_ATOMIC) || "'data'";
        const value = Blockly.Python.valueToCode(block, 'dict_value', Blockly.Python.ORDER_ATOMIC) || "''";

        const code = `${key}: ${value}, `;
        return code;
    };

    python.pythonGenerator.forBlock['dict_to_json'] = function (block, generator) {
        const dict = Blockly.Python.valueToCode(block, 'dict', Blockly.Python.ORDER_ATOMIC) || "{}";

        const code = `json.dumps(${dict})`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['json_to_dict'] = function (block, generator) {
        const dict = Blockly.Python.valueToCode(block, 'json', Blockly.Python.ORDER_ATOMIC) || "{}";

        const code = `json.loads(${dict})`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineZionGenerators() {
    python.pythonGenerator.forBlock["device_list"] = function (block) {
        var device = block.getFieldValue('device');
        var code = `"${device}"`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock["scope_list"] = function (block) {
        var scope = block.getFieldValue('scope');
        var code = `Scope("${scope}")`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock["alarm_severity_list"] = function (block) {
        var severity = block.getFieldValue('severity');
        var code = `AlarmSeverity("${severity}")`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock["alarm_search_status_list"] = function (block) {
        var search_status = block.getFieldValue('search_status');
        var code = `AlarmSearchStatus("${search_status}")`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['device_last_telemetry'] = function (block, generator) {
        var device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        var keys = generator.valueToCode(block, 'keys', python.Order.ATOMIC);

        var code = `zion_device_last_telemetry(zion, ${device}, ${keys})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['send_device_last_telemetry'] = function (block, generator) {
        const device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        const key = generator.valueToCode(block, 'key', python.Order.ATOMIC);
        const payload = generator.valueToCode(block, 'payload', python.Order.ATOMIC);

        const code = `zion_send_device_last_telemetry(zion, ${device}, ${key}, ${payload})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['send_device_attr'] = function (block, generator) {
        const device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        const scope = generator.valueToCode(block, 'scope', python.Order.ATOMIC);
        const key = generator.valueToCode(block, 'key', python.Order.ATOMIC);
        const payload = generator.valueToCode(block, 'payload', python.Order.ATOMIC);

        const code = `zion_send_device_attr(zion, ${device}, ${scope}, ${key}, ${payload})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };


    python.pythonGenerator.forBlock['device_attr'] = function (block, generator) {
        var device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        var scope = generator.valueToCode(block, 'scope', python.Order.ATOMIC);
        var keys = generator.valueToCode(block, 'keys', python.Order.ATOMIC);

        var code = `zion_device_attr(zion, ${device}, ${scope}, ${keys})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['device_alarm'] = function (block, generator) {
        var device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        var severity = generator.valueToCode(block, 'severity', python.Order.ATOMIC);
        var search_status = generator.valueToCode(block, 'search_status', python.Order.ATOMIC);

        var code = `zion_device_alarm(zion, ${device}, ${severity}, ${search_status})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };
    
    python.pythonGenerator.forBlock['send_device_alarm'] = function (block, generator) {
        var device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        var name = generator.valueToCode(block, 'name', python.Order.ATOMIC);

        var code = `zion_send_device_alarm(zion, ${device}, ${name})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['delete_device_attr'] = function (block, generator) {
        var device = generator.valueToCode(block, 'device', python.Order.ATOMIC);
        var scope = generator.valueToCode(block, 'scope', python.Order.ATOMIC);
        var key = generator.valueToCode(block, 'key', python.Order.ATOMIC);

        var code = `zion_delete_device_attr(zion,${device},${scope},${key})`

        return [code, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineRos2Generators(){
    python.pythonGenerator.forBlock['ros2_command'] = function(block, generator) {
        const command = block.getFieldValue('command');
        return `ros2_run(ros2, "${command}")\n`;
    };

    python.pythonGenerator.forBlock['ros2_subscribe'] = function(block, generator) {
        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0){
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);
        // const message_type = generator.valueToCode(block, 'message_type', python.Order.ATOMIC);
        const function_name = clean_topic_names(value_topic);
        const statement_function = generator.statementToCode(block, 'function');

        const code = `def ${function_name}(logging_queue: LoggingQueue):\n` + variables +  statement_function;
        return code;
    }

    python.pythonGenerator.forBlock['ros2_publish'] = function(block, generator) {
        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);
        const message_type = generator.valueToCode(block, 'message_type', python.Order.ATOMIC);

        const code = `ros2_publish(ros2, ${value_topic}, ${message_type})\n`
        return code;
    }

    python.pythonGenerator.forBlock['ros2_message_type'] = function(block) {
        const command = block.getFieldValue('message_type');
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_String'] = function(block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.String(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_Bool'] = function(block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.Bool(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_Int64'] = function(block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.Int64(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_Float64'] = function(block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.Float64(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineIronBoyGenerators(){
    python.pythonGenerator.forBlock['ironboy_command'] = function(block, generator) {
        const command = generator.valueToCode(block, 'command', python.Order.ATOMIC);
        const reps = generator.valueToCode(block, 'reps', python.Order.ATOMIC);
        const code = `iron_boy_command(ironboy, logging_queue, ${command}, ${reps})\n`;
        return code;
    };

    python.pythonGenerator.forBlock['command_list'] = function(block) {
        const command = block.getFieldValue('command');
        return [`IronBoyCommand.${command}`, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineGinosAIGenerators(){
    python.pythonGenerator.forBlock["ginos_ai_prompt"] = function(block, generator) {
        var prompt = generator.valueToCode(block, 'prompt', python.Order.ATOMIC);
        // var context = generator.valueToCode(block, 'context', python.Order.ATOMIC);
        // var code = `ginos_ai_prompt(ginos, ${prompt}, ${context})`;
        var code = `ginos_ai_prompt(ginos, ${prompt})`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    // python.pythonGenerator.forBlock["ginos_ai_chat"] = function(block, generator) {
    //     // var prompt = generator.valueToCode(block, 'prompt', python.Order.ATOMIC);
    //     // var context = generator.valueToCode(block, 'context', python.Order.ATOMIC);
    //     // var code = `ginos_ai_prompt(ginos, ${prompt}, ${context})`;
    //     var code = ``;
    //     return [code, Blockly.Python.ORDER_ATOMIC];
    // };

    // python.pythonGenerator.forBlock["ginos_ai_chat_message"] = function(block, generator) {
    //     var code = ``;
    //     return [code, Blockly.Python.ORDER_ATOMIC];
    // };

    // python.pythonGenerator.forBlock["ginos_ai_chat_message_role"] = function(block, generator) {
    //     var code = ``;
    //     return [code, Blockly.Python.ORDER_ATOMIC];
    // };
    
    python.pythonGenerator.forBlock['ginos_summarize_text'] = function(block, generator) {
    const text = generator.valueToCode(block, 'input_path', python.Order.ATOMIC);
        var code = `summarize_text(ginos,${text},logging_queue)`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    }

    // python.pythonGenerator.forBlock['read_static_file'] = function(block, generator) {
    //     const filepath = generator.valueToCode(block, 'path', python.Order.ATOMIC);
    //     var code = `extract_data(${filepath},logging_queue)`
    //     return code;
    // }
}

function defineMQTTGenerators(){
    python.pythonGenerator.forBlock['mqtt_subscribe'] = function(block, generator) {
        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0){
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);
        const function_name = clean_topic_names(value_topic);
        const statement_function = generator.statementToCode(block, 'function');

        const code = `def ${function_name}(logging_queue: LoggingQueue):\n` + variables +  statement_function;
        return code;
    }

    python.pythonGenerator.forBlock['mqtt_publish'] = function(block, generator) {
        const value_payload = generator.valueToCode(block, 'payload', python.Order.ATOMIC);
        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);

        const code = `mqtt_publish(mqtt, ${value_topic}, ${value_payload})\n`
        return code;
    }

    python.pythonGenerator.forBlock['mqtt_register'] = function(block, generator) {
        const code = `mqtt_register(mqtt)\n`
        return code;
    }

    python.pythonGenerator.forBlock['mqtt_unregister'] = function(block, generator) {
        const code = `mqtt_unregister(mqtt)\n`
        return code;
    }
}

function clean_topic_names(topic){
    return topic
        .replaceAll("/", "_")
        .replaceAll("\\", "_")
        .replaceAll(" ", "_")
        .replaceAll("'", "");
}