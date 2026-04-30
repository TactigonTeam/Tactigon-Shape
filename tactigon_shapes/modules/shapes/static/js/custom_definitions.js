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
    const ginos = response ? response.ginos : {};
    const file_manager = response ? response.file_manager : {};

    loadShapesBlocks();
    loadTSkinBlocks(gestures, taps);
    loadSpeechBlocks(speechs);
    loadKeyboardBlocks(funcKeys, modKeys);
    loadBraccioBlocks(wristOptions, gripperOptions);
    loadZionBlocks(zion);
    loadRos2Blocks(ros2);
    loadIronBoyBlocks(ironboy);
    loadGinosAIBlocks(ginos, file_manager);
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
        }
    ]);
    Blockly.common.defineBlocks(blocksDefinitions);
}

function loadShapesBlocks() {
    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            "type": "shapes_stop",
            "tooltip": "Stop shape execution",
            "helpUrl": "",
            "message0": "Stop shape %1",
            "args0": [
                {
                    "type": "input_dummy",
                    "name": "NAME"
                }
            ],
            "previousStatement": null,
            "colour": "#EB6152",
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
            "colour": "#7BAF1E",
            "tooltip": "Send a message to the terminal",
            "helpUrl": ""
        }
    ]);

    Blockly.common.defineBlocks(blocksDefinitions);
}

function loadDictionaryBlocks() {
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

    for (var i = 0; i < speechs.length; i++) {

        message += " %" + (i + 1);

        if (i == 0) {
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
        init: function () {
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

    // Blockly.Blocks['tskin_play'] = {
    //     init: function () {
    //         this.jsonInit({
    //             "type": "tskin_play",
    //             "message0": "Play file audio %1",
    //             "args0": [
    //                 {
    //                     "type": "input_value",
    //                     "name": "filename",
    //                     "check": "String"
    //                 }
    //             ],
    //             "previousStatement": null,
    //             "nextStatement": null,
    //             "colour": "#EB6152",
    //             "tooltip": "Use Tactigon Skin to play audio",
    //             "helpUrl": ""
    //         });
    //     }
    // };
}

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
                "colour": "#7F7F7F",
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
                "colour": "#7F7F7F",
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
                "colour": "#7F7F7F",
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
                "colour": "#7F7F7F",
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
                "colour": "#7F7F7F",
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
                "colour": "#7F7F7F",
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
                "colour": "#7F7F7F",
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

function loadZionBlocks(zion) {
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

function loadRos2Blocks(ros2blocks) {
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
        init: function () {
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

function loadGinosAIBlocks(ginos, file_manager) {
    let directory = [];
    let optionMapping = {};

    file_manager.forEach(el => {
        directory.push([el['directory']['name'], el['directory']['base_path']]);
        optionMapping[el['directory']['base_path']] = [['---', '']];

        optionMapping[el['directory']['base_path']].push(...el['content'].map(f => {
            const f_path = f['path'].replace(el['directory']['base_path'] + "/", '');
            return [f_path, f_path];
        }));
    });

    const blocksDefinitions = Blockly.common.createBlockDefinitionsFromJsonArray([
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
            "type": "ginos_load_dataframe",
            "message0": "Create dataframe from %1 %2",
            "args0": [
                {
                    "type": "field_dropdown",
                    "name": "directory",
                    "options": directory
                },
                {
                    "type": "field_dependent_dropdown",
                    "name": "filepath",
                    "parentName": "directory",
                    "optionMapping": optionMapping,
                    "defaultOptions": [['---', '']],
                }
            ],
            "output": "Boolean",
            "colour": "#EB6152",
            "tooltip": "Load dataframe locally",
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
    ]);

    Blockly.common.defineBlocks(blocksDefinitions);
}

function loadMQTTBlocks() {
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
