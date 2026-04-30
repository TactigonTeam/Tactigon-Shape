
function defineShapesGenerators() {
    python.pythonGenerator.forBlock['tactigon_shape_debug'] = function (block, generator) {
        var message = generator.valueToCode(block, 'TEXT', python.Order.ATOMIC);
        var code = `debug(logging_queue, ${message})\n`;
        return code;
    };

    python.pythonGenerator.forBlock['shapes_stop'] = function () {
        return "return False";
    };
}

function defineTSkinGenerators() {
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

function defineSpeechGenerators() {
    python.pythonGenerator.forBlock['tskin_listen'] = function (block) {
        let args = block.inputList[0].fieldRow
            .filter((f) => f.selectedOption && f.selectedOption[1] != "")
            .map((f) => {
                if (f.selectedOption[1] == "---") {
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

    // python.pythonGenerator.forBlock['tskin_play'] = function (block, generator) {
    //     let filename = generator.valueToCode(block, 'filename', python.Order.ATOMIC);

    //     return `tskin.play(${filename})\n`
    // };
}

function defineKeyboardGenerators() {
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

function defineBraccioGenerators() {
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

    python.pythonGenerator.forBlock['dict_builder'] = function (block) {
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

function defineRos2Generators() {
    python.pythonGenerator.forBlock['ros2_command'] = function (block, generator) {
        const command = block.getFieldValue('command');
        return `ros2_run(ros2, "${command}")\n`;
    };

    python.pythonGenerator.forBlock['ros2_subscribe'] = function (block, generator) {
        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0) {
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);
        // const message_type = generator.valueToCode(block, 'message_type', python.Order.ATOMIC);
        const function_name = clean_topic_names(value_topic);
        const statement_function = generator.statementToCode(block, 'function');

        const code = `def ${function_name}(logging_queue: LoggingQueue):\n` + variables + statement_function;
        return code;
    }

    python.pythonGenerator.forBlock['ros2_publish'] = function (block, generator) {
        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);
        const message_type = generator.valueToCode(block, 'message_type', python.Order.ATOMIC);

        const code = `ros2_publish(ros2, ${value_topic}, ${message_type})\n`
        return code;
    }

    python.pythonGenerator.forBlock['ros2_message_type'] = function (block) {
        const command = block.getFieldValue('message_type');
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_String'] = function (block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.String(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_Bool'] = function (block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.Bool(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_Int64'] = function (block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.Int64(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock['ros2_message_Float64'] = function (block, generator) {
        const data = generator.valueToCode(block, 'data', python.Order.ATOMIC);
        const command = `ros2_models.Float64(data=${data})`;
        return [command, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineIronBoyGenerators() {
    python.pythonGenerator.forBlock['ironboy_command'] = function (block, generator) {
        const command = generator.valueToCode(block, 'command', python.Order.ATOMIC);
        const reps = generator.valueToCode(block, 'reps', python.Order.ATOMIC);
        const code = `iron_boy_command(ironboy, logging_queue, ${command}, ${reps})\n`;
        return code;
    };

    python.pythonGenerator.forBlock['command_list'] = function (block) {
        const command = block.getFieldValue('command');
        return [`IronBoyCommand.${command}`, Blockly.Python.ORDER_ATOMIC];
    };
}

function defineGinosAIGenerators() {
    python.pythonGenerator.forBlock["ginos_ai_prompt"] = function (block, generator) {
        var prompt = generator.valueToCode(block, 'prompt', python.Order.ATOMIC);
        // var context = generator.valueToCode(block, 'context', python.Order.ATOMIC);
        // var code = `ginos_ai_prompt(ginos, ${prompt}, ${context})`;
        var code = `ginos_ai_prompt(ginos, ${prompt})`;
        return [code, Blockly.Python.ORDER_ATOMIC];
    };

    python.pythonGenerator.forBlock["ginos_load_dataframe"] = function (block, generator) {
        const dir = block.getFieldValue('directory');
        const fpath = block.getFieldValue('filepath');
        return [`ginos_load_dataframe(ginos, "${dir}", "${fpath}")`, python.Order.ATOMIC];
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

    // python.pythonGenerator.forBlock['ginos_summarize_text'] = function (block, generator) {
    //     const text = generator.valueToCode(block, 'text', python.Order.ATOMIC);
    //     var code = `summarize_text(ginos,${text},logging_queue)`;
    //     return [code, Blockly.Python.ORDER_ATOMIC];
    // }
}

function defineMQTTGenerators() {
    python.pythonGenerator.forBlock['mqtt_subscribe'] = function (block, generator) {
        let variables = block.workspace.getAllVariables().map((v) => {
            return v.name;
        }).join(', ');

        if (variables.length > 0) {
            variables = `${Blockly.Python.INDENT}global ${variables}\n`;
        }

        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);
        const function_name = clean_topic_names(value_topic);
        const statement_function = generator.statementToCode(block, 'function');

        const code = `def ${function_name}(logging_queue: LoggingQueue):\n` + variables + statement_function;
        return code;
    }

    python.pythonGenerator.forBlock['mqtt_publish'] = function (block, generator) {
        const value_payload = generator.valueToCode(block, 'payload', python.Order.ATOMIC);
        const value_topic = generator.valueToCode(block, 'topic', python.Order.ATOMIC);

        const code = `mqtt_publish(mqtt, ${value_topic}, ${value_payload})\n`
        return code;
    }

    python.pythonGenerator.forBlock['mqtt_register'] = function (block, generator) {
        const code = `mqtt_register(mqtt)\n`
        return code;
    }

    python.pythonGenerator.forBlock['mqtt_unregister'] = function (block, generator) {
        const code = `mqtt_unregister(mqtt)\n`
        return code;
    }
}


function clean_topic_names(topic) {
    return topic
        .replaceAll("/", "_")
        .replaceAll("\\", "_")
        .replaceAll(" ", "_")
        .replaceAll("'", "");
}