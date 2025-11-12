---

## Contributing
We welcome contributions from the community! To contribute:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description.

---

We appreciate your interest in Tactigon Shapes! If you encounter any issues or have feedback, feel free to [open an issue](https://github.com/TactigonTeam/Tactigon-Shape/issues).

## Creating Custom Blocks

We'll walk you through how to create the block and handle the Python code generation. The easiest step is to create your custom blocks on the Blockly Developer website:  [Blockly Developer Tools](https://developers.google.com/blockly/guides/create-custom-blocks/blockly-developer-tools). This platform provides an intuitive interface for defining block shapes, fields, and behavior without needing to write code manually. You can customize the block's appearance, input types, and logic connections. Once your block is designed, the tool generates the corresponding JSON, which can be easily integrated into Tactigon Shapes project.

### Steps to Create Custom Blocks:

1. **Create a Category (Optional)**  
   - If you want to organize your blocks, you can create a new category or assign your block to an existing category.  
   - To create a new category, go to the [Edit](https://github.com/TactigonTeam/Tactigon-Shape/blob/master/tactigon_shapes/modules/shapes/templates/shapes/edit.jinja) page in the **Shape** module.

```Html
  <category name="To Uppercase" colour="#f1c40f ">
      <block type="to_uppercase">
      </block>
  </category>
```

2. **Design Your Block**  
   - Use the Blockly Developer Tools to create your new block by customizing its inputs, logic, and appearance.  
   - Once done, copy the generated JSON code for your block.
```javascript
 Blockly.Blocks['to_uppercase'] = {
        init: function () {
            this.jsonInit({
                "type": "to_uppercase",
                "tooltip": "",
                "helpUrl": "",
                "message0": "To Uppercase %1",
                "args0": [
                  {
                    "type": "input_value",
                    "name": "TEXT",
                    "check": "String"
                  }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": '#f1c40f'
              });
        }
};
```

3. **Add the Block to Tactigon Shapes**  
   - Paste the JSON code into the [Custom blocks file](https://github.com/TactigonTeam/Tactigon-Shape/blob/master/tactigon_shapes/modules/shapes/static/js/custom_blocks.js) page in the **Shapes** module.  
   - After adding it, you will see both the new category (if created) and the new block in the Tactigon Shapes workspace.

![chrome_innQ8Zb8cg](https://github.com/user-attachments/assets/f993507d-b862-45f8-9bb3-c88f60cf46b7)

4. **Write Custom Code for the Block**  
   - You can write custom Python code for your block in the [Custom blocks file](https://github.com/TactigonTeam/Tactigon-Shape/blob/master/tactigon_shapes/modules/shapes/static/js/custom_blocks.js) page in the **Shapes** module.
   - From Tactigon Shapes, you can generate code in JavaScript, Python, PHP, or Dart based on the visual blocks. But we recommend using Python because almost all of our projects speak Python. 
```python
    python.pythonGenerator.forBlock['to_uppercase'] = function(block) {
        var text_to_print = Blockly.Python.valueToCode(block, 'TEXT', Blockly.Python.ORDER_ATOMIC) || "''";
        var code = `debug(logging_queue, ${text_to_print}.upper())\n`;
        return code;
    };  
```

5. **Run and Test**  
   - After creating your block, rerun the project and test your block by running your shape.

![chrome_32A8gvnK5J](https://github.com/user-attachments/assets/3af28103-b8f7-4981-96a6-72fb9a97ff3a)

Here, we have attached a demo video about how to create your own block in the Tactigon Shapes project.

https://github.com/user-attachments/assets/d420dd2d-30e9-48e7-b07b-90c5ae0e3bd6

# Creating a Custom Module in Tactigon Shape

This guide will walk you through how to **add your own module** to the Tactigon Shape system.

---

## Project Structure Overview

Here's what each main folder does:

### `config`
Stores all config files for the app and its modules.

### `Models`
Holds trained machine learning models and related info.

### `speech`
Includes speech recognition models and files.

### `tactigon_speech`
The main Python package where everything connects.

### Main Python File
This file is where everything starts. It handles command-line options and runs the Tactigon server.

![image](https://github.com/user-attachments/assets/42c5637b-7e1a-4b11-9490-8269c6a5b297)

---

## Module Structure

Inside the project folder `tactigon_shapes`, you’ll find a folder named `modules`. This is where all modules live.

Each module should have:

![image](https://github.com/user-attachments/assets/769ffe25-9a5d-4342-97c4-9503f34bfa16)

- `blueprint.py`: Flask routes and web APIs.
- `extension.py`: Connects your module to other systems/devices.
- `manager.py`: Business logic and how your module works.
- `models.py`: Data structures for your module.
- `templates/`, `static/`: For web UI (Jinja HTML, CSS, JS).

---

## How to Create a Module

1. Inside `modules/`, create a new folder with your module’s name.

![image](https://github.com/user-attachments/assets/a9efc003-9583-40f5-b82a-ec8b30fe0ca3)

2. Inside that folder, add these files (empty or with templates):

- `__init__.py`  
- `blueprint.py`  
- `extension.py`  
- `manager.py`  
- `models.py`  
- `templates/` and `static/` folders if needed  

3. If you're building a web page, put your Jinja templates in the `templates/` folder.

![image](https://github.com/user-attachments/assets/9bbebbab-dc01-4dcc-aaa0-3776c2976dbf)

4. In `models.py`, define your module's data (like configs or status).

![image](https://github.com/user-attachments/assets/51371194-5dba-4c12-8737-4d13ca3b189c)

5. In `manager.py`, write your core logic (how it behaves or reacts).

![image](https://github.com/user-attachments/assets/62e5d3eb-29c6-46b1-9acc-b14d315d1d22)

6. In `extension.py`, connect to external devices or other modules.

![image](https://github.com/user-attachments/assets/54c36264-1f20-49ad-85c6-9d92e2ed01d8)

7. In `blueprint.py`, create Flask routes and link your Jinja UI files.

![image](https://github.com/user-attachments/assets/d5eb019d-2f06-4a55-b76a-425215bb8c8e)

Example:  
When users go to `/zion`, they'll see `index.jinja` from the `zion` module.

![image](https://github.com/user-attachments/assets/d1ae68e7-0643-4951-a9c5-296d84e2cf17)

This is what that page would look like:

![image](https://github.com/user-attachments/assets/a5b0b543-31cb-4879-8512-b96c04939454)

 Want to create **custom blocks**? Use [this link](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Contributing#creating-custom-blocks)

![image](https://github.com/user-attachments/assets/b134b3d4-da85-4f7e-bddb-3787a8db7744)

8. Import your module’s `extension` and `manager` where needed.

![image](https://github.com/user-attachments/assets/6f23fe94-70dc-436f-bd4d-d3a8c8c3bf89)

9. Register your module in `tactigon_shapes/__init__.py`.

![image](https://github.com/user-attachments/assets/f6fb7531-de82-4473-89e7-90192cb69baf)

10. Register the blueprint in the same file.

![image](https://github.com/user-attachments/assets/4fcf9723-a6fa-451a-9e1d-cdab9b4ecf06)

11. Add your module to the navigation bar (optional, but useful!).

![image](https://github.com/user-attachments/assets/90540388-0af3-44e3-81f4-6531e7cbf810)

To do this, update `tactigon_shapes/templates/base.jinja` to include your link and icon:

![image](https://github.com/user-attachments/assets/534bfb81-14f0-43ca-9aae-49574021b992)

12. Need real-time updates from server to browser?

Use `SocketApp` from `tactigon_shapes/modules/socketio/extension.py`. It pushes live data from your backend to the UI.

![image](https://github.com/user-attachments/assets/eb5a2429-f62b-4796-9b03-947dfe315f77)

---

That’s it! Now you know how to build a new module in the Tactigon Shapes system 🎉  