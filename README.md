# Tactigon Shapes

Tactigon Shapes is an extension of [Blockly](https://developers.google.com/blockly) which is a project by Google and it allows you to create visual, drag-and-drop block-based programming interfaces. Instead of typing code, you can create programs by connecting blocks together. This README provides an overview of the project, its features, installation instructions, and guidance on contributing.

## Table of Contents
1. [Introduction](#introduction)
2. [Key Features](#key-features)
3. [Tactigon Ecosystem](#tactigon-ecosystem)
4. [Installation](#installation)
5. [Running Tactigon Shapes](#running-tactigon-shapes)
6. [Using Tactigon Shapes](#using-tactigon-shapes)
7. [Creating Custom Blocks](#creating-custom-blocks)
8. [Contributing](#contributing)

## Introduction
Tactigon Shapes simplifies programming by allowing you to build logic visually instead of writing code. This project is ideal for beginners, educators, and developers who want to create fast prototypes using gesture recognition and voice commands using Tactigon Ecosystem.

## Key Features:
- Visual Programming: Drag-and-drop interface that helps users understand programming concepts.
- Code Generation: Automatically generates code in multiple programming languages.
- Customizable: You can create your own blocks and define their behavior.
  
## Tactigon Ecosystem

Tactigon ecosystem is consists of:
 - [Tactigon Gear](https://pypi.org/project/tactigon-gear/) to connect to a Tactigon Skin wearable device and do gesture recognition
 - [Tactigon Speech](https://pypi.org/project/tactigon-speech/) to implement voice recognition on top of Tactigon Gear
 - [Tactigon Arduino Braccio](https://pypi.org/project/tactigon-arduino-braccio/) to connect to the Arduino Braccio device

## Installation
### Prerequisites

In order to use the Tactigon SDK the following prerequisites need to be observed:

- **Mac/Linux:**
To set the required tools on your system, follow the steps below:

  - CPU with AVX/FMA support
    - To check if your Mac supports **AVX/FMA** you can go to **About This Mac**. Check the CPU model and verify its features online.
  - Python 3.8.10 [Download Python 3.8.10](https://www.python.org/downloads/release/python-3810/)

- **Windows:**
To set up the required tools on your Windows 10 or Windows 11 operating system, follow the steps below:
  - CPU with AVX/FMA support
    - To check if your CPU supports AVX/FMA you can press **Win + R** then type **msinfo32** and press Enter. Check the CPU model and verify its features online.
  - Python 3.8.10 [Download Python 3.8.10](https://www.python.org/downloads/release/python-3810/)
  - Microsoft C++ Build Tools and Windows 10/11 SDK. 

- **Microsoft C++ Build Tools and Windows 10/11 SDK installation:**
Open the Visual Studio Installer on your system. if you do not have one, [Download Visual Studio Installer](https://visualstudio.microsoft.com/downloads/)
  - From the Visual Studio Installer, click Modify or select Install for new installation.
  - Go to the individual components tab and install the latest version 
    - MSVC v143 - VS 2022 C++ Build Tools x64/x86 (latest version)
    - Windows 10/11 SDK (latest version matching your Windows version)

![Screenshot 2024-11-21 162339](https://github.com/user-attachments/assets/5f6332f1-be2b-4fee-ad62-7feb734db710)

### Steps to Install:
1. Download the repository:
![Immagine 2025-02-12 101837](https://github.com/user-attachments/assets/4065de26-cb74-453f-9c3f-32497c698408)
   
2. Pick a location and extract all the files. you will see a folder named **Tactigon-Shape-master**.

2. Navigate to the project directory or open the project with your default code editor or IDE:
   ```bash
   cd Tactigon-Shape-master
   ```
3. Activate the virtual environment and Install dependencies:

   We have created an install file (install.bat - windows / install.sh - Linux, macOS) to configure everything for you. Please execute that file; you will see the following output on your terminal.

![image](https://github.com/user-attachments/assets/e2b37b37-2f24-44e6-bab1-22191465d591)

<br>
<br>

## Running Tactigon Shapes 
1. Run the main Python script:
   ```bash
   python .\main.py
   ```
2. Open your web browser and go to:
   ```
   http://127.0.0.1:5123
   ```
3. Follow the prompts to connect to your Tactigon Skin device and start using the Shapes interface.

![chrome_GBrADG4C6r](https://github.com/user-attachments/assets/30107a15-e4ca-424d-80c2-bca73e3c0370)
---

## Using Tactigon Shapes

Using Shapes is like building with LEGO bricks, you drag, drop, and connect pieces to create something functional. This section guides you step-by-step on how to use shapes.

Once you are on the Shape interface, you can see some of our example shapes, such as Powerpoint, Braccio voice, and so on. On the left side, you will see all your shapes, and on the right side, you can see the snapshot of your shape.

![chrome_0Pi7DpjX6l](https://github.com/user-attachments/assets/0bb63989-c89a-4e04-9067-00f585e86e09)


## How to use Shapes


### Creating a New Shape:
1. Click **Add Shape**.
2. Enter a unique shape name and description.
3. Add the shape to open the editing workspace.
4. Drag blocks from the left pane to the workspace to build your logic.
5. Click **Save** to save your shape.

https://github.com/user-attachments/assets/236e746c-a9f9-4f51-833e-472dd02bc227


### Editing an Existing Shape:
1. Select a shape from the homepage.
2. Click **Edit Code** to modify the shape.
3. Save your changes after editing.

Think of blocks like puzzle pieces. This means you can now connect them by dragging one block and placing it under or inside another block. However, ensure that your shapes snap together; otherwise, the program will not execute.

https://github.com/user-attachments/assets/f00c5e2a-0cc2-4bfc-b50b-51d4f147f755

### Run Your First Shape
- When you are done building, click the toggle button to see what your program does.
- You will see a page with both your shape and the terminal with the output.

https://github.com/user-attachments/assets/a1450158-5b35-4e3e-aeb9-6cf5b2f47f26

### Deleting a Shape:
- Click the **bin** icon next to the shape name to delete it.
  
![image](https://github.com/user-attachments/assets/7105f02f-e345-4081-84d2-3ff5dff262bc)


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
   - From Tactigon Shapes you can generate code in JavaScript, Python, PHP, or Dart based on the visual blocks. But we recommend using Python because almost all of our projects speak Python. 
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

Here we have attached a demo video about how to create your own block in the Tactigon Shapes project.

https://github.com/user-attachments/assets/d420dd2d-30e9-48e7-b07b-90c5ae0e3bd6

---

## Contributing
We welcome contributions from the community! To contribute:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description.

---

We appreciate your interest in Tactigon Shapes! If you encounter any issues or have feedback, feel free to [open an issue](https://github.com/TactigonTeam/Tactigon-Shape/issues).
