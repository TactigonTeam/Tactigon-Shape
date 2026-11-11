# Contributing to Tactigon SHAPE

Thank you for your interest in contributing to **Tactigon SHAPE**!

SHAPE is a visual programming environment based on Google Blockly, designed to connect robotics, IoT devices and software services. It allows users to create applications using visual blocks that interact with ROS 2, MQTT, REST APIs, local AI models and other systems.

We welcome contributions from the community, including bug fixes, new features, custom Blockly blocks, new modules, documentation improvements and examples.

This guide explains how to configure your development environment, understand the SHAPE architecture, create custom modules and blocks, and submit your contributions.

---

# Table of Contents

- [1. Getting Started](#1-getting-started)
- [2. Development Environment](#2-development-environment)
- [3. Understanding SHAPE Architecture](#3-understanding-shape-architecture)
- [4. Creating a Custom Module](#4-creating-a-custom-module)
- [5. Creating Custom Blockly Blocks](#5-creating-custom-blockly-blocks)
- [6. Adding Blocks to an Existing Module](#6-adding-blocks-to-an-existing-module)
- [7. Testing Your Changes](#7-testing-your-changes)
- [8. Submitting a Pull Request](#8-submitting-a-pull-request)
- [9. Reporting Issues](#9-reporting-issues)

---

# 1. Getting Started

Before contributing, make sure you have:

- A GitHub account.
- Git installed.
- Docker installed and running.
- Docker Compose plugin installed.
- A Linux development environment with an active graphical session.

## Fork and clone the repository

Fork the repository:

https://github.com/TactigonTeam/Tactigon-Shape

Clone your fork:

```bash
git clone https://github.com/<YOUR_USERNAME>/Tactigon-Shape.git
cd Tactigon-Shape
```

Add the original repository as an upstream remote:

```bash
git remote add upstream https://github.com/TactigonTeam/Tactigon-Shape.git
```

Create a dedicated branch for your contribution:

```bash
git checkout -b feature/my-new-feature
```

Use descriptive branch names:

- `feature/custom-mqtt-block`
- `feature/new-module`
- `fix/ros2-connection`
- `docs/update-installation-guide`

---

# 2. Development Environment

SHAPE runs inside a Docker container, which provides the Python environment, libraries and configurations required by the application.

By default, Docker Compose uses a pre-built image from Docker Hub:

`nextdeveloper/tactigon-shapes`

This is convenient for end users, but contributors need Docker to build the image from their local source code. Otherwise, changes made to the repository will not be visible inside the running application.

## 2.1 Configure Docker Compose

Open `docker-compose.yml` and locate the `tactigon-shapes` service.

Replace the pre-built image configuration with a local build configuration:

```yaml
tactigon-shapes:
    # image: nextdeveloper/tactigon-shapes
    build:
      context: .
      dockerfile: Dockerfile
```

## 2.2 Enable Local Image Rebuilding

Open `start.sh` and ensure the `docker compose up` command includes the `--build` option.

For example:

```bash
docker compose up -d --build
```

This forces Docker to rebuild the application image using your local source code.

> **Important:** These changes are only intended for local development. Do not include them in your final Pull Request, as they would modify the standard configuration used by other users.

## 2.3 Start SHAPE

On Linux, make sure the graphical session allows Docker to access the display.

The repository provides an `.xprofile` configuration for this purpose.

Append it to your user profile if required:

```bash
cat .xprofile >> ~/.xprofile
```

Then start the application:

```bash
bash start.sh
```

Once the application is running, open your browser:

http://localhost:5123

---

# 3. Understanding SHAPE Architecture

Before creating custom blocks, it is important to understand how SHAPE handles the execution of visual programs.

SHAPE is based on Blockly. Users create programs by connecting visual blocks in the editor. Blockly then translates those blocks into Python code, which is executed by the SHAPE backend.

The implementation is divided into three main components:

1. **Python backend:** implements the actual functionality of the blocks.
2. **Blockly definitions and generators:** define the visual blocks and translate them into Python code.
3. **Toolbox configuration:** determines how blocks are displayed and organized in the visual editor.

## 3.1 General Architecture

The following diagram illustrates the relationship between the main components involved in custom block development.

![SHAPE module and Blockly architecture](https://github.com/user-attachments/assets/049f5284-874e-41f9-980e-5672709c76d2)

### Python backend

The Python logic is located in:

`tactigon_shapes/modules/<myModule>/`

Each module can contain the following files:

| File | Responsibility |
|---|---|
| `extension.py` | Handles the module's Python functionality and integrations |
| `manager.py` | Implements the core business logic |
| `blueprint.py` | Defines Flask routes and connects them to Jinja templates |
| `models.py` | Defines module data, configurations and status |
| `__init__.py` | Handles module initialization |

### Blockly definitions and generators

The block definitions are managed in:

`tactigon_shapes/modules/shapes/static/js/custom_blocks.js`

The main functions are:

- `load<MyModule>Block()`: loads the JSON definitions of a module's blocks.
- `defineImportsAndLibraries()`: generates the Python header containing imports and helper functions.
- `define<MyModule>Generators()`: registers the generators that translate Blockly blocks into Python code.
- `loadCustomBlocks()`: invokes the functions responsible for loading the available block groups.

### Toolbox rendering

The toolbox configuration is managed by the Shapes module's Jinja template:

`tactigon_shapes/modules/shapes/templates/shapes/edit.jinja`

The toolbox is organized into categories, each containing one or more blocks.

Example:

```xml
<category name="CategoryName" colour="#ff00f7">
    <block type="block_name_1"></block>
    <block type="block_name_2"></block>
</category>
```

A block must be registered in Blockly and included in the toolbox before users can select it in the editor.

## 3.2 Choosing Your Development Approach

There are two ways to extend SHAPE:

**Approach A — Create a new module and its blocks**

This is the recommended approach when introducing a new integration or functionality that deserves its own backend implementation.

It involves creating the module, implementing its Python logic, registering it in SHAPE and integrating its Blockly blocks.

**Approach B — Add blocks to an existing module**

This is a simpler approach when the functionality belongs to an existing module.

You only need to implement the required Python functions and register the corresponding Blockly definitions and generators.

Both approaches are described below.

---

# 4. Creating a Custom Module

This section explains how to create a new module and integrate it into the SHAPE application.

## 4.1 Project Structure Overview

The main repository contains several directories:

| Directory | Description |
|---|---|
| `config/` | Application and module configuration files |
| `models/` | Machine learning models and related information |
| `speech/` | Speech recognition models and files |
| `tactigon_shapes/` | Main Python application package |

The main application entry point is `main.py`, which handles application startup and launches the SHAPE server.

![SHAPE project structure](https://github.com/user-attachments/assets/42c5637b-7e1a-4b11-9490-8269c6a5b297)

## 4.2 Module Structure

Inside `tactigon_shapes/`, all modules are organized under:

`tactigon_shapes/modules/`

![SHAPE modules directory](https://github.com/user-attachments/assets/769ffe25-9a5d-4342-97c4-9503f34bfa16)

Each module should follow this general structure:

```text
my_module/
├── __init__.py
├── blueprint.py
├── extension.py
├── manager.py
├── models.py
├── templates/
└── static/
```

The `templates/` and `static/` directories are optional and should be included when the module provides a web interface.

An example of an existing module:

![Example module directory](https://github.com/user-attachments/assets/a9efc003-9583-40f5-b82a-ec8b30fe0ca3)

## 4.3 Create Your Module

Create a new directory inside `tactigon_shapes/modules/`:

```bash
mkdir tactigon_shapes/modules/my_module
```

Add the following files:

```text
__init__.py
blueprint.py
extension.py
manager.py
models.py
```

If your module provides a web interface, also create:

```text
templates/
static/
```

For example, the Zion module contains a dedicated Jinja template:

![Example module templates](https://github.com/user-attachments/assets/9bbebbab-dc01-4dcc-aaa0-3776c2976dbf)

## 4.4 Implement the Module Files

Each file has a specific responsibility.

### models.py — Data Structures

Define the data structures used by your module, including configuration objects, status information and other relevant models.

Example:

![Example models.py implementation](https://github.com/user-attachments/assets/51371194-5dba-4c12-8737-4d13ca3b189c)

### manager.py — Core Logic

Implement the module's core business logic.

This file can expose helper functions used by the extension or other parts of the application.

Example:

![Example manager.py implementation](https://github.com/user-attachments/assets/62e5d3eb-29c6-46b1-9acc-b14d315d1d22)

### extension.py — Integrations

Implement the module's integration with external devices, services and other SHAPE components.

This is where you can expose functionality that will be used by custom Blockly blocks.

Example:

![Example extension.py implementation](https://github.com/user-attachments/assets/54c36264-1f20-49ad-85c6-9d92e2ed01d8)

### blueprint.py — Web Interface

If your module needs a web interface, use Flask Blueprints to define its routes and connect them to Jinja templates.

Example:

![Example blueprint.py implementation](https://github.com/user-attachments/assets/d5eb019d-2f06-4a55-b76a-425215bb8c8e)

For example, a module named `zion` can expose the `/zion` route and render its `index.jinja` template.

The original implementation is illustrated below:

![Zion module implementation in VS Code](https://github.com/user-attachments/assets/d1ae68e7-0643-4951-a9c5-296d84e2cf17)

The resulting web interface:

![Zion module web interface](https://github.com/user-attachments/assets/a5b0b543-31cb-4879-8512-b96c04939454)

## 4.5 Register Your Module

Once the module has been implemented, it must be registered in the main application.

Open:

`tactigon_shapes/__init__.py`

### Step 1 — Import the module

Import the module's extension and manager where required.

Example:

![Module imports](https://github.com/user-attachments/assets/b134b3d4-da85-4f7e-bddb-3787a8db7744)

### Step 2 — Initialize the module

Create the module instance and initialize it with the Flask application.

Example:

![Module registration and initialization](https://github.com/user-attachments/assets/f6fb7531-de82-4473-89e7-90192cb69baf)

### Step 3 — Register the Flask Blueprint

Import the module's Blueprint and register it with the Flask application.

Example:

![Blueprint registration](https://github.com/user-attachments/assets/4fcf9723-a6fa-451a-9e1d-cdab9b4ecf06)

### Step 4 — Add the module to the navigation bar (optional)

If your module provides a web interface, you can add an entry to the main navigation bar.

Open:

`tactigon_shapes/templates/base.jinja`

Add the appropriate link and icon.

Example:

![Navigation bar integration](https://github.com/user-attachments/assets/90540388-0af3-44e3-81f4-6531e7cbf810)

The following screenshot illustrates the corresponding template configuration:

![Navigation bar code](https://github.com/user-attachments/assets/534bfb81-14f0-43ca-9aae-49574021b992)

## 4.6 Real-Time Communication with SocketApp

If your module needs to exchange real-time data between the backend and the browser, SHAPE provides the `SocketApp` implementation.

It is located in:

`tactigon_shapes/modules/socketio/extension.py`

`SocketApp` provides the infrastructure for sending live updates from the backend to the user interface.

You can integrate your module with the existing SocketApp implementation instead of creating an independent communication layer.

Example:

![SocketApp integration example](https://github.com/user-attachments/assets/eb5a2429-f62b-4796-9b03-947dfe315f77)

---

# 5. Creating Custom Blockly Blocks

SHAPE uses Google Blockly, an open-source visual programming library.

Creating a custom block involves three main components:

1. **Block definition:** describes the block's visual appearance, inputs, fields and connections.
2. **Python helper function:** implements the actual functionality.
3. **Python code generator:** translates the visual block into executable Python code.

The following sections explain the entire process.

## 5.1 Blockly Developer Tools

Blockly provides an online editor called Blockly Developer Tools:

https://blockly-demo.appspot.com/static/demos/blockfactory/index.html

It allows you to design blocks visually, configure their properties and generate the JSON definition and code generator template.

![Blockly Developer Tools](https://developers.google.com/static/blockly/images/blockfactory.png)

### Name

The internal identifier of the block.

It is used by the code to recognize and register the block. Users do not normally see this identifier.

For example:

`to_uppercase`

### Inputs

Inputs define the slots where other blocks or values can be connected.

There are three main types:

| Input type | Description |
|---|---|
| Value | Accepts another block that returns a value, such as a number or string |
| Statement | Accepts a sequence of instruction blocks |
| Dummy | Adds visual spacing or a new line without accepting another block |

Each input has a unique internal name, used by the generator to retrieve its value.

For example, a square operation might use the input name `number_to_be_squared`.

The visible label is a separate field. The label is what users see in the editor, while the input name is used internally by the code.

### Fields

Fields are the visual and interactive elements inside a block.

Available field types include:

| Field type | Description |
|---|---|
| Label | Static text |
| Text input | Editable text |
| Numeric input | Numeric value with optional minimum, maximum and precision |
| Dropdown | Menu with predefined options |
| Checkbox | Boolean toggle |
| Variable | Variable selector |
| Image | Displays an image |
| Angle | Angle selector returning degrees |

For example, a dropdown can define an operation:

```javascript
[
    ["+", "PLUS"],
    ["-", "MINUS"]
]
```

The first value is displayed to the user, while the second is returned to the generator.

### Connection Checks

Connection checks define which data types can be connected to an input or returned by a block.

Common types include:

- `Boolean`
- `Number`
- `String`
- `Array`
- `null` or unrestricted connections
- Custom type identifiers

For example, setting an input check to `Number` prevents users from connecting blocks that return incompatible values.

### Connections

Connections determine how blocks can be connected to each other.

| Connection | Description |
|---|---|
| No connections | Standalone block |
| Output | Returns a value and connects horizontally |
| Previous + Next | Instruction block that connects vertically |
| Previous only | Can be placed after another instruction |
| Next only | Can start an instruction sequence |

### Tooltip, Help URL and Colour

Additional properties include:

- **Tooltip:** description displayed when the user hovers over the block.
- **Help URL:** optional documentation link accessible from the block's context menu.
- **Colour:** block colour expressed as an HSL hue or supported colour value.

### Output Configuration

Blockly Developer Tools can generate:

- **Block Definition:** JSON describing the block.
- **Generator Stub:** JavaScript function template for translating the block into code.

For SHAPE, configure:

- Block Definition format: `JSON`
- Code generator language: `Python`

You do not need to copy the Code Headers, because Blockly imports are already handled by SHAPE.

---

# 6. Adding Blocks to an Existing Module

This section demonstrates how to create custom blocks and register them in SHAPE.

We will use two examples:

- `to_uppercase`: converts a string to uppercase.
- `quadratic_equation`: calculates the real solutions of a quadratic equation.

Both blocks will be added to a new toolbox category called `Custom Math`.

## 6.1 Step 1 — Create a Toolbox Category

Open the Shapes editor template:

`tactigon_shapes/modules/shapes/templates/shapes/edit.jinja`

The toolbox is organized into XML categories.

Add your new category at the appropriate location:

```xml
<category name="Custom Math" colour="#ff00f7">

    <block type="to_uppercase">
    </block>

    <block type="quadratic_equation">
        <field name="op_1">PLUS</field>

        <value name="a_coeff">
            <shadow type="math_number">
                <field name="NUM">1</field>
            </shadow>
        </value>

        <field name="op_2">MINUS</field>

        <value name="b_coeff">
            <shadow type="math_number">
                <field name="NUM">3</field>
            </shadow>
        </value>

        <field name="op_3">PLUS</field>

        <value name="c_coeff">
            <shadow type="math_number">
                <field name="NUM">2</field>
            </shadow>
        </value>

    </block>

</category>
```

The category contains both custom blocks.

The `shadow` blocks provide default numeric values for the equation coefficients.

> **Important:** Block types, field names and input names must match their corresponding Blockly definitions. Avoid duplicate block identifiers.

![Custom Math category in edit.jinja](https://github.com/TactigonTeam/Tactigon-Shape/wiki/images/edit_jinja_category.png)

## 6.2 Step 2 — Design the Blocks

Open Blockly Developer Tools and create the first block.

### Example A — to_uppercase

Configure the block with:

- Name: `to_uppercase`
- Input: `TEXT`
- Input type: Value
- Connection check: `String`
- Previous connection: enabled
- Next connection: enabled
- Colour: `#f1c40f`

The resulting JSON definition is:

```json
{
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
    "colour": "#f1c40f"
}
```

### Example B — quadratic_equation

This block represents the equation:

\[
ax^2 + bx + c = 0
\]

It accepts three numeric coefficients and three dropdown fields to select their signs.

Its JSON definition is:

```json
{
    "type": "quadratic_equation",
    "message0": "%1 %2 x² %3 %4 x %5 %6 = 0",
    "args0": [
        {
            "type": "field_dropdown",
            "name": "op_1",
            "options": [
                ["+", "PLUS"],
                ["-", "MINUS"]
            ]
        },
        {
            "type": "input_value",
            "name": "a_coeff",
            "check": "Number"
        },
        {
            "type": "field_dropdown",
            "name": "op_2",
            "options": [
                ["+", "PLUS"],
                ["-", "MINUS"]
            ]
        },
        {
            "type": "input_value",
            "name": "b_coeff",
            "check": "Number"
        },
        {
            "type": "field_dropdown",
            "name": "op_3",
            "options": [
                ["+", "PLUS"],
                ["-", "MINUS"]
            ]
        },
        {
            "type": "input_value",
            "name": "c_coeff",
            "check": "Number"
        }
    ],
    "inputsInline": true,
    "output": "Array",
    "colour": "#ff00f7",
    "tooltip": "Calculates the real solutions of a quadratic equation."
}
```

## 6.3 Step 3 — Register the Blocks in SHAPE

Open:

`tactigon_shapes/modules/shapes/static/js/custom_blocks.js`

This file contains the definitions and generators for the custom blocks.

Find the `loadCustomBlocks()` function.

Add a call to your new block group:

```javascript
loadCustomMathBlocks();
```

Then define the function:

```javascript
function loadCustomMathBlocks() {

    const blocksDefinitions =
        Blockly.common.createBlockDefinitionsFromJsonArray([

            {
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
                "colour": "#f1c40f"
            },

            {
                "type": "quadratic_equation",
                "message0": "%1 %2 x² %3 %4 x %5 %6 = 0",
                "args0": [
                    {
                        "type": "field_dropdown",
                        "name": "op_1",
                        "options": [
                            ["+", "PLUS"],
                            ["-", "MINUS"]
                        ]
                    },
                    {
                        "type": "input_value",
                        "name": "a_coeff",
                        "check": "Number"
                    },
                    {
                        "type": "field_dropdown",
                        "name": "op_2",
                        "options": [
                            ["+", "PLUS"],
                            ["-", "MINUS"]
                        ]
                    },
                    {
                        "type": "input_value",
                        "name": "b_coeff",
                        "check": "Number"
                    },
                    {
                        "type": "field_dropdown",
                        "name": "op_3",
                        "options": [
                            ["+", "PLUS"],
                            ["-", "MINUS"]
                        ]
                    },
                    {
                        "type": "input_value",
                        "name": "c_coeff",
                        "check": "Number"
                    }
                ],
                "inputsInline": true,
                "output": "Array",
                "colour": "#ff00f7",
                "tooltip": "Calculates the real solutions of a quadratic equation."
            }

        ]);

    Blockly.common.defineBlocks(blocksDefinitions);
}
```

After saving, restart SHAPE.

The new category and blocks should now be available in the visual editor.

![Custom Math blocks in the SHAPE toolbox](https://github.com/TactigonTeam/Tactigon-Shape/wiki/images/20260430135113.png)

## 6.4 Step 4 — Write the Python Logic

Registering a visual block is not enough. We also need to define what happens when the block is executed.

This step has two parts:

1. Implement the Python helper function.
2. Implement the JavaScript generator that produces the corresponding Python code.

SHAPE supports different Blockly code generator targets, but Python is the recommended language for the project's implementations.

### 4a — Define Python Helper Functions

In `custom_blocks.js`, locate:

`defineImportsAndLibraries()`

This function returns a Python code string containing the imports and helper functions required by the generated program.

Add your implementation inside the returned template string.

For example:

```javascript
function defineImportsAndLibraries() {

    return `
# Shapes by Next Industries

# imports and functions

def quad_equation(a, b, c):

    import math

    if a == 0:
        return [-c/b] if b != 0 else []

    delta = b**2 - 4*a*c

    if delta < 0:
        return []

    elif delta == 0:
        return [-b / (2*a)]

    else:
        return [
            (-b + math.sqrt(delta)) / (2*a),
            (-b - math.sqrt(delta)) / (2*a)
        ]

# ---------- Generated code ---------------

`;
}
```

The function handles:

- Degenerate linear equations (`a = 0`).
- Equations without real solutions (`delta < 0`).
- Equations with one real solution (`delta = 0`).
- Equations with two real solutions (`delta > 0`).

### 4b — Register the Generator Group

Find `defineCustomGenerators()`.

Add a call to your new generator group:

```javascript
defineCustomMathGenerators();
```

### 4c — Implement the Blockly Generators

Below `defineCustomGenerators()`, define the generator functions.

```javascript
function defineCustomMathGenerators() {

    // Uppercase generator

    python.pythonGenerator.forBlock['to_uppercase'] =
        function(block) {

            const text_to_print =
                Blockly.Python.valueToCode(
                    block,
                    'TEXT',
                    Blockly.Python.ORDER_ATOMIC
                ) || "''";

            const code =
                `debug(logging_queue, ${text_to_print}.upper())\n`;

            return code;
        };


    // Quadratic equation generator

    python.pythonGenerator.forBlock['quadratic_equation'] =
        function(block, generator) {

            const value_a =
                generator.valueToCode(
                    block,
                    'a_coeff',
                    generator.ORDER_ATOMIC
                ) || '0';

            const value_b =
                generator.valueToCode(
                    block,
                    'b_coeff',
                    generator.ORDER_ATOMIC
                ) || '0';

            const value_c =
                generator.valueToCode(
                    block,
                    'c_coeff',
                    generator.ORDER_ATOMIC
                ) || '0';


            const op_1 = block.getFieldValue('op_1');
            const op_2 = block.getFieldValue('op_2');
            const op_3 = block.getFieldValue('op_3');


            const a_finale =
                (op_1 === 'MINUS') ? '-' + value_a : value_a;

            const b_finale =
                (op_2 === 'MINUS') ? '-' + value_b : value_b;

            const c_finale =
                (op_3 === 'MINUS') ? '-' + value_c : value_c;


            const code =
                `quad_equation(${a_finale}, ${b_finale}, ${c_finale})`;

            return [code, Blockly.Python.ORDER_ATOMIC];
        };
}
```

![Custom Blockly generators](https://github.com/TactigonTeam/Tactigon-Shape/wiki/images/define_generators.png)

> **Important — Generator vs. Python Function:** The generator does not perform the actual calculation. It produces a string of Python code, such as `quad_equation(1, -3, 2)`. The Python helper function defined in `defineImportsAndLibraries()` performs the calculation when the generated program runs.

## 6.5 Step 5 — Run and Test

Restart SHAPE and open the visual editor.

You should now see the `Custom Math` category containing both blocks.

![Custom Math blocks example](https://github.com/user-attachments/assets/88637bd1-7d87-47d5-aa88-7c7b3e99c951)

Drag the blocks into the workspace, connect the required inputs and run your Shape.

For the quadratic equation example, use:

\[
x^2 - 3x + 2 = 0
\]

The expected output is:

```python
[2.0, 1.0]
```

For the uppercase example, connect a string and verify that the Debug output displays its uppercase representation.

---

# 7. Adding Blocks to an Existing Module

If you only need to extend an existing module, you do not need to create a new module or register additional Flask Blueprints.

Instead:

1. Identify the existing module responsible for the functionality.
2. Implement the new Python helper function in its `extension.py` or appropriate backend component.
3. Add the Blockly JSON definition to the appropriate block-loading function in `custom_blocks.js`.
4. Register a generator for the new block.
5. Add the block to the existing toolbox category in `edit.jinja`.
6. Restart SHAPE and test the generated program.

This approach is useful for extending existing integrations without introducing unnecessary backend components.

Keep module-specific functionality in the appropriate module instead of adding unrelated code to the Shapes core.

---

# 8. Testing Your Changes

Before submitting a contribution, make sure your changes work as expected.

We recommend the following checks:

- Start SHAPE using your locally built Docker image.
- Verify that the application starts without errors.
- Verify that your new module is correctly initialized.
- Check that Flask routes and web interfaces work correctly, when applicable.
- Verify that custom blocks appear in the toolbox.
- Connect blocks and check that the generated Python code is correct.
- Execute the generated Shape and verify its behavior.
- Check that existing functionalities are not affected.
- Verify integrations with external services or hardware, when applicable.

For contributions involving hardware or external services, include the relevant testing requirements in your Pull Request.

---

# 9. Submitting a Pull Request

Once your changes are ready, follow these steps.

## 9.1 Review Your Changes

Check the modified files:

```bash
git status
git diff
```

Make sure that:

- Only files related to your contribution are included.
- No credentials, tokens, passwords or other sensitive information are committed.
- No local development configuration changes are included.
- Generated files and temporary artifacts are excluded when not required.

In particular, restore the original Docker configuration if you modified `docker-compose.yml` or `start.sh` for local development.

## 9.2 Commit Your Changes

Use clear and descriptive commit messages:

```bash
git add .
git commit -m "Add custom MQTT block"
```

## 9.3 Push Your Branch

```bash
git push origin feature/my-new-feature
```

## 9.4 Open a Pull Request

Open a Pull Request against the `master` branch of the original repository:

https://github.com/TactigonTeam/Tactigon-Shape/pulls

Your Pull Request should include:

- A clear title describing the contribution.
- A description of the problem being solved or the feature being introduced.
- A summary of the main changes.
- Instructions to reproduce or test the changes.
- Screenshots or videos when they help explain new blocks or UI features.
- Additional dependencies, configuration or hardware requirements.

Keep Pull Requests focused on a single feature or bug fix whenever possible.

The maintainers will review your contribution and may request changes before merging.

---

# Reporting Issues

If you find a bug, encounter installation problems or have suggestions for improvements, please open an issue:

https://github.com/TactigonTeam/Tactigon-Shape/issues

When reporting a bug, include as much relevant information as possible:

- Operating system and version.
- Docker and Docker Compose versions.
- Steps to reproduce the issue.
- Expected behavior.
- Actual behavior.
- Relevant error messages and logs.
- Screenshots or other useful information.

---

# Thank You!

Every contribution helps make SHAPE better.

Thank you for helping us improve Tactigon SHAPE and its ecosystem!
