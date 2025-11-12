### Prerequisites

In order to use the Tactigon SDK, the following prerequisites need to be observed:

- **Mac/Linux:**
To set the required tools on your system, follow the steps below:

  - CPU with AVX/FMA support
    - To check if your Mac supports **AVX/FMA** you can go to **About This Mac**. Check the CPU model and verify its features online.
  - Python 3.8.10 [Download Python 3.8.10](https://www.python.org/downloads/release/python-3810/)

- **Windows:**
To set up the required tools on your Windows 10 or Windows 11 operating system, follow the steps below:
  - CPU with AVX/FMA support
    - To check if your CPU supports AVX/FMA you can press **Win + R**, then type **msinfo32** and press Enter. Check the CPU model and verify its features online.
  - Python 3.8.10 [Download Python 3.8.10](https://www.python.org/downloads/release/python-3810/)
  - Microsoft C++ Build Tools and Windows 10/11 SDK. 

- **Microsoft C++ Build Tools and Windows 10/11 SDK installation:**
  - Get the Visual Studio Installer:
    - If you don’t already have it, [Download Visual Studio Installer](https://visualstudio.microsoft.com/downloads/)  from Microsoft’s website.
  - Open the Installer:
    - Run the installer. You’ll see options to add or update tools.
  - Choose What to Install:
    - Click Modify (if you already have Visual Studio installed) or Install (for a new setup).
  - Pick the Right Tools:
    - Go to the Individual Components tab.
    - Check these two boxes:
      - MSVC v143 - VS 2022 C++ Build Tools x64/x86 (latest version)
      - Windows 10/11 SDK (pick the newest version that matches your Windows version, e.g., Windows 11 users choose the Windows 11 SDK).
  - Finish Installation:
    - Click Install/Update and wait for the process to complete.

![Screenshot 2024-11-21 162339](https://github.com/user-attachments/assets/5f6332f1-be2b-4fee-ad62-7feb734db710)

![image](https://github.com/user-attachments/assets/84992453-3fc3-4808-b3e0-17e734881650)

![image](https://github.com/user-attachments/assets/e5b29bb3-0e81-4daf-98d6-e1c829b409bb)


### Steps to Install:
1. Download the repository:
![Immagine 2025-02-12 101837](https://github.com/user-attachments/assets/4065de26-cb74-453f-9c3f-32497c698408)
   
2. Pick a location and extract all the files. you will see a folder named **Tactigon-Shape-master**.

2. Navigate to the project directory or open the project with your default code editor or IDE.
   ```bash
   cd Tactigon-Shape-master
   ```
3. Activate the virtual environment and install dependencies:

   We have created an install file (install.bat - Windows / install.sh - Linux, macOS) to configure everything for you. Please execute that file; you will see the following output on your terminal.

![image](https://github.com/user-attachments/assets/e2b37b37-2f24-44e6-bab1-22191465d591)

<br>
<br>


## Start Tactigon Shapes 
We have created a start file (start.bat - Windows / start.sh - Linux, macOS). Please execute that file; you will see the following output on your terminal.

![image](https://github.com/user-attachments/assets/02c44feb-3d14-42e8-b97c-873240087bb8)

Follow the prompts to connect to your Tactigon Skin device and start using the Shapes interface.

![chrome_GBrADG4C6r](https://github.com/user-attachments/assets/30107a15-e4ca-424d-80c2-bca73e3c0370)
---

## Using Tactigon Shapes

Using Shapes is like building with LEGO bricks, you drag, drop, and connect pieces to create something functional. This section guides you step-by-step on how to use shapes.

Once you are on the Shape interface, you can see some of our example shapes, such as Powerpoint, Braccio voice, and so on. On the left side, you will see all your shapes, and on the right side, you can see a snapshot of your shape.

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

### How to run our Tactigon Shapes:
We’re diving into how you can control PowerPoint presentations with just a few simple gestures. Let’s get started!

First things first, make sure your TSkin is connected to the Tactigon Shapes app. You’ll know it’s connected when you see the battery percentage and the Bluetooth icon in white. If it’s gray, you’re not connected yet. On the left panel, you’ll find our default shapes, plus a couple of extras we’ve added to show off the possibilities.
Now, let’s talk about PowerPoint control. 

Here’s how it works: When you tap and hold, it toggles the presentation into full-screen mode. A single tap takes you to the previous slide, and a twist gesture moves you to the next slide. It’s super intuitive and easy to use.
I’ve got a basic PowerPoint open with a few slides. I’ll open the Tactigon Shapes app, click the toggle icon to start the program, and you’ll see both your shape and the output on the same screen.

Now, I’ll wear the TSkin and open the PowerPoint app. Watch this—tap and hold, the presentation goes full screen. if you want to move to the next slide, do twist gesture. Need to go back? A single tap does the trick. 

Ater your presentation, don’t forget to stop the program after your awesome presentation.

https://github.com/user-attachments/assets/62b4bb7d-c922-421f-8bdd-c856ba587669
