# About

This section provides a comprehensive explanation of each block’s function and purpose, detailing not only what they do but also how they operate within the tactigon shape. We will explore their underlying mechanics, interactions, and overall role in the tactigon shape.

## Import and Export Shapes

it's possible to save Shapes on your computer and share them with other users. follow these quick steps:

### Export
![image](https://github.com/user-attachments/assets/24bc1689-eae1-4432-a29c-dc5d9d11bcb3)

- Select Export (next to Shape name).
- Confirm → File saves as [ShapeName].zip in the Downloads folder

### Import
![image](https://github.com/user-attachments/assets/58a3ced9-3e2a-427d-9eef-c14b1724610b)

- Click Import (top of Shapes list).
- Select .zip → Edit → Save to generate executable.

## Notes
- Choosing a folder or file to import that is not a Shape will cause an error message to show.
- Retro compatibility with the app versions is not possible, importing a Shape from an updated version into an older one could create visual and/or functionality bugs.


# SHAPE VALIDATION
While building your Shape the system checks for errors that could create issues within the created program. these are the errors you may see during editing. The validation occurs when pressing the “save Shape” button, error messages will appear on the top right corner, preventing the saving of the shape.

## Floating Blocks
Any block that is not a function block or a variable setting block (e.g. any block with a left connection, Debug, keyboard Press…) must be connected to another block inside the Main app or a function block. The blocks that CAN stay on top (the most external ones) are:
- The main Tactigon APP
- all function blocks
- "set variable" blocks, connected to the relative value (these blocks will specify the set value only once, before the main program starts)

## Reserved Names
a list of words is forbidden to use as variable or function name in order to avoid conflicts

- the forbidden words are: "zion","code","logging_queue","debug","data","main","tskin","tspeech","gesture","ironBoy","touch","braccio"

![image](https://github.com/user-attachments/assets/c0646a5b-d5a2-421a-bc98-f53c0224e117)

# VARIABLES

Okay, Let's start with the basics. imagine you have a box where you can store different things, like a name, a number, or even your favorite color. 

A **variable** is like that box in the programming world.

Let’s look at an example. If you have a variable called age, you can store the number 20 in it. Later, when your age changes, you can update it to 21. Here, what we’re doing is telling the computer, "Please remember my age so I can use it whenever I need it."

### How to create a variable
Creating a variable is like giving a name to your storage box and putting something inside it. Here is an example of how to create a variable with Tactigon Shape.

https://github.com/user-attachments/assets/9001e75f-9a2e-4f62-a28b-594b858b1e24

### How we can assign something to a variable
After you create the variable, you can easily assign a value by dragging one of our blocks with a value (like a number, word, or color).

Now let the computer remember it for later use.

Example:
Let’s say you want to store a number in a variable, You can follow this video and try on your own.

https://github.com/user-attachments/assets/a2cec112-d2ec-45ca-9187-600994d7e159

If you do not assign a value to a variable, it will give you an error because the variable has not been assigned a value

### How to change the value of my variable
Perfect! Now, I have a variable with a value assigned to it—in this case, 2. Hmm… now I want to increment that value by 1, so I can tell the computer, 'Hey, please add 1 to this variable.'

You can follow this tutorial and try it with your shape.

https://github.com/user-attachments/assets/0826f00d-16c9-470a-9c83-d1cb3b5868ab

# LIST

Okay, but let's say we want to store not just one number or one color. Instead, we want to create multiple values, like grocery items. We could create multiple variables, but that would be time-consuming, and it would be difficult to remember all the variables. That's where a list comes to the rescue.

Think of a list as a container or box that holds multiple things, such as numbers, words, or even other lists.

Example of a List:
Let’s say you have a list of your favorite fruits:
We can create a variable called grocery and make it a list of grocery items. How does that sound?"

### How to create an empty list:
From the following tutorial, you can create an empty list. Along with the tutorial, you can try this on your own as well.

https://github.com/user-attachments/assets/20a139cb-26e4-416e-9816-49006ac35464

### Create list with:
Okay, now it's time to create a list, add some numbers, and at the same time, we can assign that list to a variable. Check this out!

https://github.com/user-attachments/assets/080012fd-9919-4558-9081-6abddfa25419

### Create list with item (block) repeated (number) times:
Imagine you want to say "Hello!" 5 times. Instead of writing it 5 times, you can use a repeat block and set it to 5. 

The computer will then automatically print "Hello!" five times.

In this case, we will show you how to create a list using a variable (here we assigned number 3) and repeat it 7 times. 

https://github.com/user-attachments/assets/5b2ee570-f0d4-4d7c-a36d-a9e129d15f06


### Length of:
The length of a block is the number of items (or elements) inside the list.

Imagine you have a list (array) of grocery items:

`grocery = ["apple", "banana", "carrot"]`

The list has 3 items in it: "apple", "banana", and "carrot". Now the length of this list is 3 because there are 3 items in it. 

https://github.com/user-attachments/assets/b7046602-b1d2-4164-aeeb-ff3950f281e4

### Is empty:
"is empty" block is a way to check if your list has no items inside it.

**Example:**
Let's say you have an empty list:

`grocery = []`

This list has no items in it. If you check whether it's empty, the answer will be "True" because there’s nothing inside the list. In this case "True" means "Yes, it is empty". 

https://github.com/user-attachments/assets/53f10e8e-a992-48bd-8b75-cdb69de0a71b

### In list [list name] find [operation type] occurrence of item (block):

Imagine you are in a library, and you have a list of books arranged on a shelf.

`books = ["Harry Potter", "Lord of the Rings", "Harry Potter", "Sherlock Holmes"]`

Now, you want to find the position of "Harry Potter" in the list.

* Find the first occurrence → The block will return 1 (since "Harry Potter" appears first at position 1).
* Find the last occurrence → The block will return 3 (since "Harry Potter" appears again at position 3)
* Find a book that isn’t in the list → If you search for "Percy Jackson," the block will return 0 (because it doesn't exist in the list).

You can find the first or last occurrence of an element in a given list. To do this, first, select the name of the list in the first field. Then, choose whether you want to find the first or last occurrence of the element. Finally, insert a block containing the name of the element you want to find in the selected list. Also, remember that this block will return 0 if the element is not found in the list

https://github.com/user-attachments/assets/3d6b3ba4-b482-4193-bb29-18334eb534f8

### In list [list name] [operation type] [order by] (block)

Imagine you have a to-do list with tasks you need to complete:

`tasks = ["Buy groceries", "Clean the house", "Pay bills", "Walk the dog"]`

Now, let's say you want to get or remove a task from the list using the Blockly block.

Example 1: Getting a Task from the List
* You use the block to retrieve the first task from the list.
* The block returns: "Buy groceries" (since it’s the first item).

Example 2: Removing a Task from the List
* You use the block to remove the last task from the list.
* "Walk the dog" gets removed.

Updated list: 

`["Buy groceries", "Clean the house", "Pay bills"]`

You can get or remove one or more elements from a given list. To do this, first, select the list you want to work on. In the second slot, choose the type of operation you want to perform on the list. In the third slot, specify the order in which you want to work on the list. Lastly, in the fourth slot, insert a block containing the element you want to search for or remove.

https://github.com/user-attachments/assets/96c9c7cf-8248-40e0-bf2d-71df1b602717

### In list [list name] [operation type] [order by] (block) as (block)

You can use this block to set an object at a defined position in the list or insert a new object into a given list at a specific spot. To do this, first, select the name of the list you want to work on in the first slot. Then, choose the type of operation you want to perform in the second slot. Next, specify the order of the operation in the third slot and enter the position where you want to place the new object in the fourth slot. Finally, enter the name of the element in the last slot

https://github.com/user-attachments/assets/5f00407f-1e28-49c2-a034-ef8670b04bc8

# DICTIONARY

Think of a dictionary as a contact list on your phone. Each person’s name is saved, and next to it, you store their age.

Example in Simple Terms
Let’s say you want to store a person's name and age:

* "name" is the key, and "John" is the value.
* "age" is the key, and 30 is the value.

This means you can ask the dictionary for John's age by looking up the "age" key.

It's like asking:
> "What is John's age?"
> The dictionary answers: 30

### In dictionary (dictionary name) Get value for key [key]
Using this block, you can retrieve the value of a given key in a dictionary. To do this, place the name of an existing dictionary in the first slot and the key associated with the value in the second slot.

https://github.com/user-attachments/assets/04d44919-0920-4bb5-b3e3-9f12bdc41d8b


# Tactigon Skin

Tactigon Skin is a Bluetooth Low Energy wearable device capable of capturing:
- Angles
- Accelerations
- Gyro
- Audio
- Touch interactions such as Tap, Tap and Hold and many more
- Gesture such as up, down, swipe, and many more

Every device is paired with a Docking Station that provides wireless charging feature via a USB C cable.

With the Tactigon Skin connected to Tactigon Shape, **You can bring your ideas to life using simple drag-and-drop coding**. You can create gesture-controlled games, build a smart light you can turn on with a hand gesure or a single tap, or even control a robot just with your hand! It’s a fun and hands-on way to learn how sensors, motion, and programming work together — no advanced coding needed. Whether you’re into tech, design, or just love building cool stuff, this tool lets you explore creativity while learning real STEM skills.

### Gesture

Here we are going to show you how you need to use gestures with the Tactigon Skin device.

**Circle**

<img src="https://github.com/user-attachments/assets/db4ccd0d-1f72-46ae-badd-6f6933a2003a" alt="circle" width="400"/>

**Down**

<img src="https://github.com/user-attachments/assets/8c0ae359-58e4-4d3f-8072-18665bcc666b" alt="down" width="400"/>

**Up**

<img src="https://github.com/user-attachments/assets/e3c540eb-bed7-4a0a-a8ee-3ecea3ba1d10" alt="up" width="400"/>

**Swipe Right**

<img src="https://github.com/user-attachments/assets/bbd3491d-1119-4c81-b144-c7cf71309fdc" alt="swipe left" width="400"/>

**Swipe Left**

<img src="https://github.com/user-attachments/assets/10f373d4-134f-47d5-a8dd-352a9d0cfdea" alt="swipe right" width="400"/>

**Push**

<img src="https://github.com/user-attachments/assets/875c3a59-152f-485f-97aa-8b9e8b336f28" alt="push" width="400"/>

**Pull**

<img src="https://github.com/user-attachments/assets/d2d5d61d-cc22-45e9-8f2c-e7afd1b2e783" alt="pull" width="400"/>

**Swipe**

<img src="https://github.com/user-attachments/assets/f007e5f8-b6fe-42f4-a599-1623ed13478d" alt="swipe" width="400"/>

This block allows you to use various gestures with the Tactigon Skin device. Simply drag the gesture block and select the gesture you want to perform. When you perform the selected gesture, it returns 'true', so you can use it with an 'If' block.

https://github.com/user-attachments/assets/4c1f10f9-989f-4d70-810b-5f341e841c14

### Touch

Here you can see some of our touch gestures and how you can use them.

**Single Tap**

<img src="https://github.com/user-attachments/assets/7ff3ab86-63c8-4583-8cfe-e0487932daff" alt="single tap" width="400"/>

**Tap and Hold**

<img src="https://github.com/user-attachments/assets/eda62e12-2f5c-4ceb-b923-bf14f769e2cd" alt="tap and hold" width="400"/>

This block allows you to use touch gesture. Simply drag the touch block and select the touch you want to perform. When you perform the selected touch, it returns 'true', so you can use it with an 'If' block.

https://github.com/user-attachments/assets/fdadcd89-ee35-47d9-ad66-8518ebaae704

### Angle

This block gives you the angle value when using the Tactigon Skin device. It returns a number, which you can use for calculations or comparisons.


https://github.com/user-attachments/assets/854b2fed-fba6-4843-ae40-408a967a980d

### Gyro
This block gives you the gyro value when using the Tactigon Skin device. It returns a number, which you can use for calculations or comparisons.

https://github.com/user-attachments/assets/c93462ec-5010-4bb6-af02-1310d175ea49

### Voice

The Tactigon Skin device is no longer limited to gestures and other features — we’ve now added speech recognition and more. Here is the detailed guide to the voice features.

### Voice Command

This block is used to analyze speech input from a Tactigon Skin device. You can run your shapes simply by speaking to the device. When using this block, please check the console for further instructions

https://github.com/user-attachments/assets/9eacfa8d-b867-4d58-bd6e-cb0b3c9cc80b

### Record

This function records audio from the Tactigon Skin device while you are speaking and you can provide a file name to save the recorded audio.

https://github.com/user-attachments/assets/2b839ed6-69ee-44d1-bdb3-2a0895acc818

### Play file audio

This is a block that plays an audio file. It takes the filename of the audio file as a parameter and plays it through the computer. Make sure to record an audio or provide a valid audio file.

https://github.com/user-attachments/assets/b7f43829-8d0d-44f9-86c0-b09fa39e1b31

# Braccio

## What is Braccio?

The Braccio is a programmable robotic arm. It can be assembled in various configurations to perform tasks like moving objects, and you can control the Braccio using the Tactigon Shape app. We have already created blocks to make it fun to use. 


### Open/Close Gripper
You can send a command to open or close the gripper using this block. With it, you can pick up an object and drop it wherever you want.

The gripper is the part of the Braccio arm that acts like a hand, allowing it to pick up, hold, or release objects. It typically consists of two "fingers" that open and close to grasp items securely.

https://github.com/user-attachments/assets/59088a8c-cb70-4910-a191-52c1c5d1f0c4

### Rotate Wrist Vertically/Horizontally

The wrist is the joint that connects the gripper to the rest of the Braccio arm, enabling flexible movement like rotation or tilting. It allows the gripper to adjust its orientation for precise positioning of objects.

https://github.com/user-attachments/assets/1d4e067f-a614-4724-b384-399c707c12e9

### Move to a position

The Braccio robot arm uses a 3D coordinate system to move:

- **X-axis (left ↔ right):**  
  - Negative = left, Positive = right  
  - Example: X = -200 → left, X = 200 → right  

- **Y-axis (back ↔ forward):**  
  - Lower = backward, Higher = forward  
  - Example: Y = 50 → backward, Y = 250 → forward  

- **Z-axis (down ↔ up):**  
  - Lower = down, Higher = up  
  - Example: Z = -50 → down, Z = 150 → up  

Using move block, you tell the arm where to go in 3D space. The graph shows which positions the arm can actually reach—if a point is outside the range, the arm can't go there.

Here is a graph that you can easily find the values that you can pass.

<img src="https://github.com/user-attachments/assets/4dee4ef3-42de-4510-86d0-38d42af2bf94" alt="tap and hold" width="800"/>

![Valid Position Analysis for Braccio Arm](https://github.com/user-attachments/assets/f9101f22-7062-4442-9875-a16c978bd26b)


Now, let's move the Braccio device! You can use the Move block within the Braccio category, which takes three values: X, Y, Z.

For example, to move from one position to another, you can try this:

https://github.com/user-attachments/assets/9037a45c-81af-43be-9ddd-1c47ee0deed6


Check it out!

## How to connect Braccio 

We will guide you step by step on how to connect the Braccio device with the Tactigon Shape.

First, make sure that both the Braccio and Tactigon Shape are powered correctly. Once powered on, the Braccio will align to a straight position.

Next, open the Tactigon Shape app and click on the Braccio icon.

![image](https://github.com/user-attachments/assets/b1bd45ff-e4eb-487f-b3a5-e657ccb0887a)

You will see that our app is searching for the Braccio device. Once it is found, you can assign a name and save it.

![image](https://github.com/user-attachments/assets/c063066e-32aa-4b9c-8382-5dce9d3ce13b)

Now, click 'Start Braccio' to connect to the device.

![image](https://github.com/user-attachments/assets/05898653-cfc3-4311-a7c7-db4ca96472fe)

Perfect! Now the Braccio is connected, and you can move it or send any commands to the Braccio device.


# Iron-Boy
## What is Iron-Boy

Iron-Boy is a little robot that comes with built-in movements. Through Shapes we can make him wave, dance and much more. it uses 16 servomotors as its muscles for arm and legs.

![image](https://github.com/user-attachments/assets/54c9462d-f6c0-4e09-a711-a5e4d04cc1fb)




## How to connect Iron-Boy

Here's how to connect the Iron-Boy device with Tactigon Shape.

Check that Iron-Boy is powered and the press the switch on his back.

He will try to stand up on his own.

Next, open the Tactigon Shape app and click on the Iron-Boy icon in the top-right corner.


You will see that our app is searching for the device. Once it is found, you can assign a name and save it.


![image](https://github.com/user-attachments/assets/fa4dc71f-5f8d-411e-a4b6-d4f3c2d209f8)


Now, click 'Start Iron-Boy' to connect to the device. you will see a blue LED on the Iron-Boy board turn on.

The Iron-Boy is ready to go.

![image](https://github.com/user-attachments/assets/568d465f-ab14-4820-aa3e-7cdbe364fe37)


## Iron-Boy commands

![image](https://github.com/user-attachments/assets/fb66ee3f-a1c2-44d9-8e29-bc1aa7e49d37)

In the “edit Shape” page a new menu is available called “IronBoy”.

- Command list, contains all the preconfigured IronBoy movements
- Command” block, send commands directly to IronBoy. Contains the command list and a repetition number

## Possible commands
### ...to walk
- Walk (forward or backwards)
- turn (left or right)
- tiptoe (in all 4 directions)

### ...to say "Hi"
- Wave
- Arm wave
- Bow down
- Clap
- Celebrate

### ...to fight!
- Provoke
- Side kick

### ...to transport small objects
- Pick up ball
- Move ball forward (with object in his hands)
- Drop ball

## Notes
- IronBoy movements need to end before the shape continues its main loop.
- The Shape execution doesn’t stop when clicking the “off” switch if IronBoy is in the middle of a movement.

## Troubleshooting:

- Bluetooth scan failed → reset IronBoy and make sure it’s placed close to your device with no obstruction in between
- “Command Error” → visible during program run, try to send command again
- Repeated “Command error” → Check IronBoy blue LED and status in the IronBoy page, reset if needed
- “IronBoy not configured” → IronBoy configuration file is not found, try disconnecting from device and reconnecting
- Blue LED is on but unresponsive → Stop IronBoy and Start again.



 


# ZION
Zion is a platform designed to help businesses manage and visualize data from their connected devices or systems. It allows you to gather data from sensors, machines, or any smart devices and then display it in a clear, user-friendly way. With Zion, you can track real-time information, analyze trends, and make informed decisions based on the data you collect.

In short, Zion is a powerful tool for businesses to make sense of data from their devices, making operations more efficient and data-driven.

## Why Zion with Tactigon Shape
Here, you can use the Zion platform with our Tactigon shapes. It's time to wake up your creativity with these two platforms. Let's go through our Zion blocks, and then you can connect various devices to our Tactigon shapes and interact with the Zion platform to manage devices. Let us know your creative shapes by creating them and contributing to our project.

### How to login to Zion Cloud

In order to use Zion within Tactigon Shape, you need to have an account on the Zion platform. If you do not know your Zion login credentials or have any issues accessing Zion, please contact a member of our Tactigon crew.

When you are ready to use Zion Cloud, you can follow these guides to log in to Zion. Only then will you be able to see the Zion blocks.

![image](https://github.com/user-attachments/assets/18407020-250d-4fc2-9b8b-4b48d1a025fb)

You will see the login page for the Zion platform. Here, please provide your Zion credentials, which were provided by the Taction crew.

![image](https://github.com/user-attachments/assets/a53242d3-3d7d-42d8-ae55-a4d0505686e3)

Once you successfully log in to the Zion platform, you can see a page similar to this with all the devices.

![image](https://github.com/user-attachments/assets/5c919a1b-121a-412c-a01a-1b2e30eb2acd)


### Get device last telemetry
This block is like asking your smart device: "Hey, what's your latest status?"

You can also see that the device block is already attached to the Zion 'Get Last Telemetry' block. Here, you can select which device you want to use, and the 'filter key' is like asking for just one piece of info instead of everything—like asking, 'Just tell me the battery level,' instead of getting battery, temperature, and location all at once.


https://github.com/user-attachments/assets/5032e8e7-aba1-468f-bf6f-e95d4d96f06b

### Get device attribute

This block is like asking a smart device: "Hey, what settings do you have saved?"

Here, it's asking a device called "NI300_TH" about value or the "alarm_threshold". The "SERVER_SCOPE" means it checks the main settings saved on the server.

Don't forget that you can change the device and attribute name; for example, you can change the device to Braccio and you can fetch the value for the x-axis.

https://github.com/user-attachments/assets/e2e2f7e9-3297-4b74-9d0f-091530ddc17d

### Get device alarm
This block is like asking: "Is my device having any serious problems right now?"

Here, we use the NI300_TH device, and the status is CRITICAL, so here we can get data that is more important. 

https://github.com/user-attachments/assets/06974096-9968-44e6-a12d-2fb2de80d230

The block focuses on CRITICAL (most important) and ACTIVE (happening now) alarms so you don't get overwhelmed with minor or old notifications.

### Update device last telemetry

This block is like changing the current reading of your smart device.

You can update device telemetry data with the Zion update block. In this example we say please update the temperature_zion to 23.5 for NI300_TH device.

https://github.com/user-attachments/assets/b2f74013-339d-472e-9271-fa4f93f4566e

### Update device attribute

This block is like changing the settings or attributes stored on your smart device.

This example shows you that we update an attribute or key for device "N300_TH" by changing values stored in the SERVER. 

So we say, please update "automatic" key as true


https://github.com/user-attachments/assets/ab2b4b8d-e47a-48ce-9d28-f4505c337490

### Update device alarm

This block is like we say "Hey we have a serious problem with this device".

This example shows you that we create an alarm for "test device" by updating the device alarm block.


https://github.com/user-attachments/assets/f9e353db-4cce-4823-a488-74bce78ce78f

# ROS and MQTT
MQTT is a lightweight network protocol designed for IoT connectivity using a Client-Broker architecture.

    Publisher block: The application connects to a central Broker and transmits data payloads to specific identification strings called Topics.
    Receiver block: The application subscribes to specific Topics on the Broker to intercept and process data routed from other clients.

ROS

ROS is a middleware framework used for robot control that enables communication between independent processes known as Nodes.

    Publisher node: The application functions as a Node, broadcasting serialized data messages over named communication buses called Topics.
    Receiver node: The application implements subscriber logic to listen to specific Topics, capturing and deserializing messages broadcast by other Nodes.

# GINOS
Ginos is the new Tactigon Shapes Architecture to implement the use of AI inside the app.
Powered by Ollama, a new Prompt block lets us send query to different models installed directly into your PC.

# How to run Tactigon Shapes using Docker

### Prerequisites

Make sure you have the following requirements:

- Linux environment (Tested on Ubuntu 24.04)
- [Docker](https://docs.docker.com/get-docker/)

![Pasted image](https://github.com/user-attachments/assets/da45b383-8fb1-4045-b1f0-aae3a696fcf2)

### Setup Instructions

We have created a file named docker-setup.sh - (Linux, macOS) to configure everything for you.

1. **Make the script executable**

   Go to the project root and open the project with the terminal, then follow this command to make the script file an executable file:

   ```bash
    chmod +x docker-setup.sh
   ```
![Screenshot from 2025-06-30 15-02-41](https://github.com/user-attachments/assets/a605aa0e-d94b-4cc9-ab87-9be959d9d86b)

2. **Run the script from the graphical session**

   ```bash
    ./docker-setup.sh
   ```
![Screenshot from 2025-06-30 15-03-21](https://github.com/user-attachments/assets/6d467966-5f40-4561-8c94-0d54c6512aec)

3. **Wait till the building process and run the Docker Container**

4. **Final output**
    
    Once the container is up and running, you will see a URL in the terminal output. Open this URL in your browser to use the Tactigon Shape application.


