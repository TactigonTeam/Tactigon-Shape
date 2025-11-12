## Variables

### Exercise 1: Creating a Variable
Can you create a variable named "age" and set its value to 10? Then, print the value of the variable on the screen.


### Exercise 2: Changing a Variable's Value
Suppose you have a variable called "score", and its initial value is 5. Can you update it to 15 and then display the new value?


### Exercise 3: Performing Math with Variables

Create two variables, "num1" and "num2", and assign them the values 8 and 12, respectively. Then, create a third variable, "total", that stores the sum of these two numbers. Finally, print the value of "total".

### Exercise 4: Multiplication with Variables

A shop sells one apple for 3 coins. Create a variable "applePrice" and set it to 3. Then, create another variable "totalCost" and set it to the price of 4 apples (by multiplying applePrice by 4). Print "totalCost".

### Exercise 5: Swapping Two Variables

You have two variables, "x" (set to 5) and "y" (set to 10). Can you swap their values so that "x" becomes 10 and "y" becomes 5? Print both variables to verify the swap.


## Loops

### Exercise 1: Sum of List Elements

Create a Blockly program that calculates the sum of all numbers in a given list.

Expected Result:

* Input: [5, 10, 15, 20]
* Output: 50

Hints:

* Use create list with to define the list.
* Use a for each item in list loop to iterate.
* Use a variable (sum) to accumulate the total.


### Exercise 2: Find Maximum Value in a List

Write a Blockly program that finds the largest number in a list.

Expected Result:

* Input: [12, 45, 7, 32, 9]
* Output: 45

Hints:

* Initialize a variable (max) with the first element.
* Loop through the list and compare each element with max.
* Update max if a larger number is found.


### Exercise 3: Reverse a List

Create a Blockly program that reverses the order of elements in a list.

Expected Result:

* Input: ["A", "B", "C", "D"]
* Output: ["D", "C", "B", "A"]

Hints:

* Use an empty list (reversed_list).
* Loop from the end of the original list and append elements to reversed_list.

### Exercise 4: Count Even Numbers

Write a Blockly program that counts how many even numbers are in a list.

Expected Result:

* Input: [3, 8, 12, 7, 4]
* Output: 3 (since 8, 12, and 4 are even)

Hints:

* Use a counter variable (even_count).
* Check each number using if (number % 2 == 0)


### Exercise 5: Remove Duplicates from a List

Create a Blockly program that removes duplicate values from a list.

Expected Result:

* Input: [5, 2, 5, 8, 2]
* Output: [5, 2, 8]

Hints:

* Use an empty list (unique_list).
* For each item, check if it’s already in unique_list before adding.

# Tactigon Skin

### Exercise 1: Dice Rolling Game

Create a shape that simulates rolling a pair of dice. Each time the program runs, and when you use twist gesture it should randomly generate a numbers between 1 and 6 (inclusive), representing the result of each die. The shape should then display the results and ask if the user would like to roll again. to exit from the game you can use swipe right gesture.

![image](https://github.com/user-attachments/assets/9e958eb5-49f9-4acd-9b87-4634233e2549)


https://github.com/user-attachments/assets/a581a27f-e2a5-4932-a2af-dab58505dc10


This is the link for [solution to exercise 1](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-1-dice-rolling-game)

### Exercise 2: Rock, Paper, Scissors Game

Create a shape to simulate a game of Rock, Paper, Scissors.
The game will prompt the player to choose rock, paper, or scissors by using gestures. 

The Tactigon Shape will randomly select its choice. The game will then display both choices using emojis and determine the winner based on the rules.

- Rock - push
- Paper - up
- Scissor - swipe right

![image](https://github.com/user-attachments/assets/c7624112-2a12-4b05-9933-681b2eda1077)

https://github.com/user-attachments/assets/233860e3-4b56-469d-b9ab-14c86bee6160


This is the link for [solution to exercise 2](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-2-rock-paper-scissors-game)

## Braccio

### Exercise 1: Pick-and-Place

Create a shape to control the braccio to pick an object from a known position and place it in another location.

Please use a small lightweight object (e.g., a block or toy) placed at a fixed position.

https://github.com/user-attachments/assets/7853b4ae-a6f3-4de4-8f6e-1e874ca41bc5

This is the link for [solution to exercise 1](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-1-pick-and-place)

### Exercise 2: Avoiding Virtual Barriers

Move the arm through a predefined path while "avoiding" imaginary obstacles.

https://github.com/user-attachments/assets/d324c4bc-504c-4c95-9cb3-1582baf7c272

Hints:
Try placing obstacles near the Braccio device, then use the Move block to navigate the arm without collisions. This is a great way to test precision and control!


This is the link for [solution to exercise 2](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-2-avoiding-virtual-barriers)

## Iron-Boy

### Exercise 1: Command loop

Use the Iron boy command block to make Iron-boy move. Try with something simple like "wave" or "celebrate". the movement will be repeated as long as the 
program is running.


here's the solution: https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-1-command-loop

### Exercise 2: First Steps

Try creating a Shape connecting the TSKIN gestures to Iron-Boy commands. Every time you make a gesture or touch, make it walk forward.


https://github.com/user-attachments/assets/471cfa35-ce89-4fc5-ae72-1c927355cfcd


here's the solution: https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-2-first-steps

## Zion

Prerequisites

1. Please make sure to log in to the Zion platform before starting any exercises related to Zion.
   - You can follow this link for a step-by-step guide on [How to Log in to Zion](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#how-to-login-to-zion-cloud) 
2. You have to connect with the Braccio device [How to connect the Braccio device](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#braccio)

### Exercise 1: Get device's last telemetry data

You need to have the Ultrasonic Distance Level Sensor for this exercise. Make sure that this sensor is connected to Zion via the gateway. If you have any questions, please contact one of our Tactigon crew members.

1. Get **distance** value from the **NI310_UDL**.
2. Get the value and store it in a variable
3. Check if the **distance** is **less than 100**. If so, please print **'Anomaly Detected!!!'** in the console.

Hints:

* Use [get device last telemetry block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-last-telemetry) from Zion Api category
* Select the right device. In this exercise, you can select the NI310_UDL device
* To test, you can place one of the objects very close to the Ultrasonic Distance Level Sensor.

![image](https://github.com/user-attachments/assets/98764469-7b5d-47f1-9921-ca718a04872b)

https://github.com/user-attachments/assets/42255a8a-24a1-4aeb-a6e5-75289d18a97c

This is the link for [solution to exercise 1](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-1-get-devices-last-telemetry-data)

### Exercise 2: Update device's last telemetry data

Now let’s move on to a very interesting exercise. Your goal is to send some data to Zion — easy peasy!

For this exercise, you'll need our Braccio device.

The overall goal of this function is to send a move command to the Braccio, and once the arm moves, it sends telemetry data to Zion to update the status of the device (moving or not).

As you can see, there's a Move block that controls the Braccio's movement. While the Braccio is moving, you can send data to Zion — for example, setting  **movement** to true. But as soon as Braccio is stopped, you need to make another request to Zion by setting  **movement** to false.

You can use following blocks related to Braccio but for this exercise, you can use Move block

![image](https://github.com/user-attachments/assets/50ea9f94-1e78-452b-8534-57a9bd46a729)

https://github.com/user-attachments/assets/1e269973-e608-44e0-9318-f84d1dfc19c0

This is the link for [solution to exercise 2](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-2-update-devices-last-telemetry-data)

### Exercise 3: Get device's attribute data

It's time to retrieve some attribute data from a specific device. Let's use Tiltmeter for this exercise.  

As you can see, we have attributes like angle_x, angle_y, and angle_z. Your goal is to fetch these attributes and print them to the console.

![image](https://github.com/user-attachments/assets/808ffaad-b565-4959-8b6f-869e618b2974)

**In this example, your output should be x = 20, y = 8 and z = 1.2**

Hints:

* Use [get device attribute block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-attribute) from Zion Api category
* Select the right device. In this exercise, you can select the NI320_TILT_1device 
* To test, you can check the current values for angle_x, angle_y, and angle_z in the Zion web application and compare the values with the console or output. (change the position of the Tiltmeter so you can see different output)

https://github.com/user-attachments/assets/12a0f951-3021-4abe-a4c2-8942fc66714a

This is the link for [solution to exercise 3](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-3-get-devices-attribute-data)

### Exercise 4: Update the device's attribute data 

Now, let’s say that for every movement of the Braccio device, we want to update all the values — x, y, and z.

The reason for this is simple: by constantly updating these values, we always have the latest position of the Braccio, and we know exactly where it is at any given moment.

For this exercise, you'll need our Braccio device.

As we mentioned in Exercise 1, when you move the Braccio device, you can send all the values related to position, for example, the value_x, value_y, and value_z values. 

![image](https://github.com/user-attachments/assets/687e1df5-eeb9-4132-8154-b5c61a4e98aa)

As you can see, there's a Move block that controls the Braccio's movement. You pass some numbers or values to the Braccio to define its motion, right?

While the Braccio is moving, you can also send data to Zion — for example:
value_x = 50, value_y = 50, value_z = 20.

This is a great exercise to both control the Braccio and update attribute data with new values.

https://github.com/user-attachments/assets/db0b5c0f-9392-4c6f-9169-5c0e539e4e9f

This is the link for [solution to exercise 4](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-4-update-the-devices-attribute-data)

### Exercise 5: Check the alarm status of a device 

Now, let's move on to alarms. Suppose we have some sensors that measure certain conditions. In this case, we can use the Get Alarm Device block to check if there's an alarm or not.

In this exercise, you can select any device you want and use it, and you can use the [Is Empry block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#is-empty) to check whether an alarm is triggered.

Hints:

* You can use one of our sensors. For example, we have the Ultrasonic Distance Level sensor to measure distance. Use the Zion web app to configure alarm settings.
* To test, you need to check if the response array is empty. If it's not empty, it means we have a bomb.


https://github.com/user-attachments/assets/c59e19b9-fd74-4fc1-aad2-4df1768bed7b

This is the link for [solution to exercise 5](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-5-check-the-alarm-status-of-a-device)

### Exercise 6: Update the alarm status of a device 

Now, let's say we want to send or trigger an alarm. You can use the Create or Update Alarm Device block. Select the device you want to send the alarm from and set the alarm status.

Once you send the alarm, you'll be able to view it through the Zion web app.

![image](https://github.com/user-attachments/assets/663d03b2-657a-40e1-862d-4b54e75fe94b)


https://github.com/user-attachments/assets/f0bf208c-c242-42a4-a34b-d364c1262665

This is the link for [solution to exercise 6](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Solutions#exercise-6-update-the-alarm-status-of-a-device)

## Advance Exercises

### Exercise 1 - Temperature Alert System

Use the temperature sensor (NI300_TH) to trigger Braccio's gripper

Challenge: If the temperature exceeds 30 celsius, open gripper; if below 30 celsius, close gripper
( 30 celsius is an example; you can define your own threshold)

Hints:

* Use [get device last telemetry block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-last-telemetry) from Zion Api category
* Select the right device. In this exercise, you can select the NI300_TH device
* This challenge showcases how robots can interact with their environment by using sensor data to make real-time decisions.

### Exercise 2 - Tilt-Controlled Safety Stop

Use tiltmeter (angle_x) to implement emergency stop

Challenge: If tilt angle_x is positive, then move Braccio to (100,100,50) safety position

Hints:

* Use [get device last telemetry block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-last-telemetry) from Zion Api category
* Select the right device. In this exercise, you can select the NI320_TILT_1 device
* This exercise is very useful because it adds a safety feature, letting the robot detect instability and react to prevent damage or accidents.

### Exercise 3 - Proximity-Controlled Arm

Use distance sensor and active server attribute to control Braccio's Z-axis height

When an object is close to the Ultrasonic Distance Level sensor and the sensor's server attribute is set to active: true, move the Z-axis to a lower position; otherwise, move it to a higher position.

In this screenshot, you can see that there is an edit icon on the right side along with the attributes. You can simply edit the value.

![image](https://github.com/user-attachments/assets/90c62956-d1ab-4eb1-b558-d7322fbb1364)

Hints:

* Use [get device last telemetry block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-last-telemetry) from Zion Api category
* Use [get device attribute block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-attribute) from Zion Api category
* Select the right device. In this exercise, you can select the NI310_UDL device
* You can change the value of the active using the Zion web application.

### Exercise 4 - Auto-Leveling Platform

Use tiltmeter data and all three axes to keep a platform level

Challenge: Continuously move Braccio with comapring angle_x, angle_y, and angle_z values of Tiltmeter. (You can map values from Tiltmeter to Braccio)

Hints:

* Use [get device last telemetry block](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Docs#get-device-last-telemetry) from Zion Api category
* Select the right device. In this exercise, you can select the NI320_TILT_1 device
* To test, you can rotate or change the position of the tiltmeter device. Once it changes, the Braccio device should move accordenly
