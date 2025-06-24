# Tactigon Shapes

Tactigon Shapes is an extension of [Blockly](https://developers.google.com/blockly), which is a project by Google and that allows you to create visual, drag-and-drop block-based programming interfaces. Instead of typing code, you can create programs by connecting blocks together. This README provides an overview of the project, its features, installation instructions, and guidance on contributing.

## Table of Contents
- [Tactigon Shapes](#tactigon-shapes)
  - [Table of Contents](#table-of-contents)
  - [Do you want to know more ?](#do-you-want-to-know-more-)
  - [More about Tactigon packages](#more-about-tactigon-packages)
  - [Tactigon Shape with Docker on Linux](#tactigon-shape-with-docker-on-linux)
    - [Prerequisites](#prerequisites)
    - [Setup Instructions](#setup-instructions)

## Do you want to know more ?

You can access our full documentation, including exercises and additional resources, by clicking the [here](https://github.com/TactigonTeam/Tactigon-Shape/wiki)


## More about Tactigon packages

Below are our most useful Python packages available via pip. Feel free to download them, explore their features, and bring your own ideas to life!

1. [Tactigon Gear](https://pypi.org/project/tactigon-gear/)
2. [Tactigon Speech](https://pypi.org/project/tactigon-speech/)

## Tactigon Shape with Docker on Linux

This guide will help you set up and run the **Tactigon Shape** app using Docker on a Linux system.

### Prerequisites

Make sure you have the following installed:

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

### Setup Instructions

1. **Grant Display Access**

   Since the app uses keyboard input and needs display access, run the following command:

   ```bash
    xhost +local:docker
   ```

2. **Navigate to the Project Root**

    Open a terminal and navigate to the root folder of the Tactigon Shape project.

3. **Build and Run the Docker Container**
    In the project root directory, run:

   ```bash
    docker-compose up --build
   ```
3. **Start the Tactigon App**
    
    Once the container is up and running, you will see a URL in the terminal output. Open this URL in your browser to use the Tactigon Shape application.