# SHAPE — Visual programming for robotics and IoT

Build robotics and IoT applications by connecting visual blocks.
Bring together ROS 2, MQTT, REST APIs and local AI through Ollama.

SHAPE is built on Google Blockly and runs locally on compatible hardware.
Download it, explore the examples and build your first project.

## Start here

- [Installation and prerequisites](https://github.com/TactigonTeam/Tactigon-Shape/wiki/Prerequisites-and-installing)
- [Documentation and tutorials](https://github.com/TactigonTeam/Tactigon-Shape/wiki)
- [Report an issue](https://github.com/TactigonTeam/Tactigon-Shape/issues)

## Quick start

Before running SHAPE on Linux, ensure your host system is ready. You can test your environment by running these commands in your terminal:

- git --version (verifies Git is installed)
- docker info (verifies Docker is running and you have correct user permissions without sudo)
- docker compose version (verifies the Compose plugin is available)
- xhost (verifies the X server access control tool is installed)
First, clone the repository and navigate into it:

```bash
git clone https://github.com/TactigonTeam/Tactigon-Shape.git
cd Tactigon-Shape
```

On Linux systems, you must configure your graphical session before starting SHAPE. This grants the Docker container access to your host's display window. To preserve any existing settings you might already have in your user profile, append the configuration instead of overwriting the file.

Run the following from the repository directory:

```bash
cat .xprofile >> ~/.xprofile
bash start.sh
```
Once the application is running, open your browser and navigate to:

http://localhost:5123

## What you can build

- Connect MQTT messages to ROS 2 applications.
- Use local language models through Ollama.
- Connect applications and services through REST APIs.
- Add voice and gesture interaction with Tactigon GEAR.

## Tutorials and examples

Explore the [Wiki](https://github.com/TactigonTeam/Tactigon-Shape/wiki)
for installation instructions, tutorials and exercises covering MQTT,
ROS 2, Ollama and supported hardware.

## Feedback and contributions

Found a bug or an unclear installation step?
[Open an issue](https://github.com/TactigonTeam/Tactigon-Shape/issues)
and include your operating system, installation method, steps to
reproduce the problem and relevant error messages.

Want to contribute? Explore the
[contribution guides](https://github.com/TactigonTeam/Tactigon-Shape/wiki)
to learn how to create custom blocks and modules.

## Related packages

- [Tactigon GEAR](https://pypi.org/project/tactigon-gear/)
- [Tactigon Speech](https://pypi.org/project/tactigon-speech/)

## License

SHAPE is distributed under the Apache License 2.0.
See [LICENSE](LICENSE) for details.
