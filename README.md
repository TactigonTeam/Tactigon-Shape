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

> **DEVELOPER REVIEW REQUIRED BEFORE MERGE**
>
> The first-run procedure must be verified on a clean Ubuntu installation.
> Keep only the validated procedure in the published README.
> See the pull request checklist for the two options to test.

### Option A — Direct start

From the repository directory:

```bash
bash start.sh
```

### Option B — Configure the graphical session and start

Check whether `~/.xprofile` already exists before copying the file.
If it exists, merge the required settings instead of overwriting it.

```bash
cp .xprofile ~/.xprofile
bash start.sh
```

> **TO VERIFY:** whether copying `.xprofile` is necessary and whether
> signing out and back in is required before starting SHAPE.

Once the application is running, open:

http://localhost:5123

> **TO VERIFY:** confirm this address as part of the clean-install test.

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
