# Tactigon Shapes

Software name: Tactigon Shapes
Project name: ONI107SHAP01

## REQUIREMENTS

**Linux**
- x86-64 CPU with AVX/FMA instructions
- Ubuntu 24.04
- Docker linux
- 8 GB RAM

## DEPENDENCIES
- Tactigon Skin firmware 5.0.7.1
- Tactigon Gear 5.5.2
- Tactigon Speech Socket 5.5.0.2

## TESTFILES
- TESTBOOK_tactigonShape_V5

## CHANGELOG

### 29/01/2026
release code: 5.5.0.3

BUGFIX:
- Updated docker-compose to prevent race condition on startup
- Fixed name duplication bug


### 21/01/2026
release code: 5.5.0.2

BUGFIX:
- Added check to handle missing code warning
- Fixed Zion interface to generalize calls and fix bug TCT-109 and TCT-110
- Fixed device_alarm method
- AI chat block append [Object object] on first stream message
- Removed zion default credentials and added check for config validation

### 9/12/2025
release code: 5.5.0.1

- Upgraded TSkin to TSkinSocket in Shape in order to send audio stream over a socket
- Created tactigon-speech-socket docker and added in the compose
- Added open-webui docker to manage Ollama using a web browser

BUGFIX:
- Fixed deadloop on shape stop

### 8/12/2025
release code: 5.5.0.0

- Added Import-Export features

### 8/12/2025
release code: 5.2.0.5

- Added ROS2 compatible blocks

### 16/12/2025
release code: 5.2.0.4-rc2

- Added header to all files
- Created module launcher
- Updated close routine to accomodate Docker
- Added docker compose
- Added github workflows

BUGFIXES:
- Fixed clone shape issue

### 10/11/2025
release code: 5.2.0.4-rc1
- MQTT Blocks
    - Publishing: The application connects to a central Broker and transmits data payloads to specific identification strings called Topics.
    - Receiving: The application subscribes to specific Topics on the Broker to intercept and process data routed from other clients.
- Ginos Blocks: "Ginos" represents the connection between Shapes ans AI. Thanks to Ginos it's possible to create AI Prompts directly from the "edit shape" menu, thanks to the "AI prompt" block.

BUGFIXES
    - end of file error in docker
    - "no connection" error with zion config
    - copiable demo Shapes creating othe readonly Shapes
