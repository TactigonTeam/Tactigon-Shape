# Tactigon Shapes

Software name: Tactigon Shapes
Project name: ONI107SHAP01
Release code: 5.2.0.4-rc2

## REQUIREMENTS

**Linux**
- x86-64 CPU with AVX/FMA instructions
- Ubuntu 24.04
- Docker linux
- 8 GB RAM

## DEPENDENCIES
- Tactigon Skin firmware 5.0.7.1
- Tactigon Gear 5.3.1
- Tactigon Speech 5.0.10

## TESTFILES
- TESTBOOK_tactigonShape_V5

## CHANGELOG

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
