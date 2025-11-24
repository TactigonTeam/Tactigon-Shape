# Tactigon Shapes

### 10/11/25
software name: Tactigon Shapes
project name: ONI107SHAP01
release code: 5.2.0.3

## REQUIREMENTS

**Linux**
- ubuntu 24.04
- ROS Jazzy
- 8 GB RAM

## DEPENDENCIES
- Tactigon Skin firmware 5.0.7.1
- Tactigon Gear 5.3.1

## TESTFILES
- TESTBOOK_tactigonShape_V5

## FEATURES
release code: 5.2.0.3
### ROS Blocks
- Publishing: The application functions as a Node, broadcasting serialized data messages over named communication buses called Topics.
- Receiving: The application implements subscriber logic to listen to specific Topics, capturing and deserializing messages broadcast by other Nodes.

### MQTT Blocks
- Publishing: The application connects to a central Broker and transmits data payloads to specific identification strings called Topics.
- Receiving: The application subscribes to specific Topics on the Broker to intercept and process data routed from other clients.

### Ginos Blocks
"Ginos" represents the connection between Shapes ans AI.
Thanks to Ginos it's possible to create AI Prompts directly from the "edit shape" menu, thanks to the "AI prompt" block.

