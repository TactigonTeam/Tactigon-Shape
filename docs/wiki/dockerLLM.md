
# Initial System Boot Sequence

this doc is to guide the user into creating a two-container network to work with Shapes and Ollama
## Network Creation
The network provides a link between the two dockers; use:

docker network create ollama-net

## Start Ollama container (Terminal 1)
download the container of Ollama with 

docker run -d \
  -v ollama:/root/.ollama \
  --name ollama \
  -p 11434:11434 \
  --network ollama-net \ 
  ollama/ollama

This activates the container in the background. To directly test the functionality of a model or download a new one, use:

docker exec -it ollama ollama run <model-name>
docker exec -it ollama ollama run tinyllama

If you want to implement the reading part of a document (e.g., Word), you need to use a mini model called nomic
which works alongside the actual model that evaluates the document. Nomic indexes the parts of the document (chunks) so that it can be read by the real model.

docker exec -it ollama ollama pull nomic-embed-text

From here, the download will start or the chat with the model will open.

## build docker file Shape
To build the image from docker file, use:

docker build -t tactigon-shapes .


## Start Tactigon Shapes (Terminal 2)

This command starts the application container, connecting it to the same network (ollama-net) to communicate with the Ollama server. It also includes the necessary configurations for the graphical interface (X server) on Linux (-e DISPLAY, -v /tmp/.X11-unix, etc.).

docker run \
  --name tactigon-shapes \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -p 5123:5123 \
  -v tactigon-config:/app/config \
  -v /var/run/dbus:/var/run/dbus \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e SDL_AUDIODRIVER=dummy \
  --network ollama-net \
  tactigon-shapes

In debug mode, add -v $(pwd):/app \

## Connection Test

After activating the two containers, simply create a shape with the debug block -> "ginos ai prompt" and a prompt such as 
"why is the sky blue?"
By activating the shape we will have to wait a few seconds for the response to be printed to the console

