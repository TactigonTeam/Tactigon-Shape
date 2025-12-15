sudo apt install ros-jazzy-camera-ros

sudo apt update
sudo apt install portaudio19-dev python3-pyaudio

source /opt/ros/jazzy/setup.bash
python3 -m venv ros2env --system-site-packages
source ./ros2env/bin/activate
pip install flask==3.1.2 flask_socketio==5.5.1 gevent==25.9.1 gevent-websocket==0.10.1 PyAudio==0.2.14 pynput==1.8.1 tactigon_ironboy==1.0.0 paho-mqtt==2.1.0 httpx==0.28.1
pip install tactigon_gear==5.4.2