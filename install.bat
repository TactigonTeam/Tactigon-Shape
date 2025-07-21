py -3.8 -m venv venv
call .\venv\Scripts\activate
pip install flask==3.0.3 flask_socketio==5.3.6 gevent==24.2.1 tactigon_gear==5.2.0 PyAudio==0.2.13 pynput==1.7.7 sympy==1.13.2 tactigon_ironboy==1.0.0 httpx==0.28.1
pip install deepspeech-tflite==0.9.3 --no-deps
pip install tactigon_speech==5.0.8.post1 --no-deps
pause
