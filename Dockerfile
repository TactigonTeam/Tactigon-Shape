FROM python:3.8-slim


RUN apt-get update && apt-get install -y \
    bluetooth \
    bluez \
    libbluetooth-dev \
    pkg-config \
    build-essential \
    libportaudio2 portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

WORKDIR /app

COPY . /app

RUN pip install \
    flask==3.0.3 \
    flask_socketio==5.3.6 \
    gevent==24.2.1 \
    tactigon_gear==5.3.1 \
    PyAudio==0.2.13 \
    pynput==1.7.7 \
    sympy==1.13.2 \
    tactigon_ironboy==1.0.0 && \
    pip install deepspeech-tflite==0.9.3 --no-deps && \
    pip install tactigon_speech==5.0.10 --no-deps


EXPOSE 5123

CMD ["python", "main.py"]
