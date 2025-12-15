FROM python:3.8-slim

RUN apt-get update && apt-get install -y \
    bluetooth \
    bluez \
    libbluetooth-dev \
    pkg-config \
    build-essential \
    libportaudio2 portaudio19-dev \
    xvfb xserver-xorg-core libice6 libxrender1 libfontconfig1 libglib2.0-0 \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

WORKDIR /app

COPY config /app/config
COPY models /app/models
COPY speech /app/speech
COPY tactigon_shapes /app/tactigon_shapes
COPY main.py /app/main.py
COPY docs /docs

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
    pip install tactigon_speech==5.0.10 --no-deps \
    paho-mqtt \
    httpx \
    argparse \
    uuid \
    anyio \
    exceptiongroup \
    sniffio \
    httpcore \
    langchain-community langchain pypdf chromadb \
    python-docx

EXPOSE 5123

HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 CMD curl -f http://localhost:5123/ || exit 1

CMD ["python", "-m", "tactigon_shapes", "--address=0.0.0.0", "--port=5123"]
