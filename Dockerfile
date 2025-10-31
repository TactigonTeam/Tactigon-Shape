FROM python:3.8-slim


RUN apt-get update && apt-get install -y \
    bluetooth \
    bluez \
    libbluetooth-dev \
    pkg-config \
    build-essential \
    libportaudio2 portaudio19-dev \
    xvfb xserver-xorg-core libice6 libxrender1 libfontconfig1 libglib2.0-0 \
    curl\
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
    pip install tactigon_speech==5.0.10 --no-deps \
    paho-mqtt\
    httpx\
    argparse\
    uuid \
    anyio \
    exceptiongroup\
    sniffio



EXPOSE 5123

HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 CMD curl -f http://localhost:5123/ || exit 1

CMD bash -c "xvfb-run --server-args='-screen 0 1024x768x24' python -u main.py --address=0.0.0.0 --port 5123"

#CMD ["xvfb-run", "--server-args", "-screen 0 1024x768x24", "python", "main.py"]
#CMD ["python", "main.py", "--address", "0.0.0.0"]