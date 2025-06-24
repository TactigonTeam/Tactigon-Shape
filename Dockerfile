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

RUN pip install -r docker-requirements.txt

RUN pip install deepspeech-tflite==0.9.3 --no-deps
RUN pip install tactigon_speech==5.0.10 --no-deps

EXPOSE 5123

CMD ["python", "main.py"]
