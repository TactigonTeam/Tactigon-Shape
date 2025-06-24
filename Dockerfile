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

EXPOSE 5123

CMD ["python", "main.py"]
