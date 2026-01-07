#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Project Name:
# Tactigon Soul - Shape
# 
# Release date: 30/09/2025
# Release version: 1.0
#
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/


FROM ros:kilted-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install -y \
    python3-pip \
    bluetooth \
    bluez \
    libbluetooth-dev \
    pkg-config \
    build-essential \
    libportaudio2 portaudio19-dev \
    xvfb xserver-xorg-core libice6 libxrender1 libfontconfig1 libglib2.0-0 \
    curl \
    iproute2 \
    net-tools \
    bash \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY config /app/config
COPY models /app/models
COPY speech /app/speech
COPY tactigon_shapes /app/tactigon_shapes
COPY main.py /app/main.py

RUN pip install --no-cache-dir \
    flask==3.0.3 \
    flask-socketio==5.5.1 \
    gevent==24.2.1 \
    gevent-websocket==0.10.1 \
    tactigon-gear==5.5.0 \
    PyAudio==0.2.14 \
    pynput==1.7.7 \
    sympy==1.13.2 \
    tactigon-ironboy==1.0.1 \
    paho-mqtt==2.1.0 \
    httpx

EXPOSE 5123

HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 CMD curl -f http://localhost:5123/ || exit 1

CMD ["python3", "-m", "tactigon_shapes", "--address=0.0.0.0", "--port=5123"]
