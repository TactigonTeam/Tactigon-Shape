# ============================================================
#  SUNRISE – Docker Image
#  Base: ROS 2 Jazzy (Ubuntu 24.04)
# ============================================================
FROM eprosima/vulcanexus:jazzy-base

# ── System dependencies ──────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-usb-cam \
    ros-jazzy-web-video-server \
    python3-opencv \
    python3-pip \
    v4l-utils \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ──────────────────────────────────────────
RUN pip install --no-cache-dir --break-system-packages "numpy<2.0.0" mediapipe

# ── Create workspace ─────────────────────────────────────────
WORKDIR /app

# Scarica il modello hand_landmarker e lo salva come hand_landmarker.task in /app
RUN wget -qO /app/hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

COPY ./nodes/camera_tracking /app/camera_tracking
COPY ./msgs/camera_tracking_msgs /app/msgs/camera_tracking_msgs


RUN chmod +x /app/camera_tracking/entrypoint.sh

SHELL ["/bin/bash", "-c"]
RUN source /opt/vulcanexus/jazzy/setup.bash && \
    # source ./install/setup.bash && \
    colcon build --packages-select camera_tracking_msgs && \
    colcon build --packages-select camera_tracking
    #colcon build

EXPOSE 8081

ENTRYPOINT ["/app/camera_tracking/entrypoint.sh"]
