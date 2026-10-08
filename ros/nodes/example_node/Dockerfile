FROM eprosima/vulcanexus:jazzy-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./nodes/example_node /app/example_node

RUN chmod +x /app/example_node/entrypoint.sh

SHELL ["/bin/bash", "-c"]
RUN source /opt/vulcanexus/jazzy/setup.bash && \
    colcon build --packages-select example_node

ENTRYPOINT ["/app/example_node/entrypoint.sh"]
