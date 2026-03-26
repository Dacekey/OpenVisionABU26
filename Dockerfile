FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-jazzy-vision-msgs \
    ros-jazzy-usb-cam \
    ros-jazzy-cv-bridge \
    libopencv-dev \
    wget \
    tar \
    && rm -rf /var/lib/apt/lists/*

# Install ONNX Runtime GPU package
RUN mkdir -p /opt/onnxruntime && \
    cd /opt/onnxruntime && \
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-gpu-1.18.1.tgz && \
    tar -xzf onnxruntime-linux-x64-gpu-1.18.1.tgz && \
    mv onnxruntime-linux-x64-gpu-1.18.1 onnxruntime && \
    rm onnxruntime-linux-x64-gpu-1.18.1.tgz

ENV ONNXRUNTIME_DIR=/opt/onnxruntime/onnxruntime
ENV LD_LIBRARY_PATH=${ONNXRUNTIME_DIR}/lib:${LD_LIBRARY_PATH}

COPY . /workspace

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build

CMD ["bash"]