FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04
ARG DEBIAN_FRONTEND=noninteractive

# base tools
RUN apt update && apt install -y \
    build-essential vim curl git wget zip \
    imagemagick ffmpeg

# pip3
RUN apt update && apt install -y python3-pip && \
    pip3 install --upgrade pip

# install pytorch
RUN pip3 install torch==2.0.0+cu118 torchvision==0.15.1+cu118 torchaudio==2.0.1 --index-url https://download.pytorch.org/whl/cu118

# Other Python packages
RUN pip3 install -r requirements.txt

# clean
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /root