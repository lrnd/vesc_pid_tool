FROM ubuntu:20.04

#TODO: see timezone env var
ENV TZ=Australia/Sydney
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
#TODO: set locale
ENV LANG en_AU.UTF-8

#TODO need boost libraries to build here and that means needed to run it too...

RUN apt-get update && apt-get install -y \
    curl \
    git \
    gnupg \
    lsb-release \
    build-essential \
    pkg-config \
    libgtk-3-dev \
    net-tools \
    && rm -rf /var/lib/apt/lists/*
