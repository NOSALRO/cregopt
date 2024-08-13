FROM --platform=x86_64 ubuntu:22.04

ARG USERNAME="robot"
ARG HOME_DIR="/home/${USERNAME}"

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Athens"

# Installation of Ubuntu packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y python3-dev python-is-python3 python3-pip git cmake build-essential unzip wget libx11-dev python3-tk tk-dev g++ gcc x11-apps nano libeigen3-dev pybind11-dev libblas-dev liblapack-dev libmetis-dev gfortran libboost-all-dev libtbb-dev software-properties-common && \
    apt-get clean

# Add the ThirdParty-HSL directory
ADD ThirdParty-HSL /code_libs/ThirdParty-HSL
ADD coinhsl /code_libs/ThirdParty-HSL/coinhsl

RUN cd /code_libs/ThirdParty-HSL && \
    ./configure && \
    make VERBOSE=1 && \
    make VERBOSE=1 install

# Add the ThirdParty-ASL directory
ADD ThirdParty-ASL /code_libs/ThirdParty-ASL

RUN cd /code_libs/ThirdParty-ASL && \
    ./get.ASL && \
    ./configure && \
    make VERBOSE=1 && \
    make VERBOSE=1 install

# Add the ThirdParty-Mumps directory
ADD ThirdParty-Mumps /code_libs/ThirdParty-Mumps

RUN cd /code_libs/ThirdParty-Mumps && \
    ./get.Mumps && \
    ./configure && \
    make VERBOSE=1 && \
    make VERBOSE=1 install

# Add Ipopt directory
ADD Ipopt /code_libs/Ipopt

RUN cd /code_libs/Ipopt && \
    ./configure && \
    make VERBOSE=1 && \
    make VERBOSE=1 install

# Add ifopt
RUN git clone https://github.com/ethz-adrl/ifopt.git && cd ifopt && mkdir build && cd build && cmake .. && make && make install

# Add algevo
RUN apt-get install libtbb-dev
RUN git clone https://github.com/NOSALRO/algevo.git && cd algevo && ./waf configure && ./waf install

# Add towr
RUN git clone https://github.com/itsikelis/towr.git && cd towr/towr && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make && make install

# User stuff
RUN useradd -ms /bin/bash $USERNAME
RUN echo "${USERNAME}:pass" | chpasswd
RUN apt install -y sudo
RUN usermod -aG sudo $USERNAME

USER $USERNAME

# Working directory
WORKDIR ${HOME_DIR}
