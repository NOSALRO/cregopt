FROM --platform=x86_64 ubuntu:22.04

ARG USERNAME="robot"
ARG HOME_DIR="/home/${USERNAME}"

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Athens"

# Installation of Ubuntu packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y python3-dev python-is-python3 python3-pip git cmake build-essential unzip wget libx11-dev python3-tk tk-dev g++ gcc x11-apps nano libeigen3-dev pybind11-dev libblas-dev liblapack-dev libmetis-dev gfortran libboost-all-dev libtbb-dev software-properties-common gdb && \
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

# Add robot_dart for visualizations
RUN     apt install -y software-properties-common \
    build-essential \
    cmake \
    pkg-config \
    git \
    python-is-python3 \
    python3-setuptools \
    python3-numpy \
    python3-dartpy \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-test-dev \
    libdart-all-dev \
    libxi-dev \
    libxmu-dev \
    freeglut3-dev \
    libopenscenegraph-dev \
    libassimp-dev \
    pybind11-dev \
    libopenal-dev \
    libglfw3-dev \
    libsdl2-dev \
    libopenexr-dev \
    libdevil-dev \
    libpng-dev \
    libfaad-dev \
    libfreetype6-dev \
    libglm-dev

RUN git clone https://github.com/resibots/robot_dart.git && cd robot_dart && \
    mkdir -p temp_robot_dart && \
    cd temp_robot_dart && \
    git clone https://github.com/mosra/corrade.git && \
    cd corrade && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/magnum .. && \
    make -j8 && \
    make install && \
    cd ../.. && \
    git clone https://github.com/mosra/magnum.git && \
    cd magnum && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/magnum -DMAGNUM_WITH_AUDIO=ON -DMAGNUM_WITH_DEBUGTOOLS=ON -DMAGNUM_WITH_GL=ON -DMAGNUM_WITH_MESHTOOLS=ON -DMAGNUM_WITH_PRIMITIVES=ON -DMAGNUM_WITH_SCENEGRAPH=ON -DMAGNUM_WITH_SHADERS=ON -DMAGNUM_WITH_TEXT=ON -DMAGNUM_WITH_TEXTURETOOLS=ON -DMAGNUM_WITH_TRADE=ON -DMAGNUM_WITH_GLFWAPPLICATION=ON -DMAGNUM_WITH_WINDOWLESSGLXAPPLICATION=ON  -DMAGNUM_WITH_WINDOWLESSEGLAPPLICATION=ON -DMAGNUM_WITH_OPENGLTESTER=ON -DMAGNUM_WITH_ANYAUDIOIMPORTER=ON -DMAGNUM_WITH_ANYIMAGECONVERTER=ON -DMAGNUM_WITH_ANYIMAGEIMPORTER=ON -DMAGNUM_WITH_ANYSCENEIMPORTER=ON -DMAGNUM_WITH_MAGNUMFONT=ON -DMAGNUM_WITH_OBJIMPORTER=ON -DMAGNUM_WITH_TGAIMPORTER=ON -DMAGNUM_WITH_WAVAUDIOIMPORTER=ON -DMAGNUM_WITH_GL_INFO=ON -DMAGNUM_WITH_AL_INFO=ON -DMAGNUM_TARGET_EGL=ON .. && \
    make -j8 && \
    make install && \
    cd ../.. && \
    git clone https://github.com/mosra/magnum-plugins.git && \
    cd magnum-plugins && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/magnum -DMAGNUM_WITH_ASSIMPIMPORTER=ON -DMAGNUM_WITH_DDSIMPORTER=ON -DMAGNUM_WITH_JPEGIMPORTER=ON -DMAGNUM_WITH_OPENGEXIMPORTER=ON -DMAGNUM_WITH_PNGIMPORTER=ON -DMAGNUM_WITH_TINYGLTFIMPORTER=ON -DMAGNUM_WITH_STBTRUETYPEFONT=ON .. && \
    make -j && \
    make install && \
    cd ../.. && \
    git clone https://github.com/mosra/magnum-integration.git && \
    cd magnum-integration && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/magnum -DMAGNUM_WITH_DART=ON -DMAGNUM_WITH_EIGEN=ON -DMAGNUM_WITH_BULLET=ON -DMAGNUM_WITH_GLM=ON .. && \
    make -j && \
    make install && \
    export PATH=/opt/magnum/bin:$PATH && \
    export LD_LIBRARY_PATH=/opt/magnum/lib:$LD_LIBRARY_PATH && \
    cd ../.. && \
    git clone https://github.com/mosra/magnum-bindings.git && \
    cd magnum-bindings && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/magnum -DMAGNUM_WITH_PYTHON=ON .. && \
    # make -j && \
    cd src/python && \
    # python3 setup.py install --root=/opt/magnum/lib --install-purelib=python3/site-packages --install-platlib=python3/site-packages --install-scripts=python3/scripts --install-headers=python3/include --install-data=python3/data && \
    cd ../../../../.. && \
    export PYTHONPATH=/opt/magnum/lib/python3/site-packages:$PYTHONPATH && \
    ./waf configure --prefix /opt/robot_dart --python --magnum /opt/magnum && \
    ./waf -j8 && \
    ./waf examples -j8 && \
    ./waf install

# User stuff
RUN useradd -ms /bin/bash $USERNAME
RUN echo "${USERNAME}:pass" | chpasswd
RUN apt install -y sudo
RUN usermod -aG sudo $USERNAME

RUN echo "export PATH=/opt/magnum/bin:$PATH" >> /home/robot/.bashrc
RUN echo "export CMAKE_PREFIX_PATH=/opt/robot_dart:/opt/magnum" >> /home/robot/.bashrc
RUN echo "export LD_LIBRARY_PATH=/opt/magnum/lib:/opt/robot_dart/lib:$LD_LIBRARY_PATH" >> /home/robot/.bashrc
RUN echo "export PYTHONPATH=/opt/robot_dart/lib/python3.10/site-packages:/opt/magnum/lib/python3/site-packages:$PYTHONPATH" >> /home/robot/.bashrc

USER $USERNAME

# Working directory
WORKDIR ${HOME_DIR}
