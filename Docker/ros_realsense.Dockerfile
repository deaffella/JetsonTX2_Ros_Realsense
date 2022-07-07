FROM dustynv/ros:noetic-ros-base-l4t-r32.7.1
MAINTAINER Letenkov Maksim <letenkovmaksim@yandex.ru>

SHELL ["/bin/bash", "-c"]


RUN apt-get update
RUN apt-get install -y \
    usbutils \
    cmake \
    libgtk2.0-dev \
    nano \
    curl \
    libssl-dev \
    libcurl4-openssl-dev \
    g++ \
    libhdf5-serial-dev \
    hdf5-tools \
    libhdf5-dev \
    zlib1g-dev \
    zip \
    libjpeg8-dev \
    liblapack-dev \
    libblas-dev \
    gfortran \
    gedit \
    git \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libudev-dev python3-pip python3-opencv
RUN apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install libxinerama-dev libsdl2-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN pip3 install tqdm transliterate


WORKDIR /
RUN mkdir /installed_deps
WORKDIR /installed_deps

RUN git clone https://github.com/IntelRealSense/librealsense
WORKDIR /installed_deps/librealsense

RUN mkdir build
WORKDIR /installed_deps/librealsense/build

RUN cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=/usr/bin/python3
RUN make -j4
RUN make install

ENV  PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2



RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/ros-drivers/rosserial.git -b noetic-devel
WORKDIR /catkin_ws/

RUN source /ros_entrypoint.sh
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make && \
    catkin_make install



RUN source /catkin_ws/devel/setup.bash





RUN apt update && \
    apt install -y python3-venv tmux
RUN pip3 install pyserial


RUN mkdir /some_temp_dir
WORKDIR /some_temp_dir


RUN cd /some_temp_dir && \
    git clone https://github.com/ros/angles && \
    mv angles/angles /catkin_ws/src/
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="angles"

RUN cd /some_temp_dir && \
    git clone https://github.com/ros/geometry2 && \
    mv geometry2 /catkin_ws/src/
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="geometry2"

RUN cd /some_temp_dir && \
    git clone https://github.com/ros/geometry && \
    mv geometry /catkin_ws/src/
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="geometry"

RUN cd /some_temp_dir && \
    git clone https://github.com/ros/roslint && \
    mv roslint /catkin_ws/src/
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="roslint"

RUN cd /some_temp_dir && \
    git clone https://github.com/locusrobotics/catkin_virtualenv && \
    mv catkin_virtualenv /catkin_ws/src/
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="catkin_virtualenv"



RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="tf2_msgs"
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="tf2"
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="tf2_py"
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="tf2_ros"
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="tf"

RUN cd /some_temp_dir && \
    git clone https://github.com/ros-drivers/nmea_navsat_driver && \
    mv nmea_navsat_driver /catkin_ws/src/
RUN cd /catkin_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="nmea_navsat_driver"


RUN source /catkin_ws/devel/setup.bash
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /mount_dir/
CMD ["bash"]
