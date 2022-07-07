FROM deaffella/ros_realsense
MAINTAINER Letenkov Maksim <letenkovmaksim@yandex.ru>

SHELL ["/bin/bash", "-c"]


RUN apt update && \
    apt install -y libopenblas-dev
WORKDIR /some_temp_dir/

RUN wget -O \
    "torch-1.10.0-cp36-cp36m-linux_aarch64.whl" \
    "https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl"
RUN pip3 install "torch-1.10.0-cp36-cp36m-linux_aarch64.whl"




WORKDIR /mount_dir/
CMD ["bash"]
