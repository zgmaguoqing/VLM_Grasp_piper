FROM 7d79b4fee201

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV http_proxy="http://10.5.20.132:19998"
ENV https_proxy="http://10.5.20.132:19998"

# 同时保留大写版本以实现最佳兼容性
ENV HTTP_PROXY="http://10.5.20.132:19998"
ENV HTTPS_PROXY="http://10.5.20.132:19998"
# 1. 配置清华源 & APT 极限容错 (内容保持不变，利用缓存)
RUN echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list

RUN echo 'Acquire::Retries "100";' > /etc/apt/apt.conf.d/80-retries && \
    echo 'Acquire::http::Timeout "300";' >> /etc/apt/apt.conf.d/80-retries && \
    echo 'Acquire::https::Timeout "300";' >> /etc/apt/apt.conf.d/80-retries

# 2. 基础工具 (内容保持不变，利用缓存)
RUN rm -rf /var/lib/apt/lists/* && apt-get clean && apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    ca-certificates \
    locales \
    git \
    vim \
    net-tools \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 3. 添加 ROS 源 & 构建工具 (内容保持不变，利用缓存)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg || \
    curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    libboost-program-options-dev

# 4. 切片安装依赖 (内容保持不变，利用缓存)
RUN apt-get install -y --no-install-recommends --fix-missing \
    ros-humble-ros-core \
    ros-humble-ros-base

RUN apt-get install -y --no-install-recommends --fix-missing \
    libxinerama1 \
    libgtk-3-0 \
    libopencv-core4.5d \
    libopencv-imgcodecs4.5d \
    libopencv-dev \
    python3-opencv

RUN apt-get install -y --no-install-recommends --fix-missing \
    libgazebo11 \
    libgazebo-dev \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

RUN apt-get install -y --no-install-recommends --fix-missing \
    ros-humble-desktop \
    ros-humble-xacro \
    ros-humble-moveit \
    mesa-utils \
    libgl1-mesa-glx

# 5. 项目构建 (Git Clone 等步骤保持不变，利用缓存)
RUN rosdep init && rosdep update || echo "rosdep failed but continuing"

WORKDIR /root/ros2_ws/src

RUN git clone https://github.com/agilexrobotics/piper_ros2.git || \
    git clone -b humble https://github.com/agilexrobotics/piper_ros.git

WORKDIR /root/VLM_Grasp_Interactive
COPY . /root/VLM_Grasp_Interactive

RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt || echo "No requirements.txt"
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple transforms3d

WORKDIR /root/ros2_ws

# =========================================================
# 【这里是关键补丁】
# 之前报错是因为没有 rosdep 命令。
# 我们在这里插入一个新层，专门安装 python3-rosdep 和 rosdepc
# 因为这行是在最后，前面的几百 MB 下载都不会受影响，直接 Cache 命中。
# =========================================================
RUN apt-get update && apt-get install -y python3-rosdep && \
    pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc && \
    rosdepc init && \
    rosdepc update

# 【修改】使用 rosdepc install 替代 rosdep install
# rosdepc 是国内特供版，不会因为连接 github 失败而报错
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdepc install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --parallel-workers 1"

# 6. 环境配置
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/ros2_ws/src/piper_ros2:/root/ros2_ws/src/piper_ros" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]