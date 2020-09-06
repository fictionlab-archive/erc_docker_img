FROM ros:melodic

# These values will be overrided by `docker run --env <key>=<value>` command
ENV ROS_IP 127.0.0.1
ENV ROS_MASTER_URI http://127.0.0.1:11311

# Install some basic dependencies
RUN apt-get update && apt-get -y upgrade && apt-get -y install \
  curl ssh \
  ros-melodic-cv-bridge \
  ros-melodic-tf \
  python-pip python3-pip \
  && rm -rf /var/lib/apt/lists/*

# Set root password
RUN echo 'root:root' | chpasswd

# Permit SSH root login
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# Install Freedom agent
ARG FREEDOM_URL
RUN curl -sSf $FREEDOM_URL | \
  sed 's:a/nmkK3DkqZEB/ngrok-2.2.8-linux-arm64.zip:c/4VmDzA7iaHb/ngrok-stable-linux-arm64.zip:' | python \
  && rm -rf /var/lib/apt/lists/* \
  && rm -rf /root/.cache/pip/* 

# Install catkin-tools
RUN apt-get update && apt-get install -y python-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

# Copy packages and build the workspace
WORKDIR /catkin_ws
COPY src ./src
RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/*
RUN catkin config --extend /opt/ros/melodic && catkin build

COPY start.sh /

ENTRYPOINT []
CMD ["/start.sh"]
