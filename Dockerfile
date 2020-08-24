FROM ros:melodic

RUN apt-get update && apt-get -y upgrade && apt-get -y install curl ssh

RUN echo 'root:root' | chpasswd
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

ARG FREEDOM_URL
RUN curl -sSf $FREEDOM_URL | python

CMD ["python", "/root/.local/lib/python2.7/site-packages/freedomrobotics/agent.py"]