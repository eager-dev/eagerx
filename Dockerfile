# Adapted from DLR-RM/stable-baselines3

ARG PARENT_IMAGE
FROM $PARENT_IMAGE
ARG PYTORCH_DEPS=cpuonly
ARG POETRY_VERSION=1.0.0
ARG ADD_SB=False
ARG PYTHON_VERSION=3.8

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y --no-install-recommends apt-utils \
  build-essential \
  cmake \
  git \
  curl \
  ca-certificates \
  libjpeg-dev \
  libpng-dev \
  libglib2.0-0 \
  apt-transport-https \
  gnupg-agent \
  python3-pip \
  gnupg \
  software-properties-common &&\
  rm -rf /var/lib/apt/lists/*

# Install Anaconda and dependencies for using the specified python version
RUN curl -o ~/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
     chmod +x ~/miniconda.sh && \
     ~/miniconda.sh -b -p /opt/conda && \
     rm ~/miniconda.sh && \
     /opt/conda/bin/conda install -y python=$PYTHON_VERSION && \
     if [ ${ADD_SB} ] ; then /opt/conda/bin/conda install -y pytorch $PYTORCH_DEPS -c pytorch; fi && \
     /opt/conda/bin/conda clean -ya
ENV PATH /opt/conda/bin:$PATH

SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]

# System deps:
RUN pip3 install --upgrade pip
RUN pip3 install "poetry==$POETRY_VERSION"

# When installing stable-baselines3, we want to use the appropriate pytorch version (cpu or gpu)
# if [ ${ADD_SB} ] ; then /opt/conda/bin/conda install -y pytorch $PYTORCH_DEPS -c pytorch; fi && \

# Copy only requirements to cache them in docker layer
WORKDIR /code
COPY poetry.lock pyproject.toml /code/

# Install eagerx without creation of poetry environment since the docker is already isolated
RUN poetry config virtualenvs.create false && poetry install --no-interaction --no-ansi

# Creating folders, and files for a project:
COPY . /code

# Install ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros-testing/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-ros-base ros-noetic-cv-bridge
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

# Install eagerx-tutorials if ADD_SB, this will also install stable-baselines3
RUN if [ ${ADD_SB} ] ; then pip install eagerx-tutorials; fi

# Use headless opencv
RUN pip uninstall -y opencv-python && pip install opencv-python-headless && rm -rf $HOME/.cache/pip
