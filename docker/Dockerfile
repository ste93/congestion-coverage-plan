FROM ros:jazzy
ENV  DOCKER_BUILDKIT=1
ARG BRANCH=master
ARG REPO_USERNAME=convince-project


# Install dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    git \
    vim \
    build-essential 


ENV DEBIAN_FRONTEND=noninteractive \
    DOCKER_UPDATED_ON=06_06_2024
USER root
# download public key for github.com
# RUN useradd -ms /bin/bash user && echo "user:user" | chpasswd && adduser user sudo
# USER user
# install lkh solver
RUN apt-get update && apt-get install -y \
    wget \
    make \
    gcc \
    libc6-dev \
    && rm -rf /var/lib/apt/lists/*

# download and compile LKH-3
WORKDIR /app
RUN wget http://www.akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz \
    && tar xvf LKH-3.0.6.tgz \
    && cd LKH-3.0.6 \
    # apply patch to fix compilation with modern gcc 
    && sed -i 's/CFLAGS =/CFLAGS = -fcommon /' SRC/Makefile \
    && make

# make it available globally
RUN ln -s /app/LKH-3.0.6/LKH /usr/local/bin/lkh
RUN git clone --branch ${BRANCH} https://github.com/${REPO_USERNAME}/congestion-coverage-plan.git

WORKDIR /app/congestion-coverage-plan
RUN bash -c "python3 -m venv venv_congestion_coverage_plan && \
    . venv_congestion_coverage_plan/bin/activate && \
    pip install --upgrade pip && \
    pip install tqdm && \
    pip install lkh && \
    pip install -e ."
# Set the entrypoint to the Python interpreter

RUN echo "source /app/congestion-coverage-plan/venv_congestion_coverage_plan/bin/activate" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd interfaces && colcon build"
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /app/congestion-coverage-plan/interfaces/install/setup.bash" >> ~/.bashrc
# add new user and add it to sudo group
WORKDIR /app/congestion-coverage-plan


CMD ["bash"]
