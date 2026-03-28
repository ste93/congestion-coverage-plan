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

# clone our private repository
RUN mkdir -p /app
WORKDIR /app

RUN git clone --branch ${BRANCH} https://github.com/${REPO_USERNAME}/congestion-coverage-plan.git

WORKDIR /app/congestion-coverage-plan
RUN bash -c "python3 -m venv venv_congestion_coverage_plan && \
    . venv_congestion_coverage_plan/bin/activate && \
    pip install --upgrade pip && \
    pip install -e ."
# Set the entrypoint to the Python interpreter

RUN echo "source /app/congestion-coverage-plan/venv_congestion_coverage_plan/bin/activate" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --base-paths static_devices_msgs"
RUN echo "source /app/congestion-coverage-plan/install/setup.bash" >> ~/.bashrc
# add new user and add it to sudo group
RUN useradd -ms /bin/bash user && echo "user:user" | chpasswd && adduser user sudo
USER user
WORKDIR /app/congestion-coverage-plan


CMD ["bash"]
