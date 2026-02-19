FROM ubuntu:24.04
ENV  DOCKER_BUILDKIT=1
ARG BRANCH=experiments


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

RUN git clone --branch $BRANCH https://github.com/convince-project/congestion-coverage-plan.git

WORKDIR /app/congestion-coverage-plan
RUN bash -c "python3 -m venv venv_congestion_coverage_plan && \
    . venv_congestion_coverage_plan/bin/activate && \
    pip install --upgrade pip && \
    pip install -e ."
# Set the entrypoint to the Python interpreter

CMD ["bash"]