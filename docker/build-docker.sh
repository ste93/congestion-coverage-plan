#!/bin/bash

# This script builds a Docker image for the application.


docker build -t ste93/congestion-coverage-plan:UC3_rebased_gem . --build-arg BRANCH=gem --build-arg REPO_USERNAME=ste93 --no-cache