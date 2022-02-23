#!/bin/bash

# Adapted from DLR-RM/stable-baselines3

CPU_PARENT=ubuntu:20.04
GPU_PARENT=nvidia/cuda:11.3.1-runtime-ubuntu20.04

TAG=eagerx
VERSION=$(cat pyproject.toml | grep "^version\b" | tr -dc '0-9.')
POETRY_VERSION=$(cat pyproject.toml | grep -w poetry-core | tr -dc '0-9.')
PYTHON_VERSION=3.8

if [[ ${ADD_SB} == "True" ]]; then
  TAG="${TAG}-sb"
fi

if [[ ${USE_GPU} == "True" ]]; then
  PARENT=${GPU_PARENT}
  PYTORCH_DEPS="cudatoolkit=11.3"
else
  PARENT=${CPU_PARENT}
  PYTORCH_DEPS="cpu"
  TAG="${TAG}-cpu"
fi

echo "docker build --build-arg PARENT_IMAGE=${PARENT} --build-arg PYTORCH_DEPS=${PYTORCH_DEPS} \
 --build-arg POETRY_VERSION=${POETRY_VERSION} --build-arg ADD_SB=${ADD_SB} \
 --build-arg PYTHON_VERSION="{PYTHON_VERSION}" -t ${TAG}:${VERSION} ."
docker build --build-arg PARENT_IMAGE=${PARENT} --build-arg PYTORCH_DEPS=${PYTORCH_DEPS} \
--build-arg POETRY_VERSION=${POETRY_VERSION} --build-arg ADD_SB=${ADD_SB} \
--build-arg PYTHON_VERSION=${PYTHON_VERSION} -t ${TAG}:${VERSION} .
docker tag ${TAG}:${VERSION} ${TAG}:latest

if [[ ${RELEASE} == "True" ]]; then
  docker push ${TAG}:${VERSION}
  docker push ${TAG}:latest
fi
