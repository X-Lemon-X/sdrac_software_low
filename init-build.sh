#!/bin/bash


git submodule update --init --recursive

python3 -m venv .venv
source .venv/bin/activate
pip install -r scripts/python/requirements.txt
pip install -r requirements.txt

pre-commit install
pre-commit autoupdate
pre-commit install


if [[ "$(id -u)" -eq 0 ]]; then
  sudo apt-get install -y cmake ninja-build clang ccache
fi

src/can_constants/./generate-files.sh -v

./build-firmware.sh