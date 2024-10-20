#!/bin/bash


git submodule update --init --recursive

python3 -m venv .venv
source .venv/bin/activate
pip install -r scripts/python/requirements.txt

if [ "$(id -u)" -e 0 ]; then
  sudo apt-get install -y cmake ninja-build clang ccache
fi

submodules/ariadna_constants/./generate-files.sh

cmake -B build -G "Ninja" -DCOPY_COMPILE_COMMANDS=ON
cmake --build build