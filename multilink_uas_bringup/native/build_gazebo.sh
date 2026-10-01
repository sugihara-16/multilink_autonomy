#!/bin/bash
set -eo pipefail
cd /root
if [ ! -d gz-sim/.git ]; then git clone --depth 1 --branch sim8/dev/multicopter_control https://github.com/ntnu-arl/gz-sim.git; fi
cd gz-sim
git fetch --depth 1 origin 1897a13938a3670e575b0f0ae45e0b5c02c427ed
git checkout --detach FETCH_HEAD
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_DOCS=false -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
sudo cmake --install build
sudo ldconfig
