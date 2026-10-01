#!/bin/bash
set -eo pipefail
python3 -m venv --system-site-packages /root/.venvs/uas
/root/.venvs/uas/bin/pip install torch==2.1.0 torchvision==0.16.0 --index-url https://download.pytorch.org/whl/cu118
/root/.venvs/uas/bin/pip install gymnasium==0.28.0 sample-factory==2.1.1 numpy==1.23.5 scipy matplotlib catkin_tools
