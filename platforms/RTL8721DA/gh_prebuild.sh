#!/bin/sh

repo_root="$PWD"

pip install -r sdk/ameba-rtos/tools/requirements.txt
mkdir /opt/rtk-toolchain
cd /opt/rtk-toolchain
wget https://github.com/Ameba-AIoT/ameba-toolchain/releases/download/prebuilts-v1.0.3/prebuilts-linux-1.0.3.tar.gz
tar -xzf prebuilts-linux-1.0.3.tar.gz
cd "$repo_root" || exit 1
source sdk/ameba-rtos/ameba.sh
