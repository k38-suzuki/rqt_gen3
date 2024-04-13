#!/bin/sh

sudo apt install ros-noetic-moveit
sudo apt install python3 python3-pip
sudo python3 -m pip install conan==1.59
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
rosdep install --from-paths src --ignore-src -y

