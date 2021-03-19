#!/bin/bash
#Script to install dependencies
rosdistro=$ROS_DISTRO

#Add L-CAS repositories (topological navigation)
curl -s http://lcas.lincoln.ac.uk/repos/public.key | sudo apt-key add -
sudo apt-add-repository http://lcas.lincoln.ac.uk/ubuntu/main
sudo apt-get update

sudo apt update
sudo apt install -y	 ros-$rosdistro-spencer-people-tracking-full \
                     ros-$rosdistro-strands-perception-people \
                     ros-$rosdistro-bayes-tracking \
                     libqglviewer-dev \
                     freeglut3-dev
