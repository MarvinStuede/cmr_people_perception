#!/bin/bash
#Script to download and uncompress a bag file for testing
ppath=$(rospack find cmr_people_perception)
bagname=cmg_people_labor.bag
wget --no-verbose --content-disposition https://high-seas.projekt.uni-hannover.de/f/3aec3e2bfd2e4ac383a6/?dl=1 -O "$ppath/test/$bagname"
#rosbag decompress "$ppath/test/$bagname"
