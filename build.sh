#!/bin/bash

# Install script for application data object management library

# # declare default parameters
# threads="6"
# dlog="ON"
# dconsole="ON"
# dfile="ON"

# while getopts "j:l:c:f" opt; do
#   case $opt in
#     j)
#       threads=$OPTARG
#       ;;
#     l)
#       dlog="OFF"
#       ;;
#     c)
#       dconsole="OFF"
#       ;;
#     f)
#       dfile="OFF"
#       ;;
#   esac
# done

if [ ! -d "build" ]; then
    mkdir build
fi
cd build

# Execute the cmake with the stored parameters
cmake .. -DLOG=$dlog -DCONSOLE=$dconsole -DFILE=$dfile -DCMAKE_BUILD_TYPE=Debug
sudo cmake --build . --target install -j$threads 
