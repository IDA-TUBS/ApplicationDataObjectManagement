#!/bin/sh
cd thirdparty/boost_1_73_0/
sh bootstrap.sh --prefix=/usr/local/ --libdir=/usr/local/lib/
sudo ./b2 install
sudo ldconfig