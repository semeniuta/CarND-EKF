#! /bin/bash

cd uWebSockets

mkdir build_conda
cd build_conda

export PKG_CONFIG_PATH=$CONDA_PREFIX/lib/pkgconfig 
cmake -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX ..

make
sudo make install