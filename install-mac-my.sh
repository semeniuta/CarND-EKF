#! /bin/bash

# The addressed issue with OpenSSL directories
# https://github.com/udacity/CarND-Kidnapped-Vehicle-Project/issues/11

brew install openssl libuv cmake

git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1

cp CMakeLists.txt CMakeLists0.txt # back-up the original CMakeLists
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build

# This is how /usr/local/Cellar/openssl extracted with the brew command:
# $(brew --cellar openssl)

OPENSSL_DIR=/usr/local/Cellar/openssl/1.0.2o_2
cmake -DOPENSSL_ROOT_DIR=$OPENSSL_DIR -DOPENSSL_LIBRARIES=$OPENSSL_DIR/lib ..

#make 
#sudo make install
#cd ..
#cd ..
#sudo rm -r uWebSockets
