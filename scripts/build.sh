#! /bin/bash

set -e

BUILD_DIR=$(realpath build)

if [ ! -d $BUILD_DIR ]
then
    echo "Creating new build dir:" $BUILD_DIR
    mkdir $BUILD_DIR
fi

pushd $BUILD_DIR
cmake ..
make -j8
popd
