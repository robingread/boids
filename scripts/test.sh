#! /bin/bash

set -e

BUILD_DIR=$(realpath build)

if [ ! -d $BUILD_DIR ]
then
    echo "Error: Build directory not found! Expected path:" ${BUILD_DIR}
    exit 1
fi

pushd $BUILD_DIR
ctest
popd