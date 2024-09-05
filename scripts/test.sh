#! /bin/bash

set -e

BUILD_DIR=$(realpath build)
TEST_DIR=$BUILD_DIR/test

if [ ! -d $BUILD_DIR ]
then
    echo "Error: Build directory not found! Expected path:" ${BUILD_DIR}
    exit 1
fi

pushd $TEST_DIR
./test_example
popd