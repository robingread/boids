#± /bin/bash
set -e

INSTALL_DIR=/workspaces/boids/build/install
BIN_DIR=$INSTALL_DIR/bin

if [ ! -d $BIN_DIR ]
then
    echo "Error: Binary directory not found! Expected path:" ${BUILD_DIR}
    exit 1
fi

export LD_LIBRARY_PATH=${INSTALL_DIR}/lib:$LD_LIBRARY_PATH

pushd $BIN_DIR
./boidsim
popd
