#! /bin/bash

set -eu

export LD_LIBRARY_PATH=/opt/boids/lib
echo "Running from entrypoint.sh"
/opt/boids/bin/boidsim
