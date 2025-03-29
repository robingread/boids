#! /bin/bash

set -e

BUILD_DIR=$(realpath build)

if [ ! -d $BUILD_DIR ]
then
    echo "Error: Build directory not found! Expected path:" ${BUILD_DIR}
    exit 1
fi

pushd $BUILD_DIR
    # Run the test suite
    export QT_QPA_PLATFORM=offscreen
    ctest --verbose
    
    # Generate the coverage report
    python3 -m fastcov \
        -d . \
        --exclude '/usr/*' '*/test/*' \
        --lcov \
        -o filtered_coverage.info

    genhtml filtered_coverage.info --output-directory coverage-report
popd