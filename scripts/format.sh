#!/bin/bash

set -e

# Directories to format
DIRECTORIES=("src/" "test/")

# Loop over directories
for DIRECTORY in "${DIRECTORIES[@]}"; do
    # Check if directory exists
    if [ -d "$DIRECTORY" ]; then
        # Find and format all C and C++ source files
        eval "find \"$DIRECTORY\" \( -name \"*.cpp\" -o -name \"*.hpp\" -o -name \"*.c\" -o -name \"*.h\" \) -exec clang-format -i {} +"
        echo "ClangFormat applied to all source files in $DIRECTORY."
    else
        echo "Directory $DIRECTORY does not exist!"
    fi
done