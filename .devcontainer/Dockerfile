FROM ubuntu:22.04 AS base

ARG DEBIAN_FRONTEND=noninteractive

# Install core dependencies
RUN apt-get update && apt-get install -y \
    catch2 \
    cmake \
    g++ \
    lcov \
    libgtest-dev \
    python3-pip \
    qtbase5-dev \
    qttools5-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Python based dependencies
RUN pip3 install \
    fastcov

#########################
# Build Development stage
#########################
FROM base AS dev

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install tools for development
RUN apt-get update && apt-get install -y \
    clang-format \
    gdb \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}

##########################
# Build Production Stage #
##########################
FROM base AS prod

# Get all the files needed to compile
WORKDIR /workspaces/boids
COPY CMakeLists.txt .
COPY src/ src/
COPY test/ test/

# Compile and install
RUN mkdir build && \
    cd build && \
    cmake \
        -DCMAKE_INSTALL_PREFIX=/opt/boids \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DCODE_COVERAGE=OFF \
        .. && \
    make -j8 && \
    make install

# Cleanup
WORKDIR /
RUN rm -rf /workspaces

# Set the main run command
COPY docker/assets/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
