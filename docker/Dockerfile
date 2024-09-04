# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

WORKDIR /agt

# Update the package lists
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y && apt-get autoclean -y

# Install required dependencies
RUN apt-get install -y \
  cmake \
  ninja-build \
  build-essential \
  curl 


# Download and unpack the ARM GNU toolchain
RUN curl -o arm-gnu-toolchain.tar.xz  -O https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz &&\
  tar -xf arm-gnu-toolchain.tar.xz && \
  rm arm-gnu-toolchain.tar.xz

# Add the toolchain to the PATH
ENV PATH="/agt/arm-gnu-toolchain/bin:${PATH}"


# Add Tini
ENV TINI_VERSION=v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini
ENTRYPOINT ["/tini", "--"]

# Run your program under Tini
CMD [ "sleep" , "infinity" ]