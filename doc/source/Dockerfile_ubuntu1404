# Build with
#   $ docker build -t newton .
FROM ubuntu:14.04


# -- Install build requirements for Newton.
RUN apt-get update && apt-get install -y build-essential cmake git


# -- Clone Newton repo and create build directory.
WORKDIR /home/ubuntu
RUN git clone https://github.com/MADEAPPS/newton-dynamics.git
RUN mkdir newton-dynamics/build
WORKDIR /home/ubuntu/newton-dynamics/build


# --- Build Core Library only (ie, no demos):
RUN rm -rf ./*
RUN apt-get install -y libtinyxml-dev
RUN cmake -DNEWTON_DEMOS_SANDBOX=OFF .. && make -j4


# -- Build everything, including OpenGL demos:
RUN rm -rf ./*
RUN apt-get install -y libwxgtk3.0-dev libfreetype6-dev libopenal-dev libglew-dev
RUN cmake -DNEWTON_DEMOS_SANDBOX=ON .. && make -j4


# Spawn Bash upon login.
ENTRYPOINT ["/bin/bash"]
