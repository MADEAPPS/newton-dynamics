# Build with
#   $ docker build -t newton .
FROM fedora:22


# -- Install build requirements for Newton.
RUN dnf group install -y 'Development Tools'
RUN dnf install -y cmake gcc-c++ git


# -- Clone Newton repo and create build directory.
WORKDIR /home/ubuntu
RUN git clone https://github.com/MADEAPPS/newton-dynamics.git
RUN mkdir newton-dynamics/build
WORKDIR /home/ubuntu/newton-dynamics/build


# --- Build Core Library only (ie, no demos):
RUN rm -rf ./*
RUN dnf install -y -y tinyxml-devel
RUN cmake -DNEWTON_DEMOS_SANDBOX=OFF .. && make -j4


# -- Build everything, including OpenGL demos:
RUN rm -rf ./*
RUN dnf install -y wxGTK3-devel freetype-devel openal-devel glew-devel
RUN cmake -DNEWTON_DEMOS_SANDBOX=ON .. && make -j4


# Spawn Bash upon login.
ENTRYPOINT ["/bin/bash"]

