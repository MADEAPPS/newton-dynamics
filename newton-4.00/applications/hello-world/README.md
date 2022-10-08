## Minimal Standalone Application with CMake (Linux)

This folder contains several self contained Newton demos. Each shows a basic
feature in isolation, for instance how to create a Newton world, or how to use
Collision callbacks.

The [CMakeLists.txt](CMakeLists.txt) also only contains the bare minimum to
build those applications against an *installed* version of Newton. As such, the
demos do not depend on the Newton source code, or the build tree or the
sandbox. The purpose of these Hello World demos is to provide a starting point
on how to integrate Newton into your own applications.

The examples below also run in a [Github Action](../../../.github/workflows/build.yml).

### Installation
If you have not compiled and installed Newton yet you can do so with

```bash
cd newton-4.00
cmake -S . -B build/
cmake --build build
sudo cmake --build build --target install
```

### Build the Demos
The demos reside in `newton-4.00/applications/hello-world` but you can build
them from anywhere.

```bash
cd newton-4.00/applications/hello-world
cmake -S . -B build/
cmake --build build/ -j2

# hello: create an empty Newton world and exit. It contains everything you
#    need to integrate Newton into your own projects.
build/hello

# transformCallback: simulate a falling sphere. Uses the BodyNotify
#    callback to apply gravity at each tick and track the position of the sphere.
build/transformCallback

# collisionCallback: drops a dynamic sphere onto a static ground (box). Shows
#     how to setup the collision callback to get notified about contacts, ie.
#     whenever the sphere hits the ground.
build/collisionCallback
```
