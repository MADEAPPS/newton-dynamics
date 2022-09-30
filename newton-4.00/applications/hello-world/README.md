## Minimal Standalone Newton Application with CMake (Linux)

This recipe shows how to incorporate Newton into your own application.
The application only depends on the installed library and headers, not on the
Sandbox or the Newton build directory.

The example below also runs as a [Github Action](../../../.github/workflows/build.yml).

```bash
# Build Newton.
cd newton-4.00
cmake -S . -B build/
cmake --build build
sudo cmake --build build --target install

# Build the demo app (you can copy that folder anywhere, it does
# not need to reside in Newton's original build folder).
cd newton-4.00/applications/standalone
cmake -S . -B build/
cmake --build build/ -j2

# Run the demo app.
build/newton-hello-world
```
