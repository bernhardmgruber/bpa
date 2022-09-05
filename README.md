# Ball Pivoting Algorithm (BPA)

![Reconstruction of bunny model](bunny.png)

This is an exemplary C++ implementation of the ball pivoting algorithm for creating triangle meshes from point clouds.
The implementation is intentionally kept simple and is based on:

*The Ball-Pivoting Algorithm for Surface Reconstruction* by Fausto Bernardini, Joshua Mittleman, Holly Rushmeier, Claudio Silva and Gabriel Taubin

The model of the bunny is provided by the [Stanford University Computer Graphics Laboratory](http://graphics.stanford.edu/data/3Dscanrep/).

## Building

1. Please install the following dependencies using your favourite dependency manager (I tried vcpkg):
    - glm
    - Catch2v3

2. Generate build system using CMake
```
git clone https://github.com/bernhardmgruber/bpa.git
mkdir bpa/build
cd bpa/build
cmake ..
```

3. Run `make` on Linux or open and build Visual Studio solution on Windows.
   Requires a compiler supporting C\++20, but can probably be easily ported down to C\++17 or C\++14.