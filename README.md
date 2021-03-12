# Pathfinder

Pathfinder is a C++17 library for finding the shortest path (for a circular agent with a specified radius) between two points in a 2d world. The 2d world is given as vertices and line segments representing the constraints of the environment (e.g. walls). This library uses [Jonathan Shewchuk's amazing Triangle library](https://www.cs.cmu.edu/~quake/triangle.html) in order to create a Constrained Delaunay Triangulation of the given world. Once the triangulation (navmesh) is created, an A*-based search algorithm is run to either find a very good path quickly or a globally optimal path at the cost of more CPU time. Most of the pathfinding algorithm concepts come from [Douglas Demyen's Efficient Triangulation-Based Pathfinding](https://skatgame.net/mburo/ps/thesis_demyen_2006.pdf).

<p align="center">
  <img src="./examples/complex_example.png" width="450" title="An example in a complex environment">
</p>

## Getting Started

These instructions will help you to build this library and link it against your own projects.

### Prerequisites

This project requires C++17 (though it could be converted to C++14 without much effort) and CMake >= 3.16.

### Building

1. Create a build directory somewhere (e.g. in a directory beside this one)

```bash
cd ../
mkdir build
cd build
```

2. Run CMake to generate Makefiles

```bash
cmake ../Pathfinder/
```

1. Build using Make

```bash
make
```

This will create a static library `libpathfinder.a` for you to link to your existing project. If necessary, you can modify [CMakeLists.txt](./CMakeLists.txt) to change this to produce a dynamically linked library or an interface library.

### Usage

Please see the [Wiki](https://github.com/SandSnip3r/Pathfinder/wiki) for details on the API.

## Known Issues

- The A* heuristic is not perfect
  - This can be worked around by applying heuristic scaling, though performance degredation is a risk
- There are no checks for whether the specified start or goal position are valid
  - i.e. the agent may be too large to fit in the chosen start or goal location
  - This will result in potentially weird and/or invalid paths
- The check to see if the agent can fit through a corridor is weak
  - This can result in paths that an agent cannot actually fit through

## Future Work

- Demyen's paper is primarily about pathfinding on an abstract graph that exists a level above the geometry-based triangle navmesh. The goal is to use this heirarchical pathfinding to greatly reduce the complexity.
- Tests, tests, tests!

## Contributing

Please read [CONTRIBUTING.md](./CONTRIBUTING.md) for details on the process for submitting pull requests to us.

## Authors

- Victor Stone - [LinkedIn](https://www.linkedin.com/in/sandsnip3r/)

## Acknowledgments

Huge thanks to:

- Jonathan Shewchuk for the incredibly fast and efficient Triangle library
- Douglas Demyen for a great collection and explanation of pathfinding techniques