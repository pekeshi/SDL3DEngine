# SDL3D Engine

A small C++ 3D cube renderer built with SDL 3 and the SDL GPU API.

## Features

- First-person mouse and keyboard camera controls
- Filled, outline, and triangle rendering modes
- Map-driven ground and cube placement
- Resizable window with depth buffering

## Requirements

- Linux
- C++17-compatible compiler
- SDL 3 development libraries
- Vulkan-compatible graphics support

## Build

```sh
g++ -std=c++17 sdl3d.cpp -lSDL3 -o sdl3d
```

## Run

From the project directory:

```sh
./sdl3d
```

The application loads `maps/test.map` and requires compiled SPIR-V shaders at:

- `shaders/cube.vert.spv`
- `shaders/cube.frag.spv`

## Controls

- `W`, `A`, `S`, `D`: Move
- Mouse: Look around
- `Left Shift`: Sprint
- `X`: Cycle drawing mode
- `Tab`: Toggle mouse capture
- `Escape`: Quit

## Map Format

Maps contain one optional ground definition and one or more cube definitions:

```text
# ground: y-position  halfExtent
ground -10.0 10.0

# cube: x-position y-position z-position yaw-degrees scale
cube 0.0 0.0 0.0 20.0 1.0
```

Lines beginning with `#` are comments. The default map is `maps/test.map`.
