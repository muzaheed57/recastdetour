*currently [![Build status](https://travis-ci.org/masagroup/recastdetour.png?branch=master)](https://travis-ci.org/masagroup/recastdetour)*

# About #

This repository is a fork of [recastnavigation](http://code.google.com/p/recastnavigation/) whose original author is [Mikko Mononen](memon@inside.org).

This library is releaed under the terms of the open souce [Zlib license](http://opensource.org/licenses/Zlib).

## Recast ##

Recast is state of the art navigation mesh construction toolset for games.

* It is automatic, which means that you can throw any level geometry
at it and you will get robust mesh out;
* It is fast which means swift turnaround times for level designers.

## Detour ##

Recast is accompanied with Detour, path-finding and spatial reasoning toolkit. You can use any navigation mesh with Detour, but of course the data generated with Recast fits perfectly.

Detour offers simple static navigation mesh which is suitable for many simple cases, as well as tiled navigation mesh which allows you to plug in and out pieces of the mesh. The tiled mesh allows to create systems where you stream new navigation data in and out as the player progresses the level, or you may regenerate tiles as the world changes. 

## Detour Crowd ##

The Detour Crowd module provides you with features for agents handling and behavior customization. You can create lots of agents and move them in the navigation mesh thanks to Detour. Moreover you can create your own behaviors that will tell your agents how to move and to react.

# Platforms/Compatibility #

**Recast/Detour** is written in [C++03](http://en.wikipedia.org/wiki/C%2B%2B03). It has been tested on several platforms:

- Mac OS X Mountain Lion (x64),
  - clang 4.2 (clang-425.0.24);
- Windows 7 (x86 & x64),
  - Visual Studio 2010 SP1 (10.0.40219.1 SP1Rel);
- Debian Linux 6.05 (x86),
  - GCC 4.4.5;
- Ubuntu Linux 12.04 (x86) (using [Travis](https://travis-ci.org/masagroup/recastdetour)),
  - GCC 4.6.x,
  - Clang 3.1.x.

# Build instructions #

## Windows/Visual Studio ##

### Prerequisites ###
- [Cmake](http://www.cmake.org/);
- [SDL](http://www.libsdl.org) development libraries;
- [Catch](https://github.com/philsquared/Catch) unit test library is used as a [submodule](http://git-scm.com/book/en/Git-Tools-Submodules) in [DetourCrowdTest/Contrib/catch](DetourCrowdTest/Contrib/catch);
- Virtually any recent version of visual studio, tested with,
    - Visual Studio 2010 x86,
    - Visual Studio 2010 x64.

### Build ###
1. Generate the Visual Studio files with CMake.
    - Set the source code directory to the root of the repository (e.g. `E:\recastdetour`);
    - Set the Cmake build directory to where you desire the `.sln`, `.vcproj` and co. to be (e.g. `E:\recastdetour\Build\vc100`);
    - Set the `SDL_INCLUDE_DIR` to the path of `include` directory part of the SDL development lib download (e.g. `E:/SDL-devel-1.2.15-VC/SDL-1.2.15/include`);
    - Set the `SDL_LIBRARY_TEMP` to the path of `SDL.lib` downloaded with the SDL development lib download (e.g. `E:/SDL-devel-1.2.15-VC/SDL-1.2.15/lib/x86/SDL.lib`);
    - Set the `SDLMAIN_LIBRARY` to the path of `SDLMain.lib` downloaded with the SDL development lib download (e.g. `E:/SDL-devel-1.2.15-VC/SDL-1.2.15/lib/x86/SDLmain.lib`);
    - Click *Configure* then *Generate*.
2. Initialize and update the submodule (git submodule init & git submodule update)
3. Build with Visual Studio.
    - Open `RecastNavigation.sln` that has been generated in the chosen directory (e.g. `E:\recastdetour\Build\RecastNavigation.sln`);
    - Build the project `ALL_BUILD`, it will build all libraries and executables.
    - Build the project `RUN_TESTS`, it will launch the unit tests and check that they passed successfully.
4. Execute `Recast_Demo` and `DetourCrowd_Demo`.
    - Make sure you execute the application from its Run directory (e.g. `E:\projects\recastdetour\RecastDemo\Run`);
    - Make sure `SDL.dll` is present in the path;
    - Run, and enjoy!

# Support #

- A Google Group is available [here](https://groups.google.com/forum/?fromgroups#!forum/recastnavigation)
- There is a doxygen documentation as a project, just build and open Docs/index.html
- If no notice a bug or something strange, please let us know about it on the Github repository, issues section

# Contributors #

## Master repository contributors ##
- Mikko Mononen <memon@inside.org>
- Steve ?? <stevefsp@gmail.com>
- Cameron Hart <cameron.hart@gmail.com>

## [MASA LIFE](http://www.masalife.net) team contributors ##
- Jérémy Chanut <jeremy.chanut@masagroup.net>
- Clodéric Mars <cloderic.mars@masagroup.net>
- Damien Avrillon <damien.avrillon@masagroup.net>

# Release Notes #

## Recast 2.0.1 ##
**Released August 9th, 2013**

- Removing the proximity grid (doesn't handle overlapping agents and 3D levels)
- Bug fixing on path following
- Features to controle the offMesh connections
- Path Following parameters no longer have to be initialized manually

## Recast 2.0 ##
**Released July 15th, 2013**

- The interface for the dtCrowd class has been completely rebuild.
- Implementation of the notion of behaviors for the agents.
- Some behaviors are available by default for the user to use.
- The user can create its own behaviors.
- The documentation has been updated and some tutorials have been added.
- The features of the Detour Crowd module have unit tests (more than 100)

## Recast 1.4 ##
**Released August 24th, 2009**

- Added detail height mesh generation (RecastDetailMesh.cpp) for single,
  tiled statmeshes as well as tilemesh.
- Added feature to contour tracing which detects extra vertices along
  tile edges which should be removed later.
- Changed the tiled stat mesh preprocess, so that it first generated
  polymeshes per tile and finally combines them.
- Fixed bug in the GUI code where invisible buttons could be pressed.

## Recast 1.31 ##
**Released July 24th, 2009**

- Better cost and heuristic functions.
- Fixed tile navmesh raycast on tile borders.

## Recast 1.3 ##
**Released July 14th, 2009**

- Added dtTileNavMesh which allows to dynamically add and remove navmesh pieces at runtime.
- Renamed stat navmesh types to dtStat* (i.e. dtPoly is now dtStatPoly).
- Moved common code used by tile and stat navmesh to DetourNode.h/cpp and DetourCommon.h/cpp.
- Refactores the demo code.

## Recast 1.2 ##
**Released June 17th, 2009**

- Added tiled mesh generation. The tiled generation allows to generate navigation for
  much larger worlds, it removes some of the artifacts that comes from distance fields
  in open areas, and allows later streaming and dynamic runtime generation
- Improved and added some debug draw modes
- API change: The helper function rcBuildNavMesh does not exists anymore,
  had to change few internal things to cope with the tiled processing,
  similar API functionality will be added later once the tiled process matures
- The demo is getting way too complicated, need to split demos
- Fixed several filtering functions so that the mesh is tighter to the geometry,
  sometimes there could be up error up to tow voxel units close to walls,
  now it should be just one.

## Recast 1.1 ##
**Released April 11th, 2009**

This is the first release of Detour.

## Recast 1.0 ##
**Released March 29th, 2009**

This is the first release of Recast.

The process is not always as robust as I would wish. The watershed phase sometimes swallows tiny islands
which are close to edges. These droppings are handled in rcBuildContours, but the code is not
particularly robust either.

Another non-robust case is when portal contours (contours shared between two regions) are always
assumed to be straight. That can lead to overlapping contours specially when the level has
large open areas.
