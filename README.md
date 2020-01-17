# check-stability
Check the physical stability of objects defined as fixed assemblies of rigid parts

## Setup

On MacOS, Just run install_macos.sh to install all the dependencies.
For Linux, it should be easy to modify this script to use `apt` instead of `brew`.

## Usage

Assumes that each input object is provided as a single .OBJ file, where the different rigid parts of the object are different connected components in the mesh.

This repo contains the following scripts (read the `argparse` blocks in each file for detailed usage info):
* `stability_stats.py`: Takes as input a directory of .OBJ files and checks stability on all of them, writing the result to a CSV file. Will also print out the percentage of objects that were stable.
* `check_stability.py`: Checks stability for a single object and prints the result the command line (used as a subroutine by `stability_stats.py`). You can also run it in interactive mode by passing the `--gui` flag, which will open up a Bullet GUI window so you can see what's going on when your object is subjected to external forces (i.e. if it's not stable, how does it fall over?)
* `obj2urdf.py`: Converts an .OBJ file to the URDF format expected by Bullet (used as a subroutine by `check_stability.py`
