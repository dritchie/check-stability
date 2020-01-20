import argparse
import numpy as np
import trimesh as tm
from trimesh.collision import CollisionManager
from trimesh.creation import box


def check_rooted(input_file, collision_dist=1e-3):
    # Load up the mesh
    mesh = tm.load(input_file)
    # Switch from y-up to z-up
    mesh.vertices = mesh.vertices[:, [0, 2, 1]]
    mesh.fix_normals()
    # Find the height of the ground plane
    z_ground = mesh.bounds[0][2]

    # Extract the individual cuboid parts
    comps = mesh.split().tolist()
    # Also create a thin box for the ground plane
    ground = box(
        extents = [10, 10, 0.01],
        transform = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, z_ground - 0.01/2],
            [0, 0, 0, 1]
        ]
    )
    comps.insert(0, ground)

    # Detect (approximate) intersections between parts
    adjacencies = {comp_index : [] for comp_index in range(len(comps))}
    manager = CollisionManager()
    for i in range(len(comps)-1):
        manager.add_object(str(i), comps[i])
        for j in range(i+1, len(comps)):
            dist = manager.min_distance_single(comps[j])
            if (dist < collision_dist):
                adjacencies[i].append(j)
                adjacencies[j].append(i)
        manager.remove_object(str(i))

    # Run a DFS starting from the ground, check if everything is reachable
    visited = [False for comp in comps]
    stack = [0]     # Index of 'ground'
    while len(stack) > 0:
        nindex = stack.pop()
        visited[nindex] = True
        for cindex in adjacencies[nindex]:
            if not visited[cindex]:
                stack.append(cindex)
    return all(visited)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check if an OBJ object is "rooted" (i.e. there exists a path from the ground to all parts)')
    # Path to input OBJ file
    parser.add_argument('--input-file', type=str, required=True)
    # Optional collision distance threshold
    parser.add_argument('--collision-dist', type=float, default=1e-3)
    args = parser.parse_args()
    print(check_rooted(args.input_file, args.collision_dist))