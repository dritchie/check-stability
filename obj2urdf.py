import argparse
import numpy as np
import os
import trimesh as tm
from trimesh.collision import CollisionManager
import xml.etree.ElementTree as xml


def obj2urdf(input_file, output_dir, scale=1, density=1, collision_dist=5e-3):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Load up the mesh
    mesh = tm.load(input_file)
    # Switch from y-up to z-up
    mesh.vertices = mesh.vertices[:, [0, 2, 1]]
    mesh.fix_normals()
    # Uniform scale
    s = scale
    mesh.apply_transform(np.array([
        [s, 0, 0, 0],
        [0, s, 0, 0],
        [0, 0, s, 0],
        [0, 0, 0, 1],
    ]))
    # Record how high above the ground the centroid is from the z=0 ground plane
    # (We're going to need this to know where to initialize the object in simulation)
    z_min = mesh.bounds[0][2]
    # Extract the individual cuboid parts
    comps = mesh.split().tolist()

    # Detect (approximate) intersections between parts, to use for building joints
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

    # Build a directed tree by DFS
    root = {'id': 0, 'children': []}
    fringe = [root]
    visited = set([root['id']])
    while len(fringe) > 0:
        node = fringe.pop()
        for neighbor in adjacencies[node['id']]:
            if not (neighbor in visited):
                child_node = {'id': neighbor, 'children': []}
                node['children'].append(child_node)
                visited.add(child_node['id'])
                fringe.append(child_node)

    # Write out the URDF file
    urdf_root = xml.Element('robot')
    urdf_root.set('name', 'part_graph_shape')
    # Links
    for i,comp in enumerate(comps):
        link = xml.SubElement(urdf_root, 'link')
        link.set('name', f'part_{i}')
        visual = xml.SubElement(link, 'visual')
        geometry = xml.SubElement(visual, 'geometry')
        mesh = xml.SubElement(geometry, 'mesh')
        mesh.set('filename', f'{output_dir}/part_{i}.stl')
        material = xml.SubElement(visual, 'material')
        material.set('name', 'gray')
        color = xml.SubElement(material, 'color')
        color.set('rgba', '0.5 0.5 0.5 1')
        collision = xml.SubElement(link, 'collision')
        geometry = xml.SubElement(collision, 'geometry')
        mesh = xml.SubElement(geometry, 'mesh')
        mesh.set('filename', f'{output_dir}/part_{i}.stl')
        inertial = xml.SubElement(link, 'inertial')
        mass = xml.SubElement(inertial, 'mass')
        mass.set('value', str(comp.volume * density))
        inertia = xml.SubElement(inertial, 'inertia')
        inertia.set('ixx', '1.0')
        inertia.set('ixy', '0.0')
        inertia.set('ixz', '0.0')
        inertia.set('iyy', '1.0')
        inertia.set('iyz', '0.0')
        inertia.set('izz', '1.0')
    # Joints
    fringe = [root]
    while len(fringe) > 0:
        node = fringe.pop()
        for child_node in node['children']:
            joint = xml.SubElement(urdf_root, 'joint')
            joint.set('name', f'{node["id"]}_to_{child_node["id"]}')
            joint.set('type', 'fixed')
            parent = xml.SubElement(joint, 'parent')
            parent.set('link', f'part_{node["id"]}')
            child = xml.SubElement(joint, 'child')
            child.set('link', f'part_{child_node["id"]}')
            origin = xml.SubElement(joint, 'origin')
            origin.set('xyz', '0 0 0')
            fringe.append(child_node)
    # Save to disk
    with open(f'{output_dir}/assembly.urdf', 'wb') as f:
        f.write(xml.tostring(urdf_root))

    # Write the parts to disk as STL files for the URDF to refer to
    for i,comp in enumerate(comps):
        comp.export(f'{output_dir}/part_{i}.stl')

    # Also record the z_min
    with open(f'{output_dir}/zmin.txt', 'w') as f:
        f.write(str(z_min))

    


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert an OBJ consisting of separate cuboid meshes into a URDF')
    # Path to input OBJ file
    parser.add_argument('--input-file', type=str, required=True)
    # Writes its output to a directory containing the URDF and all the part meshes
    parser.add_argument('--output-dir', type=str, required=True)
    # Optional collision distance threshold
    parser.add_argument('--collision-dist', type=float, default=5e-3)
    # Optional material density
    parser.add_argument('--density', type=float, default=1)
    # Optional uniform scale to apply to the entire object
    parser.add_argument('--scale', type=float, default=1)
    args = parser.parse_args()
    obj2urdf(args.input_file, args.output_dir, args.scale, args.density, args.collision_dist)