import argparse
import json
import numpy as np
import os
import trimesh as tm
from trimesh.collision import CollisionManager
from trimesh.util import concatenate as meshconcat
import xml.etree.ElementTree as xml


def obj2urdf(input_file, output_dir, density=1):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Load up the mesh
    mesh = tm.load(input_file)
    # Switch from y-up to z-up
    mesh.vertices = mesh.vertices[:, [0, 2, 1]]
    mesh.fix_normals()
    # Extract the individual cuboid parts
    comps = mesh.split().tolist()

    # Detect (approximate) intersections between parts, to use for building joints
    collision_dist = 0.005 * mesh.scale
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

    # Compute connected components
    conn_comps = []
    visited = [False for _ in comps]
    while not all(visited):
        conn_comp = set([])
        start_idx = visited.index(False)
        stack = [start_idx]
        while len(stack) > 0:
            idx = stack.pop()
            visited[idx] = True
            conn_comp.add(idx)
            for nidx in adjacencies[idx]:
                if not visited[nidx]:
                    stack.append(nidx)
        conn_comps.append(list(conn_comp))

    # We export one URDF object file per connected component
    for i,conn_comp in enumerate(conn_comps):

        # Re-center this connected component
        ccmesh = meshconcat([comps[j] for j in conn_comp])
        c = ccmesh.centroid
        transmat = [
            [1, 0, 0, -c[0]],
            [0, 1, 0, -c[1]],
            [0, 0, 1, -c[2]],
            [0, 0, 0, 1]
        ]
        for j in conn_comp:
            comps[j].apply_transform(transmat)
        ccmesh.apply_transform(transmat)
        # Also, record where to start this mesh in the simulation
        # That's the x,y coords of the centroid, and -bbox bottom for the z (so it sits on the ground)
        # And the bbox diagonal (we use this for error thresholding)
        metadata = {
            'start_pos': [c[0], c[1], -ccmesh.bounds[0][2]],
            'scale': ccmesh.volume
        }
        with open(f'{output_dir}/assembly_{i}.json', 'w') as f:
            f.write(json.dumps(metadata))

        # Build a directed tree by DFS
        root_idx = conn_comp[0]
        root = {'id': root_idx, 'children': []}
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

        # Build up the URDF data structure
        urdf_root = xml.Element('robot')
        urdf_root.set('name', 'part_graph_shape')
        # Links
        for j in conn_comp:
            comp = comps[j]
            link = xml.SubElement(urdf_root, 'link')
            link.set('name', f'part_{j}')
            visual = xml.SubElement(link, 'visual')
            geometry = xml.SubElement(visual, 'geometry')
            mesh = xml.SubElement(geometry, 'mesh')
            mesh.set('filename', f'{output_dir}/part_{j}.stl')
            material = xml.SubElement(visual, 'material')
            material.set('name', 'gray')
            color = xml.SubElement(material, 'color')
            color.set('rgba', '0.5 0.5 0.5 1')
            collision = xml.SubElement(link, 'collision')
            geometry = xml.SubElement(collision, 'geometry')
            mesh = xml.SubElement(geometry, 'mesh')
            mesh.set('filename', f'{output_dir}/part_{j}.stl')
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

        # Save URDF file to disk
        # Have to make sure to split it into multiple lines, otherwise Bullet's URDF parser will
        #    throw an error trying to load really large files as a single line...
        xmlstring = xml.tostring(urdf_root, encoding='unicode')
        xmlstring = '>\n'.join(xmlstring.split('>'))
        with open(f'{output_dir}/assembly_{i}.urdf', 'w') as f:
            f.write(xmlstring)

    # Write the parts to disk as STL files for the URDF to refer to
    for i,comp in enumerate(comps):
        comp.export(f'{output_dir}/part_{i}.stl')

    


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert an OBJ consisting of separate cuboid meshes into a URDF')
    # Path to input OBJ file
    parser.add_argument('--input-file', type=str, required=True)
    # Writes its output to a directory containing the URDF and all the part meshes
    parser.add_argument('--output-dir', type=str, required=True)
    # Optional material density
    parser.add_argument('--density', type=float, default=1)
    args = parser.parse_args()
    obj2urdf(args.input_file, args.output_dir, args.density)