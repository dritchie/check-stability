import argparse
from check_rooted import check_rooted
import json
import math
import numpy as np
from obj2urdf import obj2urdf
import os
import pybullet as p
import pybullet_data
import shutil
import time

def check_stability(input_file, gui=False):
    # First, check if the file is even rooted.
    # If it's not rooted, it can't be stable
    if not check_rooted(input_file):
        return False

    # Start up the simulation
    mode = p.GUI if gui else p.DIRECT
    physicsClient = p.connect(mode)
    p.setGravity(0, 0, -9.8)

    # Load the ground plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # print(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    # Convert the object to a URDF assembly, load it up
    # There may be more than one URDF, if the object had more than one connected component
    obj2urdf(input_file, 'tmp')
    objIds = []
    startPositions = {}
    scales = {}
    for urdf in [f for f in os.listdir('tmp') if os.path.splitext(f)[1] == '.urdf']:
        with open(f'tmp/{os.path.splitext(urdf)[0]}.json', 'r') as f:
            data = json.loads(f.read())
        startPos = data['start_pos']
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        objId = p.loadURDF(f"tmp/{urdf}",startPos, startOrientation)
        objIds.append(objId)
        startPositions[objId] = startPos
        scales[objId] = data['scale']
    shutil.rmtree('tmp')

    # Disable collisions between all objects (we only want collisions between objects and the ground)
    # That's because we want to check if the different components are *independently* stable, and
    #    having them hit each other might muck up that judgment
    for i in range(0, len(objIds)-1):
        ni = p.getNumJoints(objIds[i])
        for j in range(i+1, len(objIds)):
            nj = p.getNumJoints(objIds[j])
            for k in range(-1, ni):
                for l in range(-1, nj):
                    p.setCollisionFilterPair(objIds[i], objIds[j], k, l, False)

    # See if objects are stable under some perturbation
    for objId in objIds:
        s = scales[objId]
        p.applyExternalForce(objId, -1, (0, 0, 600*s), startPositions[objId], p.WORLD_FRAME)
        p.applyExternalTorque(objId, -1, (0, 5*s, 0), p.WORLD_FRAME)
        p.applyExternalTorque(objId, -1, (5*s, 0, 0), p.WORLD_FRAME)
        p.applyExternalTorque(objId, -1, (0, 0, 200*s), p.WORLD_FRAME)

    # Run simulation
    if gui:
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
    else:
        for i in range(10000):
            p.stepSimulation()
        for objId in objIds:
            endPos, _ = p.getBasePositionAndOrientation(objId)
            zend = endPos[2]
            zstart = startPositions[objId][2]
            zdiff = abs(zend - zstart)
            if zdiff > abs(0.05*scales[objId]):
                p.disconnect()
                return False
        p.disconnect()
        return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check physical stability of an object')
    # Path to input OBJ file
    parser.add_argument('--input-file', type=str, required=True)
    # Whether to run in interactive mode (for debugging)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    print(check_stability(args.input_file, args.gui))