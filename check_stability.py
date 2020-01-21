import argparse
from check_rooted import check_rooted
import numpy as np
from obj2urdf import obj2urdf
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
    obj2urdf(input_file, 'tmp')
    with open('tmp/zmin.txt', 'r') as f:
        zmin = float(f.read())
    startPos = [0,0,-zmin]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    objId = p.loadURDF("tmp/assembly.urdf",startPos, startOrientation)
    shutil.rmtree('tmp')

    # See if object is stable under some perturbation
    p.applyExternalForce(objId, -1, (0, 0, 200), startPos, p.WORLD_FRAME)
    p.applyExternalTorque(objId, -1, (0, 7, 0), p.WORLD_FRAME)
    p.applyExternalTorque(objId, -1, (7, 0, 0), p.WORLD_FRAME)
    p.applyExternalTorque(objId, -1, (0, 0, 100), p.WORLD_FRAME)

    # Run simulation
    if gui:
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
    else:
        for i in range(10000):
            p.stepSimulation()
        endPos, _ = p.getBasePositionAndOrientation(objId)
        p.disconnect()
        zdiff = abs(startPos[2] - endPos[2])
        if zdiff > abs(0.05*zmin):
            return False
        return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check physical stability of an object')
    # Path to input OBJ file
    parser.add_argument('--input-file', type=str, required=True)
    # Whether to run in interactive mode (for debugging)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    print(check_stability(args.input_file, args.gui))