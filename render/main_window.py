from collections import deque
import pybullet as p
import pybullet_data
import time
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple

from config import config

planeId = None
_link_name_to_index = {}

def initialize_environment():
    global planeId
    physicsClient = p.connect(p.GUI)  # or p.DIRECT
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    p.setGravity(0, 0, -9.81)

def create_arm():
    WIDTH = HEIGHT = 0.04
    l1, l2, l3 = (1, 1, 1)
    # Define base link (root link)
    base_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[l1/2, WIDTH, HEIGHT], collisionFramePosition=[0.5, 0, 0])
    base_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[l1/2, WIDTH, HEIGHT],  visualFramePosition=[0.5, 0, 0])

    # Define links
    link1_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[l2/2, WIDTH, HEIGHT], collisionFramePosition=[0.5, 0, 0]
    )
    link1_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.5, 0.05, 0.05], visualFramePosition=[0.5, 0, 0],  rgbaColor=[1, 0, 0, 1]
    )

    link2_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.04, 0.04])
    link2_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.4, 0.04, 0.04], rgbaColor=[0, 1, 0, 1])

    # MultiBody parameters
    link_masses = [1, 1]  # Mass for link1 and link2
    link_collisions = [link1_collision, link2_collision]
    link_visuals = [link1_visual, link2_visual]

    # Positions and orientations
    link_positions = [[1, 0, 0], [0.9, 0, 0]]  # Relative positions of link1 and link2
    link_orientations = [[0, 0, 0, 1], [0, 0, 0, 1]]  # Quaternion orientations
    link_inertial_positions = [[0, 0, 0], [0, 0, 0]]
    link_inertial_orientations = [[0, 0, 0, 1], [0, 0, 0, 1]]

    # Joint parameters
    link_joint_types = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]  # Revolute joints for both links
    link_joint_axes = [[0, 0, 1], [0, 0, 1]]  # Rotational axis for both joints

    # Create the multibody
    robot = p.createMultiBody(
        baseMass=0,  # Root link is static
        baseCollisionShapeIndex=base_collision,
        baseVisualShapeIndex=base_visual,
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 1],
        linkMasses=link_masses,
        linkCollisionShapeIndices=link_collisions,
        linkVisualShapeIndices=link_visuals,
        linkPositions=link_positions,
        linkOrientations=link_orientations,
        linkInertialFramePositions=link_inertial_positions,
        linkInertialFrameOrientations=link_inertial_orientations,
        linkParentIndices=[0, 1],  # Parent indices: link1 connects to base, link2 connects to link1
        linkJointTypes=link_joint_types,
        linkJointAxis=link_joint_axes,
    )

    time.sleep(2)
    p.setJointMotorControl2(
        bodyUniqueId=robot,  # ID of the multibody
        jointIndex=0,        # Index of the first joint
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.5,  # Target angle in radians for Joint J1
    )

    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    initialize_environment()
    arm = create_arm()
    while True: pass

    