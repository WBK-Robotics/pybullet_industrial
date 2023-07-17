import math
import numpy as np
import pybullet as p


class Point3D:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.position = np.array([x, y, z])
        self.orientation = np.array([roll, pitch, yaw])
        self.offset = np.array([roll, pitch, yaw])

    def update_point(self, new_x=0, new_y=0, new_z=0, new_roll=0, new_pitch=0,
                     new_yaw=0):

        or0 = p.getQuaternionFromEuler(self.orientation)

        p1, o1 = p.multiplyTransforms(
            self.position, or0, self.position, self.offset)

        new_posision = np.array([new_x, new_y, new_z])
        new_orientation = p.getQuaternionFromEuler(
            np.array([new_roll, new_pitch, new_yaw]))

        p2, o2 = p.multiplyTransforms(
            p1, o1, new_posision, new_orientation)

        self.position = p2
        self.orientation = p.getEulerFromQuaternion(o2)

    def apply_offset(self, offset_roll, offset_pitch, offset_yaw):

        self.offset = p.getQuaternionFromEuler(
            np.array([offset_roll, offset_pitch, offset_yaw]))


# Example usage
p.connect(p.DIRECT)  # Connect to the physics engine

point = Point3D()

print("Initial Orientation:", point.orientation)

x = p.getQuaternionFromEuler(
    [(np.pi/2), 0, 0])
v = p.getQuaternionFromEuler(
    [(np.pi/2), (np.pi/2), 0])
y = p.getQuaternionFromEuler(
    [(np.pi/2), 0, (np.pi/2)])
z = p.getQuaternionFromEuler(
    [(np.pi/2), (np.pi/2), (np.pi/2)])

point.apply_offset(offset_roll=(np.pi/2),
                   offset_pitch=(np.pi/2),
                   offset_yaw=0)


point.update_point(new_roll=math.radians(90))

print("New Orientation:", np.degrees(point.orientation))

p.disconnect()  # Disconnect from the physics engine
