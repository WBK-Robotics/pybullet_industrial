from email.policy import default
import pybullet_industrial as pi
import pybullet as p
import unittest
import os
import numpy as np
import pybullet_data

def material_spawn_test(material):
    """Helper function to test wheter a given material is spawned in their correct positions.
       This test depends on the get_positon function of the material.

    Args:
        material (_type_): _description_

    Returns:
        _type_: _description_
    """
    physics_client = p.connect(p.DIRECT)
    particle_size = 0.2
    half_extents = 0.1
    spawned_particles = pi.spawn_material_block(
        [0, 0, 0], [1, 1, 1], material, {'particle size': particle_size})

    spawned_positions = set()
    for particles in spawned_particles:
        spawned_positions.add(tuple(particles.get_position()))

    expected_positions = set()
    for x in range(int(1/particle_size)):
        for y in range(int(1/particle_size)):
            for z in range(int(1/particle_size)):
                expected_positions.add(tuple(
                    [x * particle_size+half_extents,
                     y * particle_size+half_extents,
                     z * particle_size+half_extents]))

    p.disconnect()
    return expected_positions == spawned_positions

class TestMaterials(unittest.TestCase):

    def test_metal_voxel(self):
        self.assertTrue(material_spawn_test(pi.MetalVoxel))

    def test_plastic(self):
        self.assertTrue(material_spawn_test(pi.Plastic))

    def test_paint(self):
        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        cube=p.loadURDF("cube.urdf", [2, 0, 0.5], useFixedBase=False)

        ray_cast_result=p.rayTest([1.75, 0, 1.2], [1.75, 0, 0])
        material_properties={'particle size': 0.3,
                            'color': [1, 0, 0, 1]}
        paint_particle=pi.Paint(ray_cast_result[0], material_properties)

        new_pos=[5,0,0.5]
        new_orn=p.getQuaternionFromEuler([0,0,np.pi/2])
        
        p.resetBasePositionAndOrientation(cube, new_pos, new_orn)
        painted_position=paint_particle.get_position()
        expected_position=[5, -0.25, 1]
        
        output=np.allclose(painted_position, expected_position)
        p.disconnect()
        self.assertTrue(output)
       

if __name__ == '__main__':
    unittest.main()
