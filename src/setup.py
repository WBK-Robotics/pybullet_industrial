# setup.py
import os

import setuptools

with open("requirements.txt", "r") as fh:
    requirements = fh.readlines()

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "triped_sim"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in 'png gif jpg urdf sdf obj mtl dae off stl STL xml '.split():
            fn = root + "/" + fn
            need_files.append(fn[1+len(hh):])

setuptools.setup(
    name='pybullet_industrial',
    version='0.9',
    author='Jan Baumgärtner',
    description='Pybullet_industrial is a process-aware robot simulation framework for pybullet.',
    install_requires=[req for req in requirements if req[:2] != "# "],
    packages=setuptools.find_packages(),
    package_data={'pybullet_industrial': need_files}
)
