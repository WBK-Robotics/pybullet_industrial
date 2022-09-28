# setup.py

import setuptools

with open("requirements.txt", "r") as fh:
    requirements = fh.readlines()


setuptools.setup(
    name='pybullet_industrial',
    version='0.9',
    author='Jan Baumgärtner, Malte Hansjosten, Dominik Schönhofen',
    description='Pybullet_industrial is a process-aware robot simulation framework for pybullet.',
    url='https://pybullet-industrial.readthedocs.io/en/latest/',
    license='MIT',
    install_requires=[req for req in requirements if req[:2] != "# "],
    packages=setuptools.find_packages(),

)
