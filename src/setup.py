# setup.py

import setuptools


setuptools.setup(
    name='pybullet_industrial',
    version='1.0.4',
    author='Jan Baumgärtner, Malte Hansjosten, Dominik Schönhofen',
    description='Pybullet_industrial is a process-aware robot simulation framework for pybullet.',
    long_description='Pybullet_industrial is a process-aware robot simulation framework for pybullet.',
    url='https://pybullet-industrial.readthedocs.io/en/latest/',
    license='MIT',
    install_requires=['numpy<2.0.0', 'pybullet', 'scipy'],
    packages=setuptools.find_packages(),

)
