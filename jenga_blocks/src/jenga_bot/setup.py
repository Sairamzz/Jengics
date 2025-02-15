from setuptools import find_packages, setup
import os
import glob

package_name = 'jenga_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*.*')),
        (os.path.join('share', package_name, 'meshes'), glob.glob('meshes/*.*')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.*')),
        (os.path.join('share', package_name, 'meshes/wx250s_meshes'), glob.glob('meshes/wx250s_meshes/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rituraj',
    maintainer_email='riturajn1200@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
