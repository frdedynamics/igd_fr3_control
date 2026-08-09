import os
from glob import glob
from setuptools import setup

package_name = 'igd_fr3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hvl-robotics2404',
    maintainer_email='gizem.ates@hvl.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'igd_fr3_control = igd_fr3_control.igd_fr3_control:main',
            'spacemouse_publisher = igd_fr3_control.spacemouse_publisher:main',
            'spacemouse_twiststamped_publisher = igd_fr3_control.spacemouse_twiststamped_publisher:main',
            'tsdf_grasp_node = igd_fr3_control.tsdf_grasp_node:main',
            'temp = igd_fr3_control.temp:main',
            'save_extrinsic = igd_fr3_control.save_extrinsic:main',
            'camera_color_duplicate_publisher = igd_fr3_control.camera_color_duplicate_publisher:main'
        ],
    },
)
