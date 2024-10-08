from setuptools import setup
import os
from glob import glob

package_name = 'diff_bot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['odom_transform = diff_bot_description.odom_tf:main',
                            'traj_viz = diff_bot_description.visualize_pose:main',
                            'omni_pub_joint_vel=diff_bot_description.isaac_teleop:main',
                            'rec_frames=diff_bot_description.rec_frames:main',
        ],
    },
)
