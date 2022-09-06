from glob import glob
from setuptools import setup

package_name = 'webots_spot'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*_launch.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
    ('share/' + package_name + '/resource', glob('resource/*')),
    ('share/' + package_name + '/params', glob('params/*.yaml')),
    ('share/' + package_name + '/protos', glob('protos/*.proto')),
    ('share/' + package_name + '/protos/icons', glob('protos/icons/*.png')),
    ('share/' + package_name + '/protos/apriltags/protos', glob('apriltags/protos/*.proto')),
    ('share/' + package_name + '/protos/apriltags/images', glob('apriltags/images/*.png')),
]
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maximillian.kirsch@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_driver = ' + package_name + '.spot_driver:main',
            'spot_pointcloud2 = ' + package_name + '.spot_pointcloud2:main',
            'set_initial_pose = ' + package_name + '.set_initial_pose:main',
            'apriltag_ros = ' + package_name + '.apriltag_ros:main',
        ],
    },
)
