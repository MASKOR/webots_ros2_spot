from glob import glob
from setuptools import setup

package_name = 'webots_spot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/spot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/spot.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/spot.urdf']))
data_files.append(('share/' + package_name + '/protos', ['protos/Spot.proto','protos/SpotLeftLeg.proto', 'protos/SpotRightLeg.proto']))
data_files.append(('share/' + package_name + '/protos', ['protos/icons/Spot.png']))
data_files.append(('share/' + package_name, ['package.xml']))
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
            'spot_driver = webots_spot.spot_driver:main',
            'env_tester = webots_spot.env_tester:main'
        ],
    },
)
