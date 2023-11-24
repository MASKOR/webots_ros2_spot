import os
from glob import glob
from setuptools import setup

package_name = "webots_spot"
setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.wbt")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/tags/*.png")),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
        (os.path.join("share", package_name, "yolo_images"), glob("yolo_images/*")),
        (os.path.join("share", package_name, "hazmat_signs"), glob("hazmat_signs/*")),
        (os.path.join("share", package_name, "protos"), glob("protos/*.proto")),
        (
            os.path.join("share", package_name, "protos", "icons"),
            glob("protos/icons/*"),
        ),
        (
            os.path.join("share", package_name, "protos", "meshes"),
            glob("protos/meshes/*"),
        ),
        (
            os.path.join("share", package_name, "protos", "textures"),
            glob("protos/textures/*"),
        ),
        (
            os.path.join("share", package_name, "protos", "SpotArm", "meshes"),
            glob("protos/SpotArm/meshes/*"),
        ),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "map"), glob("map/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="max",
    maintainer_email="maximillian.kirsch@alumni.fh-aachen.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spot_driver = " + package_name + ".spot_driver:main",
            "spot_pointcloud2 = " + package_name + ".spot_pointcloud2:main",
            "set_initial_pose = " + package_name + ".set_initial_pose:main",
            "gpp_stacker = " + package_name + ".gpp_stacker:main",
            "retract_manipulator = " + package_name + ".retract_manipulator:main",
            "arena_modifier = " + package_name + ".arena_modifier:main",
        ],
    },
)
