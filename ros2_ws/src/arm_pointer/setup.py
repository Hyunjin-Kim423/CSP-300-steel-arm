from setuptools import find_packages, setup

package_name = "arm_pointer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/pointer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="0413l",
    maintainer_email="0413ldy@gmail.com",
    description="Point an SO-ARM101 follower arm at a 3D target received on /target_point.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "pointer_node = arm_pointer.pointer_node:main",
        ],
    },
)
