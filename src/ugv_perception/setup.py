from setuptools import find_packages, setup

package_name = "ugv_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Perception nodes for UGV Navigation Challenge",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector = ugv_perception.aruco_detector:main",
            "sign_detector = ugv_perception.sign_detector:main",
            "mission_controller = ugv_perception.mission_controller:main",
        ],
    },
)
