from setuptools import find_packages, setup

package_name = "franka_buttons_ros2"

setup(
    name=package_name,
    version="0.0.0",
    maintainer="Jelle Hierck",
    maintainer_email="j.j.hierck@student.utwente.nl",
    description="Read the franka buttons and publish it in ROS 2",
    license="MIT",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
    ],
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "websockets",
        "requests",
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    zip_safe=True,
)
