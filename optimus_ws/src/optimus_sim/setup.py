from glob import glob

from setuptools import setup

package_name = "optimus_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thirunavukkarasu Veeramani",
    description="Warehouse simulation and obstacle-response tests for the Optimus AMR",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_test = optimus_sim.obstacle_test:main",
            "spawn_obstacle = optimus_sim.spawn_obstacle:main",
            "traffic = optimus_sim.traffic:main",
        ],
    },
)
