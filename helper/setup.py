import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = "helper"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name = PACKAGE_NAME,
    version="1.0.0",
    packages = [PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch","*.launch.py")))
    ],
    zip_safe=True,
    author="Anton Larionov",
    author_email="statskey.sovetnik@gmail.com",
    description="Package for simulation of a vision system",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "helper = helper.helper:main",
        ],
    },
)
