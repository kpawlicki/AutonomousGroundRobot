"""Setup configuration for the autonomous_ground_robot package."""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="autonomous_ground_robot",
    version="0.1.0",
    author="kpawlicki",
    description="Software stack for autonomous ground robot navigation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/kpawlicki/AutonomousGroundRobot",
    packages=find_packages(exclude=["tests*"]),
    package_data={
        "autonomous_ground_robot": ["config/*.yaml"],
    },
    python_requires=">=3.8",
    install_requires=[
        "pyyaml>=6.0",
    ],
    extras_require={
        "hardware": [
            "pyserial>=3.5",
            "RPi.GPIO>=0.7",
            "pyrealsense2>=2.50",
            "rplidar-roboticia>=0.9",
        ],
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=4.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "agr=main:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Hardware",
    ],
)
