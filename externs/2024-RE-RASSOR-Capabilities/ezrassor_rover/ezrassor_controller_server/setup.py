"""Setup the ezrassor_controller_server module."""
import setuptools
import glob

setuptools.setup(
    name="ezrassor_controller_server",
    version="2.0.0",
    description="Handle incoming network requests for the EZRASSOR.",
    maintainer="EZRASSOR Team",
    maintainer_email="ez.rassor@gmail.com",
    license="MIT",
    keywords=["EZRASSOR", "ROS", "ISRU", "NASA", "Rover", "UCF", "Robotics"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Astronomy",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    packages=["ezrassor_controller_server", "ezcamera"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    package_data={
        "ezcamera": ["*.npy"],  # Include all .npy files in the ezcamera package
        # Adjust the path for the .png files in the ezcamera/resources folder
        "": ["ezcamera/resources/*.png"],
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_controller_server"],
        ),
        ("share/ezrassor_controller_server", ["package.xml"]),
        ("share/ezrassor_controller_server/launch", glob.glob("launch/*")),
        # Include the .png files from the ezcamera/resources folder in the build
        ("share/ezrassor_controller_server/resources", glob.glob("source/ezcamera/resources/*.png")),
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_server = ezrassor_controller_server.__main__:main",
        ],
    },
)

