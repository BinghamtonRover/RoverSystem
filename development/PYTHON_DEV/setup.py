from setuptools import setup, find_packages


setup(
    name="BinghamtonRover",
    version="0.0.1",
    install_requires=[
        "inputs"
        "pyglet"
    ],
    # setup tests (allowing for "python setup.py test")
    tests_require=['mock', 'nose', 'coverage'],
    test_suite="nose.collector",
    packages=find_packages(exclude=["ez_setup"]),

)