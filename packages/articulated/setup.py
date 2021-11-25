from glob import glob
from os.path import basename
from os.path import splitext

from setuptools import find_packages
from setuptools import setup


setup(
    name="articulated",
    version="0.0.0",
    license="BSD-2-Clause",
    description="",
    long_description="",
    packages=find_packages("src"),
    package_dir={"": "src"},
    py_modules=[splitext(basename(path))[0] for path in glob("src/*.py")],
    python_requires="==3.8",
    install_requires=[],
    extras_require={
        # eg:
        #   'rst': ['docutils>=0.11'],
        #   ':python_version=="2.6"': ['argparse'],
    },
    # entry_points={
    #     "console_scripts": [
    #         "quackquackduckiebot = quackquackduckiebot.cli:main",
    #     ]
    # },
)
