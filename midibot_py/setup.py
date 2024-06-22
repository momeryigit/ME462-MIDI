from setuptools import setup, find_packages

setup(
    name="midibot_py",
    version="0.3.0",
    packages=find_packages(),
    install_requires=[
        "pyserial",
    ],
    author="Omar Habib, Sarp Dengizmen",
    author_email="omar1farouk@gmail.com; e.dengizmen@gmail.com",
    description="A Python API for the MIDIbot",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS  Independent",
        "License :: OSI Approved :: Apache Software License",
    ],
    python_requires=">=3.6",
    license="Apache License 2.0",
)
