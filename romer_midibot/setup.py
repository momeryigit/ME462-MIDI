from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as file:
    long_description = file.read()

setup(
    name="romer-midibot",
    version="1.1.5",
    packages=find_packages(),
    install_requires=[
        "pyserial",
    ],
    author="Omar Habib",
    author_email="omar1farouk@gmail.com",
    description="A Python API for Romer's MIDIbot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
    ],
    python_requires=">=3.6",
    license="Apache License 2.0",
)
