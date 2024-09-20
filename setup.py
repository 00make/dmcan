from setuptools import setup, find_packages

with open("README.rst", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="dmcan",
    version="0.1.2",
    author="达妙智能控制",
    author_email="support@dmrobot.com", 
    description="用于控制DM系列电机的Python库",
    long_description=long_description,
    long_description_content_type="text/x-rst",
    url="https://github.com/00make/dmcan",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    python_requires=">=3.6",
    install_requires=[
        "pyserial>=3.4",
        "numpy>=1.19.0",
    ],
)