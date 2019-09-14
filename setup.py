import re

from setuptools import setup

version = re.search(
    '^__version__\\s*=\\s*"(.*)"',
    open('pyal5d/__init__.py').read(),
    re.M
).group(1)

with open("README.md", 'r') as f:
    long_description = f.read()

setup(
      name="pyal5d",
      version=version,
      author="Eduardo Nunes",
      author_email="diofanto.nunes@gmail.com",
      license="MIT",
      description="Communication with robotic arm AL5D-Lynxmotion",
      long_description=long_description,
      long_description_content_type="text/markdown",
      url="https://github.com/dioph/pyal5d",
      packages=["pyal5d"],
      install_requires=["pyserial", "numpy"],
      classifiers=[
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: MIT License",
          "Intended Audience :: Science/Research",
      ],
)
