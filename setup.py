from setuptools import setup, find_packages


setup(
    name="pyopendbc",
    version="1.0",
    packages=find_packages(),
    package_dir={
        'pyopendbc': 'pyopendbc',
        'opendbc': 'opendbc',
    },
)