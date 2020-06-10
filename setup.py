from setuptools import setup, find_packages

setup(
    name='pro',
    version='0.1.0',
    url='https://github.com/johwiebe/pro.git',
    author='Johannes Wiebe',
    author_email='j.wiebe17@imperial.ac.uk',
    description='Pyomo robust optimization toolbox',
    packages=find_packages(),
    install_requires=['pyomo'],
)
