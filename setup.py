from setuptools import setup, find_packages

setup(
    name='romodel',
    version='0.0.1',
    url='https://github.com/johwiebe/romodel.git',
    author='Johannes Wiebe',
    author_email='j.wiebe17@imperial.ac.uk',
    description='Pyomo robust optimization toolbox',
    packages=find_packages(),
    install_requires=['pyomo'],
)
