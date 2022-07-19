from setuptools import setup

setup(
    name='nmpyc',
    version='1.0.1',
    packages=['nmpyc'],
    author='Jonas Schießl, Lisa Krügel',
    author_email='nmpyc@uni-bayreuth.de',
    url='https://nmpyc.readthedocs.io/',
    description='Python library for solving optimal control problems via model predictive control',
    license='GNU Lesser General Public License version 3',
    long_description=open('README.md', 'r').read(),
    long_description_content_type="text/markdown",
    install_requires=[
        "casadi",
        "osqp",
        "numpy",
        "scipy",
        "matplotlib",
        "dill"
    ],
)



