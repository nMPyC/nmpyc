from setuptools import setup

setup(
    name='nMPyC',
    version='1.0.0',
    packages=['nMPyC'],
    author='Jonas Schießl',
    #author_email='sergio.lucia@tu-berlin.de',
    #url='https://www.do-mpc.com',
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



