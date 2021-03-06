import setuptools
import os

version = '1.0'

with open(os.path.join(os.path.dirname(__file__), 'README.md')) as f:
    LongDescription = f.read()

setuptools.setup(
    name='dronekit',
    zip_safe=True,
    version=version,
    description='Ardupilot API',
    long_description_content_type="text/markdown",
    long_description=LongDescription,
    url='https://github.com/o-gent/dronekit-python',
    author='ogent',
    install_requires=[
        'pymavlink>=2.2.20',
    ],
    author_email='',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
    ],
    license='apache',
    packages=setuptools.find_packages()
)
