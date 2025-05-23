from setuptools import find_packages
from setuptools import setup

package_name = 'rosidl_pycommon'

setup(
    name=package_name,
    version='3.3.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Michel Hidalgo, Shane Loretz',
    maintainer_email='michel@ekumenlabs.com, sloretz@openrobotics.org',
    description='Common Python functions used by rosidl packages.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
