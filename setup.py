from setuptools import find_packages
from setuptools import setup

package_name = 'ros2custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli', 'tabulate'],
    zip_safe=True,
    author='Roberto Morales',
    author_email='rmorales.ext@catec.aero',
    maintainer='Roberto Morales',
    maintainer_email='rmorales.ext@catec.aero',
    url='https://github.com/rmorales-catec/ros2custom',
    download_url='https://github.com/rmorales-catec/ros2custom',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='A minimal plugin example for ROS 2 command line tools.',
    long_description="""The package provides the custom command as a plugin example of ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    #Necesario para que no de warning en colcon build
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'ros2cli.command': [
            'custom = ros2custom.command.custom:CustomCommand',
        ],
        'ros2custom.verb': [
            'prueba = ros2custom.verb.prueba:PruebaVerb',
            'monitor = ros2custom.verb.monitor:MonitorVerb',
        ],
    }
)
