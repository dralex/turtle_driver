from glob import glob

from setuptools import find_packages, setup

package_name = 'turtle_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start.launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # the hsm-turtle plugin of CyberiadaPro (docs/editor_plugin.md)
        ('share/' + package_name + '/editor-plugin',
            ['editor-plugin/plugin.json', 'editor-plugin/turtle_env']),
        ('share/' + package_name + '/editor-plugin/i18n', glob('editor-plugin/i18n/*.json')),
        ('share/' + package_name + '/editor-plugin/icons', glob('editor-plugin/icons/*.svg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexey Fedoseev',
    maintainer_email='aleksey@fedoseev.net',
    description='The simple ROS2 node which control a turtle moving to the particular point',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'driver = turtle_driver.turtle_driver:main'
        ],
    },
)
