from setuptools import find_packages, setup

package_name = 'jarvis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main = jarvis.main_ctrl:main',
            'robot = jarvis.robot_ctrl:main',
            'vision = jarvis.vision_ctrl:main',
            'vision_controller = jarvis.vision_controller:main',
        ],
    },
)
