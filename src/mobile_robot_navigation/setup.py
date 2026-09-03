from setuptools import find_packages, setup

package_name = 'mobile_robot_navigation'

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
    maintainer='cs_kn',
    maintainer_email='181945253+MonasteryStudent@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "navigate_to_position_server = mobile_robot_navigation.navigate_to_position_server:main",
            "navigate_to_position_client = mobile_robot_navigation.navigate_to_position_client:main"
        ],
    },
)
