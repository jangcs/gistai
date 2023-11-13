from setuptools import setup

package_name = 'crms_demo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jangcs',
    maintainer_email='jangcs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'FoodListBuilder_node = crms_demo_robot.FoodListBuilder_node:main',
            'FoodMention_node = crms_demo_robot.FoodMention_node:main',
            'Camera_node = crms_demo_robot.Camera_node:main',
            'listener_py = crms_demo_robot.listener_py:main',
            'talker_py = crms_demo_robot.talker_py:main'
        ],
    },
)
