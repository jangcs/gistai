from setuptools import setup

package_name = 'crms_demo'

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
            'FoodListBuilder_node = crms_demo.FoodListBuilder_node:main',
            'FoodMention_node = crms_demo.FoodMention_node:main',
			'listener_py = crms_demo.listener_py:main',
			'talker_py = crms_demo.talker_py:main'
        ],
    },
)
