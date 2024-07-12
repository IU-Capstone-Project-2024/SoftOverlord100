from setuptools import find_packages, setup

package_name = 'turtle_subscriber'

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
    maintainer='yehia',
    maintainer_email='yehiasobehacm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'turtle_subscriber_node = turtle_subscriber.turtle_subscriber_node:main',
         'dummy_map_publisher = turtle_subscriber.dummy_map_publisher:main ',
         'turtle_publisher_node = turtle_subscriber.turtle_publisher_node:main',
          'turtle_request_publisher_node = turtle_subscriber.turtle_request_publisher_node:main',
        ],
    },
)
