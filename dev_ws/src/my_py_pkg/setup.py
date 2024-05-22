from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='bayrakt4rdem',
    maintainer_email='68343746+Bayrakt4rdem@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "template_publishernode = my_py_pkg.template_publishernode:main",
            "template_subscribernode = my_py_pkg.template_subscribernode:main",
            "serial_publisher = my_py_pkg.serial_publisher:main",
            "battery_publishernode = my_py_pkg.battery_publishernode:main"
        ],
    },
)
