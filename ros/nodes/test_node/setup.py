from setuptools import find_packages, setup

package_name = 'test_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['test_node', 'test_node.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='developer@nextind.eu',
    description='Simple GUI test node to publish on sensor/analyze topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_node = test_node.test_node:main',
        ],
    },
)
