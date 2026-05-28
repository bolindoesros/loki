from setuptools import find_packages, setup

package_name = 'loki_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='bolin@bolin.com',
    description='Hardware bridge for Loki AUV.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hw_bridge = loki_hardware.hw_bridge:main'
        ],
    },
)
