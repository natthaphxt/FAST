from setuptools import find_packages, setup

package_name = 'lidar_concat'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/concat.launch.py',
            'launch/kmutt_pipeline.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fibo5',
    maintainer_email='surachai@salvis.co.th',
    description='Approximate-time concatenation of 5 KMUTT LiDAR topics.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'concat_node = lidar_concat.concat_node:main',
        ],
    },
)
