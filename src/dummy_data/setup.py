from setuptools import find_packages, setup

package_name = 'dummy_data'

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
    maintainer='swarn',
    maintainer_email='arlikararyan@gmail.com',
    description='Package for dummy data',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_tf_broadcaster = dummy_data.imu_tf_broadcaster:main',
        ],
    },
)
