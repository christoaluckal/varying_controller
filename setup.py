from setuptools import find_packages, setup
from glob import glob

package_name = 'varying_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christoa',
    maintainer_email='christo12aluckal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_syncer = varying_controller.sync:main',
            'run_test_signals = varying_controller.test_signals:main',
            'run_rand_generator = varying_controller.generate_rand_signal:main',
            'run_seq_generator = varying_controller.generate_seq_signal:main',
        ],
    },
)
