from setuptools import setup

package_name = 'training_room'

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
    maintainer='yueqian',
    maintainer_email='yueqianliu@outlook.com',
    description='DRL training script',
    license='BSD, Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = training_room.main:main'
        ],
    },
)
