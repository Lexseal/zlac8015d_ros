from setuptools import find_packages, setup

package_name = 'zlac8015d_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transforms3d', 'numpy'],
    zip_safe=True,
    maintainer='xinsonglin',
    maintainer_email='zhongjiezhex12@me.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = zlac8015d_ros.driver:main'
        ],
    },
)
