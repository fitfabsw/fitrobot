from setuptools import setup

package_name = 'fitrobot'

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
    maintainer='ubuntu',
    maintainer_email='jason.kj.ke@fit-foxconn.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_converter_node = fitrobot.tf_converter_node:main',
            'save_map_service = fitrobot.save_map_service:main',
            'save_map = fitrobot.save_map_client:main',
        ],
    },
)
