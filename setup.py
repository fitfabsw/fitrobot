from setuptools import setup
from glob import glob

package_name = 'fitrobot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ['package.xml']),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/common", glob("common/*.py")),
        (f"share/{package_name}/script", glob("script/*.py")),
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
            'server_node = fitrobot.server_node:main',
            'bridge_node = fitrobot.bridge_node:main',
            'master_service = fitrobot.master_service:main',
            'save_map_service = fitrobot.save_map_service:main',
            'list_map_service = fitrobot.list_map_service:main',
            'waypoint_follower = fitrobot.waypoint_follower:main',
        ],
    },
)
