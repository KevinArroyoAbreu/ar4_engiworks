from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'opencv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Add this line to install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karroyabreu',
    maintainer_email='karroyoabreu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_image_from_camera = opencv_pkg.load_image_from_camera:main',
            'load_image = opencv_pkg.load_image_from_camera:main',
            'color_filtering = opencv_pkg.color_filtering:main',
            'edge_detection = opencv_pkg.edge_detection:main',
            'stream_camera = opencv_pkg.stream_camera:main',
            'write_camera = opencv_pkg.write_camera:main',
            'stream_top_position = opencv_pkg.stream_top_position:main',
            'get_position_server = opencv_pkg.get_position_server:main',
            'position_server_v2 = opencv_pkg.position_server_v2:main',
        ],
    },
)
