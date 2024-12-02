from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'oakd_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.py')),
        (os.path.join('share', package_name, 'models'), 
         glob('models/*.py'))
    ],
    install_requires=['setuptools','depthai', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='yoheiyanagi',
    maintainer_email='yanagi0214yohei@gmail.com',
    description='oakd_pkg test',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_topic_test = oakd_pkg.image_topic_test:main',
            'yolo_detection= oakd_pkg.yolo_detection:main'
        ],
    },
)
