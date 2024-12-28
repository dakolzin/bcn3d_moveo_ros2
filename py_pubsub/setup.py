from setuptools import setup
import os
from glob import glob

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danil',
    maintainer_email='podkolzindanil@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy = py_pubsub.joy_teleop:main',
            'go_go_bcn3d = py_pubsub.go_go_bcn3d:main',
            'go_go_bcn3d_tcp = py_pubsub.go_go_bcn3d_tcp:main',
            'go_go_fsr_topic = py_pubsub.go_go_fsr_topic:main',
            'go_go_workspace = py_pubsub.go_go_workspace:main',
            'go_go_workspace_all = py_pubsub.go_go_workspace_all:main',
            'go_go_workspace_inside = py_pubsub.go_go_workspace_inside:main',
            'go_go_workspace_delaunay = py_pubsub.go_go_workspace_delaunay:main',
            'go_go_workspace_delaunay_inside = py_pubsub.go_go_workspace_delaunay_inside:main',
            'go_go_camera = py_pubsub.go_go_camera:main',
            'go_go_sub = py_pubsub.go_go_sub:main',
            'go_go_find = py_pubsub.go_go_find:main',
            'go_go_roma = py_pubsub.go_go_sub_with_roma:main',
            'go_go_rep = py_pubsub.go_go_rep:main',
            'go_go_point_to_grasp = py_pubsub.go_go_point_to_grasp:main',
            'go_go_point_to_grasp_and_marker = py_pubsub.go_go_point_to_grasp_and_marker:main',
            'go_go_topic_for_picking = py_pubsub.go_go_topic_for_picking:main',
            'publish = py_pubsub.publish:main',
        ],
    },
)
