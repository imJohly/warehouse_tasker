import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'warehouse_tasker'

# Iterate through all the files and subdirectories
# to build the data files array
def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ] + generate_data_files('share/' + package_name + '/', 'models')
    + generate_data_files('share/' + package_name + '/', 'launch')
    + generate_data_files('share/' + package_name + '/', 'params'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnson Ly',
    maintainer_email='johnson.ly@student.uts.edu.au',
    description='Warehouse task allocation system',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node         = warehouse_tasker.agent_node:main',
            'old_agent_node     = warehouse_tasker.old_agent_node:main',
            'mission_node       = warehouse_tasker.mission_node:main',
            'marker_mapper_node = warehouse_tasker.marker_mapper_node:main',
            'voronoi_graph_node = warehouse_tasker.voronoi_graph_node:main',
            'voronoi_grapher    = warehouse_tasker.voronoi_grapher:main',
            'drop_zone_broadcaster = warehouse_tasker.drop_zone_broadcaster:main'
        ],
    },
)
