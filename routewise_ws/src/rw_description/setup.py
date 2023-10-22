from setuptools import setup, find_packages

package_name = 'rw_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the urdf directory and its contents
        ('share/' + package_name + '/urdf', ['rw_description/urdf/routewise_material.xacro',
                                              'rw_description/urdf/routewise_core.xacro', 
                                              'rw_description/urdf/routewise.urdf.xacro',
                                              'rw_description/urdf/geometry_check.bash']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='onur',
    maintainer_email='onurulusoys4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'geometry_check_node = rw_description.nodes.geometry_check_node:main',
        ],
    },
)
