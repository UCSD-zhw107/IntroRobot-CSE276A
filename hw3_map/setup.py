from setuptools import setup

package_name = 'hw3_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[     
        'hw3_map.mpi_control', 
        'hw3_map.control_node',
        'hw3_map.slam'
        'hw3_map.kalman'
        'hw3_map.pds'
        'hw3_map.utils'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='wzy2575765661@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam = hw3_map.slam:main',
            'control_node = hw3_map.control_node:main'
        ],
    },
)
