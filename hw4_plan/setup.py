from setuptools import setup

package_name = 'hw4_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[     
        'hw4_plan.mpi_control', 
        'hw4_plan.mpi_twist_control_node',
        'hw4_plan.pbs',
        'hw4_plan.visibility',
        'hw4_plan.voronoi',
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
            'pbs = hw4_plan.pbs:main',
            'mpi_twist_control_node = hw4_plan.mpi_twist_control_node:main'
        ],
    },
)
