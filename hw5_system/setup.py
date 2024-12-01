from setuptools import setup

package_name = 'hw5_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'hw5_system.mpi_control',  
        'hw5_system.twist_send',        
        'hw5_system.plan_node',         
        'hw5_system.pose_estimate_node',
        'hw5_system.pid_node',
        'hw5_system.all_node',
        'hw5_system.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system.launch.py']),
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
            'pid_node = hw5_system.pid_node:main',
            'plan_node = hw5_system.plan_node:main',
            'twist_send = hw5_system.twist_send:main',
            'pose_estimate_node = hw5_system.pose_estimate_node:main',
            'all_node = hw5_system.all_node:main',
        ],
    },
)
