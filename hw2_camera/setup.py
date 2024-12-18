from setuptools import setup

package_name = 'hw2_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[     
        'hw2_camera.key_parser', 
        'hw2_camera.camera_control',
        'hw2_camera.PBS_node'
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
            'PBS_node = hw2_camera.PBS_node:main'
        ],
    },
)
