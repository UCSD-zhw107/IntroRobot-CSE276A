from setuptools import setup

package_name = 'hw1_path'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'hw1_path.path_sender',      
        'hw1_path.key_parser', 
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',  
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='wzy2575765661@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_sender = hw1_path.path_sender:main'
        ],
    },
)
