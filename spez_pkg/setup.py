from setuptools import find_packages, setup

package_name = 'spez_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leonie',
    maintainer_email='leonie.bolt1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'dummycam = spez_pkg.dummycam:main',
        #'camera_pipeline = spez_pkg.camera_pipeline:main',
        'vlfm_pipeline = spez_pkg.vlfm_pipeline:main',
        'ros_pipeline = spez_pkg.ros_pipeline:main',
    ],
},

)
