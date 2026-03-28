from setuptools import find_packages, setup

package_name = 'detections_visualizer'

setup(

    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/madama3.jpg']),
        ('share/' + package_name + '/config', ['resource/map_madama3_september.csv']),
    ],
    install_requires=['setuptools>61.0'],
    zip_safe=True,
    maintainer='ste',
    maintainer_email='stefano.bernagozzi@iit.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detections_visualizer = detections_visualizer.detections_visualizer:main'
        ],
    },
)
