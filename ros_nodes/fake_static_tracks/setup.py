from setuptools import setup

package_name = 'fake_static_tracks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@TODO.com',
    description='Fake publisher for static_tracks (DetectionsArray)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_static_tracks_pub = fake_static_tracks.fake_static_tracks_publisher:main',
        ],
    },
)
