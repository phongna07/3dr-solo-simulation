from setuptools import setup

setup(
    name='solo_bringup', version='0.1.0', packages=['solo_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/solo_bringup']),
        ('share/solo_bringup', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Solo project team', maintainer_email='maintainer@example.invalid',
    description='Gazebo fleet runtime, readiness and flight supervision', license='Apache-2.0',
    entry_points={'console_scripts': [
        'readiness = solo_bringup.readiness:main',
    ]},
)
