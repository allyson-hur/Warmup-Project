from setuptools import setup

package_name = 'Warmup-Project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elockwood',
    maintainer_email='elockwood@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop= Warmup-Project.teleop:main',
            'drive_square= Warmup-Project.drive_square:main',
            'wall_follower=Warmup-Project.wall_follower:main',
            'person_follower=Warmup-Project.person_follower:main',
            'obstacle_avoider=Warmup-Project.obstacle_avoider:main',
            'finite_state_controller=Warmup-Project.finite_state_controller:main'
        ],
    },
)
