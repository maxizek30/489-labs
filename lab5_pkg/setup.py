from setuptools import find_packages, setup

package_name = 'lab5_pkg'

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
    maintainer='lopezgm',
    maintainer_email='familiacachorin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'w_node = lab5_pkg.go_to_goal_action_client:main',
                'Chaewon = lab5_pkg.go_to_goal_server:main'
            ],
    },
)
