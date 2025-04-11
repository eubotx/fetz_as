from setuptools import find_packages, setup

package_name = 'combat_strategizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/combat.launch.py']),
        ('share/' + package_name + '/launch', ['launch/letsgo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/letsgo_sim.launch.py']),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Decides combat strategy and sets goal_pose / path',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_attack = combat_strategizer.simple_attack:main',
            'weapon_control = combat_strategizer.weapon_control:main',
        ],
    },
)
