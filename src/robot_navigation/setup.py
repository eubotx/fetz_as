from setuptools import setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    mmaintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Custom navigation package',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_navigator = robot_navigation.simple_navigator:main',
        ],
    },
)