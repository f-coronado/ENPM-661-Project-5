from setuptools import setup


package_name = 'test_turtlebot3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name, [package_name + '/angle_helpers.py']),
        # ('lib/' + package_name, [package_name + '/helper_functions.py']),
        # ('lib/' + package_name, [package_name + '/occupacy_field.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='landis',
    maintainer_email='osmith15@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
        'driving_terpbot = test_turtlebot3.driving_terpbot:main'
        ],
    },
)
