from setuptools import find_packages, setup

package_name = 'potential_field_control'

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
    maintainer='carsten',
    maintainer_email='cars2109@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'potential_field_control = potential_field_control.potential_field_control:main',
             'better_user_input = potential_field_control.better_user_input:main',
             'test = potential_field_control.test:main',
             
        ],
    },
)
