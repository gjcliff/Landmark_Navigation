from setuptools import setup

package_name = 'navigation_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/navigation_gui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rshima',
    maintainer_email='rintarohshima2023@u.northwestern.edu',
    description='This package contains a GUI that allows users to save landmarks, cancel navigation, and navigate to selected landmarks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_gui = navigation_gui.navigation_gui:main'
        ],
    },
)
