from setuptools import setup

package_name = 'py_srvcli'

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
    maintainer='ryanlin12',
    maintainer_email='rl386170@gmail.com',
    description='Client Server for setting background color and other things',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_color_service = py_srvcli.set_color_service:main',
            'set_color_client = py_srvcli.set_color_client:main',
            'clear_line_client = py_srvcli.clear_line_client:main',
            'clear_line_service = py_srvcli.clear_line_service:main',
            'set_background_color = py_srvcli.set_backgrnd_col:main'
        ],
    },
)
