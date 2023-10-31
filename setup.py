from setuptools import setup
# from glob import glob

package_name = 'ros_introspection'

setup(
    name=package_name,
    package_dir={'': 'src'},
    version='1.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    scripts=[
        'scripts/print_packages.py',
        'scripts/print_cmake_order_info',
        'scripts/print_cmake_parsing',
    ],
    install_requires=['setuptools'],
)
