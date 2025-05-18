from setuptools import find_packages, setup

package_name = 'line_mapper_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='trsr',
    maintainer_email='traserlaci@gmail.com',
    description='Line_mapper node for mikrobi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_mapper = line_mapper_pkg.line_mapper:main',
            'test_marker = line_mapper_pkg.test_marker:main',
            'line_follower_cnn = line_mapper_pkg.line_follower_cnn:main',
        ],
    },
)
