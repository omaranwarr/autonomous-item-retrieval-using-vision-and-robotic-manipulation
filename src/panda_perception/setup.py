from setuptools import setup

package_name = 'panda_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot7',
    maintainer_email='robot7@example.com',
    description='YOLO perception node for Panda pick-place project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = panda_perception.yolo_node:main',
        ],
    },
)

