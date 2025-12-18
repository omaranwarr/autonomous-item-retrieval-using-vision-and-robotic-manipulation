from setuptools import setup

package_name = 'panda_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot7',
    maintainer_email='robot7@example.com',
    description='Planning scene setup for Panda robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'setup_scene = panda_scene.setup_scene:main',
        ],
    },
)

