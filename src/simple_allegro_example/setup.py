from setuptools import find_packages, setup

package_name = 'simple_allegro_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'viser'
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kh11kim@kaist.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_control = simple_allegro_example.simple_control:main'
        ],
    },
)
