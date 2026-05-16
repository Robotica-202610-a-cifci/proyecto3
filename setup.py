from setuptools import find_packages, setup

package_name = 'proyecto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', [
            'data/Escena-Problema1.txt',
            'data/Escena-Problema2.txt',
            'data/Escena-Problema3.txt',
            'data/Escena-Problema4.txt',
            'data/Escena-Problema5.txt',
            'data/Escena-Problema6.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devuser',
    maintainer_email='sinfonia@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation = proyecto.navigation_node:main',
        ],
    },
)
