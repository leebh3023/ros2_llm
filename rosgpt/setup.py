from setuptools import find_packages, setup

package_name = 'rosgpt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'openai'],
    zip_safe=True,
    maintainer='lbh',
    maintainer_email='lbh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'client_input_node = rosgpt.client_input_node:main',
        'gpt_node = rosgpt.gpt_node:main',
        'rosgptparser_turtlesim = rosgpt.rosgptparser_turtlesim:main',
        ],
    },
)
