from setuptools import setup

package_name = 'tf_example'

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
    maintainer='openeuler',
    maintainer_email='openeuler@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_pub_node = tf_example.publish_tf:main',
            'tf_listener_node = tf_example.listen_tf:main'
        ],
    },
)
