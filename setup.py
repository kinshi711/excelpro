from setuptools import setup

package_name = 'turtlebot_line_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'src.line_follower'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='User',
    author_email='user@example.com',
    description='A ROS 2 package for TurtleBot line following',
    license='BSD',
    entry_points={
        'console_scripts': [
            'line_follower = src.line_follower:main',
        ],
    },
)

