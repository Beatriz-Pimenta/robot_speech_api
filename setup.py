from setuptools import find_packages, setup

package_name = 'robot_speech_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch',
            ['launch/demo_speech.launch.py', 'launch/speech_with_server.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'demo_tts = robot_speech_api.demo_tts:main',
             'tts_speech = robot_speech_api.tts_api:main',
        ],
    },
)
