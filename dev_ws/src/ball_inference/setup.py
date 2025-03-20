from setuptools import setup

package_name = 'ball_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/data", ['share/data/image1.png']),
        ('share/' + package_name + "/data", ['share/data/image2.png']),
        ('share/' + package_name + "/data", ['share/data/image3.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='joran.van.dolder@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_inference = ball_inference.ball_inference:main'
        ],
    },
)
