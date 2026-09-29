from setuptools import setup

setup(
    name='ssb_tools',
    version='0.1.0',
    packages=['ssb_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ssb_tools']),
        ('share/ssb_tools', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='dk.illidan@gmail.com',
    description='Independent references, reconstruction and validation for tunnel scan sessions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'validate_stage_a = ssb_tools.validate_stage_a:main',
    ]},
)
