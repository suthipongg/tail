from distutils.core import setup
import sys

if sys.version_info[0] == 3:
    setup(
        name='bplprotocol',
        version='0.1dev',
        packages=['bplprotocol'],
        install_requires=['cobs', 'crcmod'],
        long_description=open('README.md').read(),
    )
else:
    setup(
        name='bplprotocol',
        version='0.2dev',
        package_dir={"": 'src27'},
        packages=['bplprotocol'],
        install_requires=['cobs', 'crcmod'],
        long_description=open('README.md').read(),
    )