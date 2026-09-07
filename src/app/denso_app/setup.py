from glob import glob

from setuptools import find_packages, setup

package_name = 'denso_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/frontend', glob('frontend/*')),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    package_data={'mock': ['config.yaml']},
    zip_safe=True,
    maintainer='tan',
    maintainer_email='chumanhtanxcxt@gmail.com',
    description='Web console for the PNK bimanual robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mock_server = mock.main:main',
        ],
    },
)
