import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'imu'

setup(
	name=package_name,
	version='0.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
		 ['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share',package_name,'launch'), glob('launch/*.py')),

	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='ssyximmlove',
	maintainer_email='mail@supaku.cn',
	description='TODO: Package description',
	license='MIT',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			"main = imu.main:main",
			"imu_tf_pub = imu.tf_pub:main",
		],
	},
)
