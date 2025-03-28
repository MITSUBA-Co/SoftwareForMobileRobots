from setuptools import setup
from setuptools.command.install import install as InstallCommand
import os
from glob import glob
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.info('build start')

package_name = 'mitsuba_gui'

setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/guisetting', ['guisetting/guisetting.csv']),
        ('share/' + package_name + '/paramsetting', glob('paramsetting/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_main = mitsuba_gui.gui_main:main',
            'motor_test = mitsuba_gui.motor_test:main',
            'param_set = mitsuba_gui.param_set:main',
            'map_save = mitsuba_gui.map_save:main',
            'rout_set = mitsuba_gui.route_set:main',
            'autonom_run = mitsuba_gui.autonom_run:main',
        ],
    },
)


#paramsettingフォルダのファイルをshareフォルダにシンボリックリンクでコピーする
def create_symlink(data_dir):
    current_dir = os.getcwd()
    build_dir = os.path.join('build', package_name)
    if not current_dir.endswith(build_dir):   #buildフォルダではない時（srcフォルダの時）
        base_dir = current_dir.rsplit('/src/', 1)[0]  #srcフォルダの親フォルダ
        target_dir = os.path.join(base_dir, 'install', package_name, 'share', package_name, data_dir)
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)
        source_dir = os.path.join(current_dir, data_dir)
        files = os.listdir(source_dir)
        for filename in files:
            source_file = os.path.join(source_dir, filename)
            target_file = os.path.join(target_dir, filename)
            if os.path.isfile(target_file):
                os.remove(target_file)
            os.symlink(source_file, target_file)

data_dir = 'paramsetting'
create_symlink(data_dir)  #データファイルをシンボリックリンクにする（コピーしたい場合はコメントにする）
