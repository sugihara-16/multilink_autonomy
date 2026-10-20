#!/usr/bin/env python3
"""Recreate the isolated comparison workspace from pinned upstream revisions.

Invoke through comparison_ros1.sh, after the existing SLAM underlay is built.
Does not change the normal simulation or SLAM workspace.
"""
import json,shutil,subprocess,shlex
from pathlib import Path
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[3]/'comparison_ws';ASSETS=ROOT/'comparison'
REVISIONS=json.loads((ASSETS/'revisions.json').read_text())
def call(args,**kw):subprocess.run([str(a) for a in args],check=True,**kw)
def clone(name,url,path):
    if not path.exists():call(['git','clone',url,path]);call(['git','-C',path,'checkout',REVISIONS[name]['revision']])
    actual=subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()
    if actual!=REVISIONS[name]['revision']:raise RuntimeError(f'{name}: unexpected revision {actual}')
def patch(path,name):
    f=ASSETS/name
    if subprocess.run(['git','-C',str(path),'apply','--reverse','--check',str(f)],capture_output=True).returncode==0:return
    call(['git','-C',path,'apply','--check',f]);call(['git','-C',path,'apply',f])
def main():
    (WS/'src').mkdir(parents=True,exist_ok=True)
    for name,url,path in [
        ('drpm','https://github.com/ntnu-arl/drpm.git',WS/'drpm'),
        ('FAST-LIVO2','https://github.com/hku-mars/FAST-LIVO2.git',WS/'src/FAST-LIVO2'),
        ('rpg_vikit','https://github.com/xuankuzcr/rpg_vikit.git',WS/'src/rpg_vikit'),
        ('image_common','https://github.com/ros-perception/image_common.git',WS/'src/image_common'),
        ('Sophus','https://github.com/strasdat/Sophus.git',WS/'Sophus')]:clone(name,url,path)
    for path,name in [(WS/'src/FAST-LIVO2','fast-livo2.patch'),(WS/'src/rpg_vikit','vikit.patch'),(WS/'Sophus','sophus.patch')]:patch(path,name)
    patch(WS/'src/FAST-LIVO2','fast-livo2-multicamera.patch')
    shutil.copyfile(ASSETS/'synchronized_images.hpp',WS/'src/FAST-LIVO2/include/synchronized_images.hpp')
    shutil.copyfile(ASSETS/'fastlivo_multicamera.launch',WS/'src/FAST-LIVO2/launch/multicamera.launch')
    shutil.copyfile(ASSETS/'fastlivo_offline.cpp',WS/'src/FAST-LIVO2/src/offline.cpp')
    shutil.copyfile(ASSETS/'fastlivo_stream.cpp',WS/'src/FAST-LIVO2/src/stream.cpp')
    cmake=WS/'src/FAST-LIVO2/CMakeLists.txt'
    if 'add_executable(fastlivo_stream ' not in cmake.read_text():
        cmake.write_text(cmake.read_text()+'\nadd_executable(fastlivo_stream src/stream.cpp)\ntarget_link_libraries(fastlivo_stream laser_mapping vio lio pre imu_proc ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Boost_LIBRARIES})\n')
    mimosa=WS/'src/mimosa'
    if not mimosa.exists():
        shutil.copytree(ROOT.parents[3]/'slam_ros1_ws/src/mimosa/mimosa',mimosa)
        call(['patch','-p1','-i',ASSETS/'mimosa-drpm.patch'],cwd=mimosa)
    if 'drpm_enabled' not in (mimosa/'include/mimosa/lidar/geometric_config.hpp').read_text():raise RuntimeError('Existing comparison MIMOSA is missing the patch')
    patch(mimosa,'mimosa-odometry-covariance.patch')
    patch(mimosa,'mimosa-drpm-noise.patch')
    patch(mimosa,'mimosa-drpm-official.patch')
    for f in ['drpm_kernel.hpp','drpm_pose_factor.hpp','drpm_registration.hpp']:
        shutil.copyfile(ASSETS/f,mimosa/'include/mimosa/lidar'/f)
    shutil.copytree(ASSETS/'official_drpm',mimosa/'include/mimosa/lidar/official_drpm',dirs_exist_ok=True)
    shutil.copyfile(ASSETS/'drpm_registration.cpp',mimosa/'src/lidar/drpm_registration.cpp')
    shutil.copyfile(ASSETS/'drpm.hpp',mimosa/'include/mimosa/lidar/drpm.hpp')
    call(['cmake','-S',WS/'Sophus','-B',WS/'Sophus/build','-DCMAKE_BUILD_TYPE=Release',f'-DCMAKE_INSTALL_PREFIX={WS}/deps'])
    call(['cmake','--build',WS/'Sophus/build','-j2']);call(['cmake','--install',WS/'Sophus/build'])
    call(['c++','-std=c++17','-O2',ASSETS/'test_synchronized_images.cpp','-o',WS/'test_synchronized_images'])
    call([WS/'test_synchronized_images'])
    catkin='/root/.venvs/uas/bin/catkin'
    call([catkin,'config','--extend',ROOT.parents[3]/'slam_ros1_ws/devel','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DCMAKE_CXX_STANDARD=17','-DPYTHON_EXECUTABLE=/usr/bin/python3',f'-DSophus_DIR={WS}/Sophus/build','-DBUILD_TESTING=OFF'],cwd=WS)
    call([catkin,'build','mimosa','fast_livo','-j2','-p1','--no-status','--force-cmake'],cwd=WS)
    call(['c++','-O2','-I/usr/include/eigen3',ASSETS/'test_drpm.cpp','-o',WS/'test_drpm'])
    call([WS/'test_drpm'])
    call(['c++','-O2','-I/usr/include/eigen3',ASSETS/'test_drpm_noise.cpp','-o',WS/'test_drpm_noise'])
    call([WS/'test_drpm_noise'])
    underlay=ROOT.parents[3]/'slam_ros1_ws/devel'
    call(['c++','-O2','-I/usr/include/eigen3',f'-I{underlay}/include',ASSETS/'test_drpm_map_covariance.cpp',f'-L{underlay}/lib',f'-Wl,-rpath,{underlay}/lib','-lgtsam_points','-o',WS/'test_drpm_map_covariance'])
    call([WS/'test_drpm_map_covariance'])
    call(['c++','-O2','-I/usr/include/eigen3',ASSETS/'test_drpm_kernel.cpp','-o',WS/'test_drpm_kernel'])
    call([WS/'test_drpm_kernel'])
    call(['c++','-O2','-I/usr/include/eigen3',f'-I{underlay}/include',ASSETS/'test_drpm_pose_factor.cpp',f'-L{underlay}/lib',f'-Wl,-rpath,{underlay}/lib','-lgtsam','-ltbb','-o',WS/'test_drpm_pose_factor'])
    call([WS/'test_drpm_pose_factor'])
    flags=(WS/'build/mimosa/CMakeFiles/mimosa_rosbag.dir/flags.make').read_text()
    inc=next(line.split('=',1)[1] for line in flags.splitlines() if line.startswith('CXX_INCLUDES ='))
    call(['c++','-O1','-std=c++17','-fopenmp',*shlex.split(inc),'-c',ASSETS/'test_drpm_registration.cpp','-o',WS/'test_drpm_registration.o'])
    call(['c++',WS/'test_drpm_registration.o',f'-L{WS}/devel/lib',f'-L{underlay}/lib','-L/root/ros1_noetic/install_isolated/lib',f'-Wl,-rpath,{WS}/devel/lib','-lmimosa_lib','-lgtsam_points','-lgtsam','-ltbb','-lpcl_common','-lfmt','-lrosconsole','-lrostime','-lroscpp','-lconfig_utilities','-o',WS/'test_drpm_registration'])
    call([WS/'test_drpm_registration'])
if __name__=='__main__':main()
