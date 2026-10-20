#!/usr/bin/env python3
"""Build an isolated, opt-in exact iVox index replacement on the speed baseline."""
import argparse,hashlib,json,shlex,shutil,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
parser=argparse.ArgumentParser();parser.add_argument('--scratch',action='store_true');parser.add_argument('--pruned',action='store_true');parser.add_argument('--cached',action='store_true');parser.add_argument('--neighbors',action='store_true');parser.add_argument('--prefetch',action='store_true');args=parser.parse_args()
base=WS/'comparison_ws';source=base/'src/mimosa';prior=WS/'build/mimosa-speed';out=WS/('build/mimosa-compact-prefetch' if args.prefetch else 'build/mimosa-compact-neighbors' if args.neighbors else 'build/mimosa-compact-cached' if args.cached else 'build/mimosa-compact-pruned' if args.pruned else 'build/mimosa-compact-scratch' if args.scratch else 'build/mimosa-compact')
args.neighbors=args.neighbors or args.prefetch
out.mkdir(exist_ok=True);shutil.copytree(prior/'include',out/'include',dirs_exist_ok=True)
if args.scratch or args.pruned or args.neighbors:
 p=out/'include/mimosa/lidar/geometric_factor.hpp';s=p.read_text()
 replacements={
  '    MX3D points(config_.num_corres_points, 3);':'    thread_local MX3D points; points.resize(config_.num_corres_points, 3);',
  '    MX1D ones(config_.num_corres_points, 1);\n    ones.setOnes();':'    // Unused all-ones vector removed.',
  '    const MX3D centered = points.rowwise() - corres_means_target_[i].transpose();':'    thread_local MX3D centered; centered = points.rowwise() - corres_means_target_[i].transpose();',
  '        std::vector<size_t> k_indices(config_.num_corres_points);':'        thread_local std::vector<size_t> k_indices; k_indices.resize(config_.num_corres_points);',
  '        std::vector<double> sq_dists(config_.num_corres_points);':'        thread_local std::vector<double> sq_dists; sq_dists.resize(config_.num_corres_points);',
 }
 for before,after in replacements.items():
  assert s.count(before)==1,before
  s=s.replace(before,after)
 p.write_text(s)
if args.cached or args.neighbors:
 p=out/'include/mimosa/lidar/geometric_factor.hpp';s=p.read_text()
 s=s.replace('  mutable int linearize_count_;',"""  mutable int linearize_count_;
  // Exact-input memoization. No tolerance, skipped iteration or reduced data.
  mutable Eigen::Matrix4d cached_pose_ = Eigen::Matrix4d::Zero();
  mutable V3D cached_gravity_ = V3D::Zero();
  mutable std::shared_ptr<gtsam::GaussianFactor> cached_factor_;
""")
 needle='    const V3D local_z = delta_pose.rotation().inverse() * global_z;'
 assert s.count(needle)==1
 s=s.replace(needle,"""    if (!is_binary_ && !config_.drpm_enabled && cached_factor_ &&
        (cached_pose_.array() == delta_pose.matrix().array()).all() &&
        (cached_gravity_.array() == global_z.array()).all()) {
      return cached_factor_->clone();
    }
"""+needle)
 needle='      return std::make_shared<gtsam::HessianFactor>(\n        keys()[0], J_source_T_J_source, -J_source_T_b, f);'
 assert s.count(needle)==1
 s=s.replace(needle,"""      auto result = std::make_shared<gtsam::HessianFactor>(
        keys()[0], J_source_T_J_source, -J_source_T_b, f);
      if (!config_.drpm_enabled) {
        cached_pose_ = delta_pose.matrix(); cached_gravity_ = global_z;
        cached_factor_ = result->clone();
      }
      return result;""")
 p.write_text(s)
relative=Path('mimosa/lidar/incremental_voxel_map.hpp');header=(source/'include'/relative).read_text()
header=header.replace('#include <pcl/point_cloud.h>','#include <pcl/point_cloud.h>\n#include "performance/compact_ivox.hpp"')
needle='ivox_ = std::make_shared<gtsam_points::iVox>(*other.ivox_);'
assert header.count(needle)==1
header=header.replace(needle,'''if (const auto* compact=dynamic_cast<const uas_performance::CompactIVox*>(other.ivox_.get())) {
        ivox_=std::make_shared<uas_performance::CompactIVox>(*compact);
      } else { '''+needle+' }')
(out/'include'/relative).write_text(header)
cpp=(source/'src/lidar/incremental_voxel_map.cpp').read_text()
assert cpp.count('std::make_shared<gtsam_points::iVox>(leaf_size)')==1
cpp=cpp.replace('std::make_shared<gtsam_points::iVox>(leaf_size)','std::make_shared<uas_performance::CompactIVox>(leaf_size)')
(out/'incremental_voxel_map.cpp').write_text(cpp)
d=base/'build/mimosa/CMakeFiles/mimosa_lib.dir';flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
objects={}
for part,src in [('geometric',WS/'build/active_perception/geometric.cpp'),('incremental_voxel_map',out/'incremental_voxel_map.cpp')]:
 obj=out/(part+'.o');objects[part]=obj
 print('Compiling',part,flush=True)
 subprocess.run(['c++',*flags['CXX_DEFINES'],*(['-DUAS_IVOX_AABB=1','-DUAS_IVOX_STATIC5=1'] if args.pruned or args.neighbors else []),*(['-DUAS_IVOX_NEIGHBOR_CACHE=1'] if args.neighbors else []),*(['-DUAS_IVOX_PREFETCH=1'] if args.prefetch else []),f'-I{out}/include',f'-I{HERE}',*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(src),'-o',str(obj)],check=True)
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
replacements={'/lidar/geometric.cpp.o':objects['geometric'],'/lidar/incremental_voxel_map.cpp.o':objects['incremental_voxel_map'],
 '/lidar/drpm_registration.cpp.o':WS/'build/active_perception/drpm_registration.o','/imu/manager.cpp.o':prior/'imu_manager.o'}
cmd=[next((str(p) for suffix,p in replacements.items() if x.endswith(suffix)),x) for x in cmd]
subprocess.run(cmd,cwd=d.parents[1],check=True)
files=[out/'include/mimosa/lidar/geometric_factor.hpp',HERE/'performance/compact_ivox.hpp',out/'include'/relative,out/'incremental_voxel_map.cpp',out/'libmimosa_lib.so']
(out/'manifest.json').write_text(json.dumps(dict(change='Contiguous coordinate index; original point order, neighbor order, LRU and snapshot point sharing',scratch=args.scratch or args.pruned or args.neighbors,pruned=args.pruned or args.neighbors,cached=args.cached or args.neighbors,neighbors=args.neighbors,prefetch=args.prefetch,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}),indent=2)+'\n')
print(out,flush=True)
