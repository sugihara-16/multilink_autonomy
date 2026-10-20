# ZaB single-camera LIVO comparison

Input camera: link3 (original cam1), rectified monochrome 480×360 at 20 Hz.
Runs: run2a_end and run1_tail_a2_prefix. Evaluation uses the recorded tracking
windows only; preceding IMU/images/clouds initialize every estimator. Raw
integer source timestamps are preserved. Ground truth is never estimator input.

All three MIMOSA conditions consume identical monocular ROVIO odometry and
pose covariance. ROVIO has 25 features and the previously validated
`gyro_matched_depth10` model. FAST-LIVO2 consumes the same image, IMU and cloud
and does not consume ROVIO. Both camera/IMU and LiDAR/IMU extrinsics are taken
from the saved simulation calibration, without fitting to ground truth.

MIMOSA keeps existing `mimosa_dragon_vio.yaml` weights and IMU settings.
OFF/ON/DRPM differ only in LiDAR degeneration handling. ON keeps translation
threshold 40 and rotation threshold 0. FAST-LIVO2 starts from upstream avia.yaml,
with input topics, source geometry, zero time offsets, LiDAR type and common
0.8–30 m range / point skip 2 / 0.5 m voxel settings adapted. Internal noise and
factor weights are algorithm-specific: this is a comparison of these configured
pipelines, not proof of the best accuracy attainable by each paper's method.

The Gazebo point cloud is instantaneous. Point timestamps remain zero relative
to scan header; no synthetic spinning time or motion distortion is introduced.

## Local changes
- MIMOSA: repair missing per-point Jacobian/residual storage in existing ON path.
- MIMOSA: handle zero-length deskew interval when synchronous VIO already has
  the pose at the instantaneous cloud time, without timestamp offsets.
- MIMOSA: save the final optimized pose after same-timestamp sensor updates;
  standard graph publishing suppresses repeated updates to the same state.
- DRPM: paper Eqs.14–23 (non-degeneracy probability), Eq.35 (normal-rotation
  covariance), Eq.28 (information matrix) integrated into the LiDAR graph factor.
  Reweight H and gradient in Hessian eigen-directions. No zero-motion regularizer
  is introduced because IMU/VIO already supply motion constraints. This is a
  factor-graph adaptation, not a reproduction of the paper's standalone ICP loop.
- DRPM uses point sigma 0.03 m and SNR factor 10, identically for both runs.
  The noise model is isotropic while Gazebo adds range noise; this modeling
  approximation and fixed five-neighbor normal estimates limit interpretation.
- FAST-LIVO2: ordered offline adapter around original callbacks; no ROS
  transport queues. Generic x86-64 Eigen/PCL ABI avoids invalid aligned frees.
- Dependencies: C++17 for current ROS libraries, old Sophus complex setter fix,
  and explicit Sophus linking. No changes to normal simulation/SLAM workspace.

## Reproduction
Use the system ROS environments, not an activated Python virtual environment.
Existing SLAM underlay must be built. `build_livo_comparison.py` pins revisions
and applies the included patches in `/root/multilink_autonomy_ws/comparison_ws`.
The existing catkin executable is used solely as a build tool.

```
env -i HOME=/root USER=root PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  bash /root/uas_native/comparison_ros1.sh python3 /root/uas_native/build_livo_comparison.py
```
Start a private ROS1 master on port11376, build `build_rovio_monocular.py 25`,
prepare inputs using `prepare_livo_comparison.py` in the ROS2 environment,
then run `replay_monocular_comparison.py` and `make_comparison_bag.py`.
Run `run_livo_comparison.py CONDITION run2a_end run1_tail_a2_prefix` in the ROS1
environment, in OFF → ON → FAST-LIVO2 → DRPM order. The commands, input lineage,
configuration and wall time are saved with each result. Replot with
`python3 /root/uas_native/plot_livo_comparison.py`.

Output: `native/logs/zab-offline/livo_comparison/`.
Numerical tests: `test_drpm.cpp`; noisy full-rank geometry must keep six axes,
noisy plane must suppress three, and the resulting information stays finite/PSD.
All results are single trials with one chosen camera. No outcome-based tuning,
scale fitting, trajectory-wide alignment, clipping, or hidden axis ranges.
