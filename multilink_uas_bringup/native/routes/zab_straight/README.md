# ZaB straight 100 m

FC/world path: (210,-60,-0.5) to (309.980006,-61.999600,-0.5) m.
Straight 100 m. Speed ramps 0→1 m/s in 2 s, holds 1 m/s for 98 s,
then ramps 1→0 m/s in 2 s: 102 simulation seconds for the route.
Takeoff, start alignment, final hover and landing are additional.

Collision meshes: route.json world checksum and mesh_sources.json.
At 0.05 m spacing, nearest collision-triangle distance minus 0.025 m
is at least 2.46095 m. Against the 1 m FC-centered robot envelope,
nominal surface margin is at least 1.46095 m. Floor/ceiling ray checks
confirm the route is inside the tunnel. Ascent/descent are not included
in that sphere certificate; ground contact is expected at both ends.

Regenerate: `python3 /root/uas_native/plan_zab_straight.py`.
Run once, record, RViz only, stop all after landing:

```bash
ros2 launch multilink_uas_bringup zab.launch.py headless:=true record:=true route:=/root/uas_native/routes/zab_straight
```

Default ZaB launch without route remains stationary and recording is OFF.
Flight control uses ground truth. MIMOSA LIO and ROVIO VIO are independent.
