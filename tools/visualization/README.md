# HL_VTD RViz visualization

The controller writes visualization data without linking ROS into the driving
process. `rviz_bridge.py` watches those files and publishes two
`visualization_msgs/MarkerArray` topics.

## Run

Start a scenario with visualization output:

```bash
python3 tools/verification/run_scenario.py scenario/mixed/M09 \
  --run-id m09_rviz --auto-vtd --rviz-data
```

In a second terminal with ROS2 sourced:

```bash
source /opt/ros/humble/setup.bash
python3 tools/visualization/rviz_bridge.py \
  --data-dir output/verification/M09/m09_rviz/rviz
```

Open RViz in a third terminal (or background the bridge):

```bash
source /opt/ros/humble/setup.bash
rviz2 -d tools/visualization/hl_vtd.rviz
```

Replace `humble` with the installed ROS2 distribution.

## Color legend

- Wide transparent yellow: free-driving mission corridor
- Thin bright yellow: final gateway lanes
- Red spheres: checkpoints
- White/yellow map lines: lane boundaries, stop lines, road center lines
- Cyan box/arrow: Ego
- Purple/orange/red boxes: nearby/on-path/colliding objects
- Purple lane: evaluated lane candidate
- Green lane/path: selected lane-change candidate
- Red thin paths: rejected candidate paths
- Magenta paths: collision-free viable candidates
- Blue thick path: the local path currently sent to and tracked by the controller

The only floating text is a two-line Ego status panel.  It shows DRIVE/STOP,
the stop reason when stopped, and current/target speed.  Detailed object and
candidate rejection data remains in `latest.json` without cluttering RViz.
