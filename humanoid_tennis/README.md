# Humanoid Tennis

A humanoid robot simulation that plays tennis.

To build the code, run:
```bash
colcon build --symlink-install
```

Then, source the setup file:
```bash
source install/setup.bash
```
Note you might need to source from ~/robotws/install/setup.bash

To launch the simulation:
```bash
ros2 launch humanoid_tennis tennis_basic.launch.py
```

## Visualization

In RViz:
1. Add a **Marker** display.
2. Set the topic to `/swing_markers`.
3. Change the Fixed Frame to anything other than `world` (like `pelvis`).
