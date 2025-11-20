# Third-Party Sources

This directory tracks external ROS 2 packages that are not committed to the repository.

## Ouster ROS 2 driver

To vendor the complete Ouster OS-series driver locally, use `vcs` to import the upstream repository:

```bash
vcs import < /path/to/Navigation/third_party/ros2_ouster.repos
```

After importing, build the workspace from the repository root:

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

The imported `ros2_ouster` package follows the standard BSD-licensed upstream layout and can be managed with normal `git` commands inside the `src` directory.
