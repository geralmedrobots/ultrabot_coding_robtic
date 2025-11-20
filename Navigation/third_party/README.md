# Third-Party Sources

This directory tracks external ROS 2 packages that are not committed to the repository.

## Ouster ROS 2 driver

To vendor the complete Ouster OS-series driver locally, use `vcs` to import the upstream repository and its point cloud transport plugin dependency:

```bash
vcs import < /path/to/Navigation/third_party/ros2_ouster.repos
```

After importing, build the workspace from the repository root:

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

The imported `ros2_ouster` package follows the standard BSD-licensed upstream layout and can be managed with normal `git` commands inside the `src` directory.

## Intel RealSense ROS 2 driver

To vendor the RealSense driver from source (for D455 or other supported devices), import the upstream repository manifest and rebuild:

```bash
vcs import < /path/to/Navigation/third_party/realsense_ros.repos
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

The `realsense-ros` repository uses Apache-2.0 licensing and brings its own message definitions and launch files. The upstream README documents any hardware-specific firmware or udev setup that may be required.
