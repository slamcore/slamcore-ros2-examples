# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## Version `v1.0.0` (published on 2023/03/24)

### Added

- Create 3 Galactic example.
- TurtleBot 4 Standard and Lite Galactic example.
- `<robot>-dependencies.txt` files to make it easier to install required
  dependencies.
- Kobuki `bumper2pc` node, to include bumper events in the navigation costmaps.
- `static_transform_publisher_from_file.py` script to pull the
  `slamcore/base_link` to robot `base_link` transform from a provided yaml
  file at launch.

### Changed

- Reorganised metapackage - Each example is now its own package, with common
  files used by all packages in `slamcore_ros2_examples_common`.
- Changed tool to help build the examples in a Docker container from
  `Makefile` to [Taskfile](https://taskfile.dev/).
- Updated `kobuki_setup_comms_launch.py` to also launch a `cmd_vel_mux` and
  safety controller, which makes the robot stop and back up when it bumps into
  something.
- Updated Slamcore `JSON` config files for Slamcore's `v23.01` software release.

### Removed

- Removed `Makefile` in favour of `Taskfile.dist.yaml`.

## Version `v0.1.0` (published on 2022/01/31)

Initial release of the Nav2 Integration Demo
