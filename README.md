
# Nereid

DIPC simulator, estimator, and controller ROS2 modules in a Docker container.

## Getting Started

* Run setup script to build Docker image. ⚠ Warning: this takes a very long time.

* Open the container in vscode:
  1. press `F1`
  2. select `Remote-Containers: Open Folder in Container...`
  3. select this folder (don't use paths with symlinks)

* Select a build kit:
  1. press `F1`
  2. select `CMake: Scan for Kits`
  3. press `F1`
  4. select `CMake: Select a Kit`
  5. choose `GCC 7.4.0`

* Select a build type:
  1. press `F1`
  2. select `CMake: Select Variant`
  3. select `Debug`

* Build and install:
  1. press `F1`
  2. select `Cmake: Install`






## TODO

- make a ros app that takes msmt inputs and produces state estimate outputs

- use double inverted pendulum as test case

- make a second ros app for simming the pendulum

- make a third ros app for applying control inputs

- randomized u

- sdre

