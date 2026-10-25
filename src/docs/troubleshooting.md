# Troubleshooting

This document collects the errors found while installing the repository on the laboratory laptop, together with the steps followed to solve them. It complements the installation steps described in the [Quick-start](../../README.md#quick-start).

> [!TIP]
> After applying any of the fixes below, rebuild the workspace and source it again:
> ```bash
> cd ~/workspace
> colcon build \
>   --allow-overriding ur_description ur_moveit_config ur_controllers ur_robot_driver ur_calibration ur_dashboard_msgs ur_bringup \
>   --cmake-args -DCMAKE_BUILD_TYPE=Release
> source ~/workspace/install/setup.bash
> ```

## Overview

| Stage | Error | Section |
|-------|-------|---------|
| Build | `tf2_kdl::tf2_kdl` not found | [tf2_kdl not found](#tf2_kdl-not-found) |
| Build | Compilation errors in `ur_controllers` | [Incompatibilities in ur_controllers](#incompatibilities-in-ur_controllers) |
| Build | `primary_client.h` not found | [Outdated ur_client_library](#outdated-ur_client_library) |
| System | `Could not get lock /var/lib/dpkg/lock-frontend` | [dpkg lock error](#dpkg-lock-error) |
| System | GPG key expired on `apt update` | [Expired ROS 2 GPG key](#expired-ros-2-gpg-key) |
| System | `permission denied` when running Docker | [Docker permission denied](#docker-permission-denied) |

---

## Build errors

### tf2_kdl not found

**Symptom**

```text
tf2_kdl::tf2_kdl not found
```

**Cause**

The `CMakeLists.txt` of `npam_trajectory` does not declare `tf2_kdl` as a CMake dependency, even though the package uses it internally.

**Solution**

Add the dependency manually:

```bash
sed -i 's/find_package(Boost REQUIRED COMPONENTS system filesystem)/find_package(Boost REQUIRED COMPONENTS system filesystem)\nfind_package(tf2_kdl REQUIRED)/' \
  ~/workspace/src/npam_trajectory/CMakeLists.txt
```

### Incompatibilities in ur_controllers

**Symptom**

The `ur_controllers` package fails to compile with API-related errors.

**Cause**

The `Universal_Robots_ROS2_Driver` included in the repository is version 2.9.0, while some system dependencies are installed in newer versions with incompatible API changes.

**Solution**

Patch the controller source file:

```bash
FILE=~/workspace/src/Universal_Robots_ROS2_Driver/ur_controllers/src/scaled_joint_trajectory_controller.cpp

sed -i 's/get_segment_tolerances(logger, params_)/get_segment_tolerances(params_)/' $FILE
sed -i 's/rt_has_pending_goal_ = false/rt_has_pending_goal_.writeFromNonRT(false)/g' $FILE
sed -i 's/!rt_has_pending_goal_/!(*rt_has_pending_goal_.readFromRT())/g' $FILE
sed -i 's/rt_has_pending_goal_ &&/(*rt_has_pending_goal_.readFromRT()) \&\&/g' $FILE
sed -i 's/!rt_is_holding_/!(*rt_is_holding_.readFromRT())/g' $FILE
sed -i 's/auto active_tol = active_tolerances_.readFromRT();/const auto \& active_tol = default_tolerances_;/' $FILE
sed -i 's/active_tol->/active_tol./g' $FILE
sed -i 's/!check_state_tolerance_per_joint/!joint_trajectory_controller::check_state_tolerance_per_joint/g' $FILE

python3 -c "
import re
with open('$FILE', 'r') as f:
    content = f.read()
content = re.sub(
    r'set_point_before_trajectory_msg\(([^,]+),\s*([^,]+),\s*joints_angle_wraparound_\)',
    r'set_point_before_trajectory_msg(\1, \2)',
    content
)
with open('$FILE', 'w') as f:
    f.write(content)
"
```

### Outdated ur_client_library

**Symptom**

```text
ur_client_library/primary/primary_client.h not found
```

**Cause**

The installed version of `ur_client_library` is too old.

**Solution**

Update the package:

```bash
sudo apt install ros-humble-ur-client-library -y
```

---

## System errors

### dpkg lock error

**Symptom**

```text
Could not get lock /var/lib/dpkg/lock-frontend
```

**Cause**

The system is running automatic updates in the background.

**Solution**

Wait a few minutes until they finish, then run the command again.

### Expired ROS 2 GPG key

**Symptom**

A "GPG key expired" error appears when running `sudo apt update`.

**Cause**

The GPG key of the ROS 2 repository has expired.

**Solution**

Renew the key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Docker permission denied

**Symptom**

A `permission denied` error appears when running Docker commands.

**Cause**

The current user does not have permission to use Docker.

**Solution**

Add the user to the `docker` group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

> [!NOTE]
> Adding the user to the group is only needed the first time, and a system restart may be required for the change to take effect.
