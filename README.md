# Data Recording Workspace Setup
## Prerequisites
### ROV (Khadas SBC)

* Ubuntu 24.04 LTS
* ROS 2 Jazzy

Check:

```bash
lsb_release -a
ros2 --version
```

### PC
Same requirements as for ROV :)

---

## Copy Workspace to ROV

From PC:

```bash
scp -r ros2_ws khadas@192.168.42.12:~
```

Or copy manually and `cd` into it on the SBC.

---

## Build Workspace (ROV)

```bash
./scripts/build_ROV.bash
```

_NOTE:_ You might have to give permissions to scripts to execute, use command `sudo chmod 777 scripts/*` to give it necessary permissions for all users. Then `ls scripts/`, files should show as executable (green).

_NOTE:_ You **will** see warnings and stderr due to deprecated APIs. Build is OK if it ends with *all 10 packages built* (even with warnings).

Expected packages include:

* apos_interfaces
* bluerov_interfaces
* brov2_sonar
* bluerov2_pid
* brov2_barometer
* sentireader_ros2
* bluerov2_teleop
* brov2_interfaces
* pendulum_utils
* bluerov_actuator_driver

---

## Build Workspace on PC (Required)

```bash
./scripts/build_topside_PC.bash
```

Here you only build a **subset of packages**. This is required so the PC knows the **exact same ROS 2 message types** as the ROV. Some data published by the ROV uses **custom message definitions** (for example `bluerov2_interfaces`) that are **not part of standard ROS 2**. If these message packages are not built and sourced on the PC, ROS 2 cannot correctly record or replay the data. Building and sourcing these packages ensures message compatibility when recording bags on the PC.

---

## Xbox Controller

Required for teleoperation. Recomeneded to use USB, not Bluetooth as mapping on bluetooth is different for some reason :P
Controls:

* left stick = thrust/surge
* right stick = yaw
* D-PAD left/right toggles lights
* D-PAD up/down controls camera tilt
* **Y** enables actuators (arming)
* **X** disables actuators
* **A** toggles automatic depth hold (requires `/bluerov/odom`)
* **B** currently unused.

---

## Running

```bash
./scripts/ROV_startup_and_record_data_on_topside_PC.bash
```

Recorded data will be stored as ROS 2 bags.

---

## Playback Only (No Hardware Required)

After data has been recorded, it can be replayed without any hardware using the playback script.

```bash
./scripts/build_topside_PC.bash
```

This is required because ROS 2 cannot replay bags unless the custom ROV message definitions are built and sourced.

```bash
./scripts/data_playback.bash
```

By default, the script plays back the latest recorded dataset, but a specific dataset can be selected by passing it as an argument like this `./scripts/data_playback.bash <data_set_folder_name>`

---
