# Development notes for the Kebni SensAItion driver
(To be removed before a pull request to the ArduPilot repo.)
Some of this information would be suitable to submit to the Ardupilot wiki as documentation, like that for [VectorNav](https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-external-ahrs-vectornav.rst).

## Notes
It's important to note that the Ardupilot documentation uses the term "AHRS" for a device that delivers a full navigation solution with both (GPS) position and attitude. This version of the SensAItion sensor, is called Inertial Navigation System (INS). The SensAItion sensor called Attitude and Heading Reference System (AHRS) only outputs the orientation of the platform.

## Ardupilot installation
How to clone Kebni's fork of Ardupilot and install it in Ubuntu 24.04 (optionally inside WSL2):

```bash
sudo apt update
sudo apt install git python3-pip python3-dev
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi # For cross-compilation to Cube Orange+
git clone https://github.com/kebni-dev/ardupilot_sensaition_driver.git
cd ardupilot_sensaition_driver
git submodule update --init --recursive
source Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile # Reload environment
```

## Build instructions
Before building, enable the External AHRS:

```bash
./waf configure --board CubeOrangePlus --enable-AHRS_EXT
```

Then build and upload at once:

```bash
./waf copter --upload --board CubeOrangePlus
```

## Connecting with MissionPlanner
If the Cube Orange+ connected via USB shows up as a "USB Serial Port" in the Windows Device Manager, Mission Planner cannot connect to it. If so, one needs to upgrade its firmware by opening the tab "Setup/Install Firmware" in Mission Planner and programming "Copter V4.6.2 OFFICIAL" to the Cube. After that, it should show up as "Cube Orange+ Mavlink" and "Cube Orange+ SLCAN" in the Device Manager. The Mavlink port works for Mission Planner to connect to in the "Auto" connection mode.

After that, use the file ```mission_planner_params.param``` for the Mission Planner parameter settings needed.
