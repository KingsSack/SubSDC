# Submersible Simulation Data Collector (SubSDC)

Runs and collects data from simulations using Gazebo and ArduSub.

## Installation

1. Clone the repository.
2. Setup submodules.
```bash
git submodule update --init --recursive
```

### Ubuntu (26.04)

```bash
# 1. Gazebo Jetty (https://gazebosim.org/docs/jetty/install_ubuntu)
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install gz-jetty

# 2. ArduPilot Gazebo plugin (libArduPilotPlugin.so)
sudo apt install libgz-sim10-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-gl

export GZ_VERSION=jetty
(cd ardupilot_gazebo && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4)

# 3. ArduSub SITL and Python setup
(cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh && . ~/.profile && ./waf configure --board sitl && ./waf sub)
```

### MacOS (27.0 and higher)

```zsh
# 1. Gazebo Jetty (https://gazebosim.org/docs/jetty/install_osx)
brew tap osrf/simulation
brew trust osrf/simulation
brew install gz-jetty

# 2. ArduPilot Gazebo Plugin (libArduPilotPlugin.so)
brew update
brew install rapidjson
brew install opencv gstreamer1

export GZ_VERSION=jetty
(cd ardupilot_gazebo && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4)

# 3. ArduSub SITL and Python setup
(cd ardupilot && sh ./Tools/environment_install/install-prereqs-mac.sh && ./waf configure --board sitl && ./waf sub)
```

## Usage

`run_tests.py` runs configurable open-loop manoeuvre tests on Gazebo models, flown by ArduSub SITL.
Per-phase metrics come from the simulator's ground truth in the SITL `.BIN` log.

```bash
python3 run_tests.py --layout bluerov2            # bluerov1 | bluerov2 | bluerov2-heavy
python3 run_tests.py --trials 1                   # number of trials to run
python3 run_tests.py --config my_test.json        # any CONFIG keys, e.g. {"mass_kg": 11, "mode": "STABILIZE"}
```

Each run writes `results/<layout>_<timestamp>/` containing:

- `config.json`: the settings used for the run
- `summary.csv`: per-phase metrics for every trial
- `truth.png`: plot of depth, speed, roll and pitch
- `trial_NN/`: per-trial output
  - `gazebo.log`, `sitl.log`: process output
  - `logs/*.BIN`: the full ArduSub log
  - `truth.csv`: ground truth at about 400 Hz

## Configuration

|Value|Default|Description|
|---|---|---|
|`layout`|`"bluerov1"`|The model, world, and frame config to use|
|`trials`|`1`|The number of trials to run|
|`mode`|`"MANUAL"`|The sub mode to use|
|`rate_hz`|`20`|`MANUAL_CONTROL` command send rate|
|`preflight_s`|`10`|Delay before arming|
|`mass_kg`|`None`|Overrides the model's default mass|
|`com_offset_m`|`None`|A list of offsets to the models default center of mass on each axis|
|`mission`|A set of basic commands to test the ROV in different conditions (settle, surge, sway, dive, rise, etc)|Commands to test each trial|
|`plugin_path`|`../arudpilot_gazebo/build`|Folder containing the built `libArduPilot.so` file|
|`results_dir`|`../results`|Folder to generate the results in|

## Layout

```
ardupilot/          ardupilot repository
ardupilot_gazebo/   ardupilot_gazebo repository
models/             bluerov1 (approximation, see models/bluerov1/BlueROV1.md), bluerov2, bluerov2_heavy
worlds/             <model>_underwater.world  (buoyancy, IMU, 1 ms physics; vehicle spawned at 2 m depth)
```

### Models

The default BlueROV2 and BlueROV2 Heavy models come from the
[bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz/) repository.
The BlueROV1 model was modified from the BlueROV2 model by
Claude Opus 5.5 following [these specifications](https://docs.bluerobotics.com/bluerov/).

### Worlds

The worlds don't need internet access and also load in Gazebo Harmonic. To watch a run, start
`gz sim -g` with the same `GZ_PARTITION`, or run the world by hand with
`GZ_SIM_RESOURCE_PATH=models gz sim -r worlds/<model>_underwater.world`.
