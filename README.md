# hector_recorder (ROS 2 Humble Port)

![hector_recorder showcase](media/teaser.gif)

> **Note:** This is a port of hector_recorder for ROS 2 Humble. Some features available in the original Jazzy/Rolling version are not supported due to API limitations in Humble. See [Humble Limitations](#humble-limitations) for details.

## Changes from Original

This fork includes the following customizations:

- **Default storage format:** Changed from `sqlite3` to `mcap` for better performance and compatibility
- **Custom default output directory:** Supports `ROSBAGS_DIR` environment variable to set a default recording location across different machines (falls back to current working directory if not set)

A terminal UI for recording ROS2 bags (strongly inspired by [rosbag_fancy](https://github.com/xqms/rosbag_fancy)).

Some of its features:
- Displays message count, topic type, frequency, bandwith, duration, disk size...
- Adaptive display based on terminal window size
- Same arguments as ```rosbag2``` ```(ros2 bag record)```
- Specify arguments per command line or YAML file
- Publishes status topic (optional)

If you are familiar with `ros2 bag record`, you can use `hector_recorder` as a drop-in replacement with additional convenience.

## Requirements

- ROS 2 Humble
- ncurses
- fmt
- yaml-cpp
```
sudo apt update &&
sudo apt install libncurses-dev libfmt-dev libyaml-cpp-dev
```

## Build

```bash
# clone this repo into your ros2 workspace, then build:
colcon build --packages-select hector_recorder hector_recorder_msgs
source install/setup.bash
```

## Usage
  ```bash
  bag_recorder <args>
  ```
Place all ROS arguments at the end of the command line.
  ```bash
  bag_recorder <args> --ros-args -r __ns:=my_namespace
  ```
We support many ```ros2 bag record``` arguments as explained in the [official documentation](https://github.com/ros2/rosbag2?tab=readme-ov-file#record).
In addition, there is:

    --max-bag-size-gb   Specify the split size in GB instead of bytes
    --publish-status    If true, recorder stats will be published on a topic
    --config            Load all parameters from a YAML file (see below for more details)

### Humble Limitations

Due to ROS 2 Humble API limitations, the following features are **not available**:
- **Service recording** (`--services`, `--all-services`, `--exclude-services`)
- **Topic type filtering** (`--topic-types`, `--exclude-topic-types`)
- **Exclude topic lists** (`--exclude-topics`)
- **Manual bagfile splitting** (bagfiles split automatically based on size/duration settings)
- **Compression thread priority** (`--compression-threads-priority`)
- **Keyboard controls disable** (`--disable-keyboard-controls`)
- **Custom metadata** (`--custom-data`)
- **Start/end time constraints** (`start_time_ns`, `end_time_ns`)

Available filtering options in Humble:
- `--all` or `--all-topics`: Record all topics
- `--topics`: Specific topic list
- `--regex`: Topic name pattern matching (requires full path match - see below)
- `--exclude-regex` or `--exclude`: Exclude topics by pattern

**Regex Pattern Guide:**  
Unlike substring matching, regex patterns must describe the **complete topic path**. Use `.*` (match anything) as wildcards:

| Pattern | What it matches | Example matches |
|---------|----------------|-----------------|
| `.*/odom.*` | Topics containing `/odom` anywhere | `/robot/odom`, `/robot/odom/filtered` |
| `.*/tf.*` | Topics containing `/tf` anywhere | `/robot/tf`, `/robot/tf_static` |
| `/robot/sensors/.*` | All topics starting with `/robot/sensors/` | `/robot/sensors/imu`, `/robot/sensors/camera/image` |
| `.*/camera$` | Topics ending with `/camera` | `/front/camera`, `/back/camera` |

**Why the `.*` is needed:** The pattern must cover the entire topic name from beginning to end. Without `.*` before or after, you're only matching part of the path and the regex will fail.

### Examples
- Record everything (all topics):  
  ```bash
  bag_recorder --all
  # or
  bag_recorder --all-topics
  ```  

- Record specific topics:  
  ```bash
  bag_recorder --topics /tf /odom
  ```  

- Record topics matching a pattern:  
  ```bash
  # Record all topics with 'odom' in the name:
  bag_recorder --regex ".*/odom.*"
  
  # Record all topics under /robot namespace:
  bag_recorder --regex "/robot/.*"
  
  # Record all sensor topics:
  bag_recorder --regex ".*/sensors/.*"
  ```

- Exclude topics matching a pattern:  
  ```bash
  # Record all except diagnostics topics:
  bag_recorder --all --exclude-regex ".*/diagnostics.*"
  ```

- Load from YAML:  
  ```bash
  bag_recorder --config /path/to/config.yaml
  ```

### Config file
All arguments can be specified either via command line or in a config file.

Example (Humble-compatible):
```yaml
node_name: "my_node_name"     # defaults to 'hector_recorder'
output: "/tmp/bags"           # absolute path to output directory
# output:                     # if omitted entirely, uses $ROSBAGS_DIR env var (if set) or current directory
all_topics: true              # record all topics (ROS 2 Humble uses 'all_topics' or 'all')
# Or specify specific topics:
# topics: 
#  - "/tf" 
#  - "/odom"
max_bag_duration: 60          # split the bag at 60s
publish_status: true          # publish hector_recorder_msgs status
regex: ""                     # optional: regex pattern for topic names
exclude: ""                   # optional: exclude pattern (replaces exclude_regex in Humble)
```

**Note:** If the `output` field is **completely omitted** from the config file, it will use the `ROSBAGS_DIR` environment variable if set, otherwise the current working directory.

**Note for Humble:** In the config file, use `all_topics: true` which will be mapped to the Humble API's `all` field. Service recording and advanced exclusion options are not available.

See here for all available parameters and their default values:
[hector_recorder/config/default.yaml](hector_recorder/config/default.yaml)

### Directory resolution
- If ```--output/-o``` is not specified, a timestamped folder will be created in:
  - The directory specified by the `ROSBAGS_DIR` environment variable (if set), or
  - The current working directory (if `ROSBAGS_DIR` is not set)
- ```-o some_dir``` creates ```some_dir``` (works with absolute/relative paths)
- If you want to have timestamped bag files in a specified log dir (useful for automatic logging), you can append a slash:  
  ```-o some_dir/``` creates ```some_dir/rosbag_<stamp>```

**Setting a custom default directory:**

You can set the `ROSBAGS_DIR` environment variable to change the default output location:

```bash
# In your shell profile (~/.bashrc, ~/.zshrc, etc.):
export ROSBAGS_DIR="/workspace/rosbags"

# Or set it per-session:
export ROSBAGS_DIR="/my/custom/path"
bag_recorder --topics /tf /odom
```

### TODOs
- [ ] Add/test qos-profile-overrides-path


#### Acknowledgement
This project includes components from:
- ROS 2 [rosbag2](https://github.com/ros2/rosbag2)
- [CLI11](https://github.com/CLIUtils/CLI11) by Henry Schreiner (BSD-3-Clause)
