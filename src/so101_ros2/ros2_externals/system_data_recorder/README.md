# System Data Recorder (SDR)

A lifecycle node for recording multiple, distinct rosbag2 bags sessions, while simultaneously copying the split bag files to another location as each bag file is completed.
This is useful, for example, to copy bag data files to an external disc during recording as each data file is completed, rather than waiting until all data is recorded before copying the entire bag at once (an operation that can take a significant time if the bag is large).

The copying function requires that a maximum file size for bag files be enabled.
Otherwise no splitting will be performed and the files will not be copied until recording is terminated.

## Configuration

The SDR is configured using a YAML parameter file, which is loaded via a ROS 2 launch file. This is the recommended way to run the node.

#### Parameters

* `bag_name_prefix` (string): The base name for each bag file. A timestamp will be appended to this (e.g., `"recording"` becomes `"recording_2025-07-10_02-30-00"`).
* `copy_destination` (string): The parent directory where all recording session folders will be saved.
* `max_file_size` (int): The maximum size in bytes for an individual `.db3` file before it is split. A value greater than `0` is required for the copy-on-split feature to work.
* `topic_names` (string array): A list of topics to record.
* `topic_types` (string array): A corresponding list of message types for the topics.

#### Example `sdr_example.yaml`

```yaml
sdr:
  ros__parameters:
    # The base name for each bag file. A timestamp will be appended to this.
    bag_name_prefix: "robot_data"

    # The parent directory where all recording session folders will be saved.
    copy_destination: "/home/user/sdr_bags"

    # Maximum size of each individual .db3 file in bytes before splitting.
    max_file_size: 268435456  # 256 MiB

    # List of topics and their types to record.
    topic_names: [
      "/chatter",
      "/diagnostics"
    ]
    topic_types: [
      "std_msgs/msg/String",
      "diagnostic_msgs/msg/DiagnosticArray"
    ]
```

## Use

### Lifecycle management

Run the executable using a launch file that loads your parameters. For example:
`ros2 launch system_data_recorder sdr_example.launch.py`

In a separate terminal, use the lifecycle manager to control the node.

1. **Configure the node.**
This sets up the main session directory and prepares the node for recording. This is done only once.

```bash
ros2 lifecycle set /sdr configure
```

2. **Activate to start recording.**
This begins a new, timestamped bag recording.

```bash
ros2 lifecycle set /sdr activate
```

3. **Deactivate to stop recording.**
This finalizes the current bag and copies its files to the permanent destination.

```bash
ros2 lifecycle set /sdr deactivate
```

You can repeat steps 2 and 3 as many times as you like to create multiple, separate bag files within the same run.

4. **Cleanup the node.**
This stops the background threads and cleans up resources.

```bash
ros2 lifecycle set /sdr cleanup
```

5. **Shutdown the node.**

```bash
ros2 lifecycle set /sdr shutdown
```

### A Simple Test

1. In one terminal, start publishing data:

```bash
ros2 launch system_data_recorder sdr_example.launch.py
```

2. In another terminal, launch the SDR node with your parameters:

```bash
ros2 launch your_package_name your_launch_file.py
```

3. In a third terminal, configure the SDR, then activate it to start the first recording:

```bash
ros2 lifecycle set /sdr configure
ros2 lifecycle set /sdr activate
```

4. After some time, deactivate it to save the first bag:

```bash
ros2 lifecycle set /sdr deactivate
```

5. Activate it again to start a second, new recording:

```bash
ros2 lifecycle set /sdr activate
```

6. Deactivate and clean up:

```bash
ros2 lifecycle set /sdr deactivate
ros2 lifecycle set /sdr cleanup
```

Now, navigate to your `copy_destination` directory (e.g., `/home/user/sdr_bags`). You will find a main session folder (e.g., `sdr_session_2025-07-10_03-00-00`), and inside it will be two complete, timestamped bag directories from your two recording sessions.

You can play back one of the bags:

```bash
ros2 bag play /home/user/sdr_bags/sdr_session_.../robot_data_...
```

## Lifecycle Transition Behaviours

### on_configure

Creates the main, timestamped session directory that will contain all bags recorded during this run. It also starts the background file-copying thread. **It does not start any recording.**

### on_activate

Begins a **new, unique recording session**. It creates a timestamped bag, sets up a temporary storage location, initializes a `rosbag2::Writer`, and subscribes to the requested topics. Data begins being written to the temporary bag file.

### on_deactivate

**Finalizes the current recording session**. It unsubscribes from topics, closes the `rosbag2::Writer` (which flushes all data to disk and writes the `metadata.yaml` file), and queues the final bag files to be copied to their permanent destination within the main session folder.

### on_cleanup

Ensures the last active recording session is properly deactivated and finalized. It then gracefully shuts down the background copy thread.

### on_shutdown

Performs the same actions as `on_cleanup` before shutting down the node.


---

## Keyboard Commander Utility

For manual control and testing, an `SDRKeyboardCommander` node is available. This node listens for keyboard presses and sends the corresponding lifecycle transition requests to the `/sdr` node.


### Running

1.  In one terminal, run your `sdr` node:

    ```bash
    ros2 launch system_data_recorder sdr_example.launch.py
    ```

2.  In a second terminal, source your workspace and run the commander:

    ```bash
    ros2 run system_data_recorder sdr_commander
    ```

### Controls

Once the commander node is running and connected to the `/sdr` services, you can use the following keys to control the recorder:

| Key | Action | Lifecycle Transition |
| :--- | :--- | :--- |
| **c** | Configure | `CONFIGURE` |
| **a** | Activate | `ACTIVATE` (Starts recording) |
| **d** | Deactivate | `DEACTIVATE` (Pauses recording) |
| **l** | Cleanup | `CLEANUP` |
| **s** | Shutdown | `SHUTDOWN` |
| **g** | Get State | (Queries and prints the current state) |
| **h** | Help | (Prints the help menu) |
| **q** | Quit | (Shuts down the commander node) |

