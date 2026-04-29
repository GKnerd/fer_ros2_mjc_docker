# High-Performance ROS 2 Stack: Zero-Copy Shared Memory (CycloneDDS + Iceoryx) in Docker

## -- Highly Experimental --

### The Architecture & The "Why"
Default Docker networking creates a virtual Layer 2 bridge (docker0), which forces every ROS 2 message through Network Address Translation (NAT). For MoveIt and ros2_control running a 1000 Hz real-time hardware loop, the serialization/deserialization and NAT routing introduce unacceptable CPU overhead and latency jitter. If a command packet misses the strict 1.0 ms window, real robot hardware (like the Franka arm) will trigger a safety fault and lock its brakes.

The Solution: We use CycloneDDS with Iceoryx to enable Zero-Copy Shared Memory (--ipc=host). This bypasses the OS network stack entirely for massive data structures (like JointTrajectory arrays), dropping latency to microseconds and CPU overhead to near 0%.

###  Step 1: Host Machine Preparation (Crucial)
CycloneDDS requires a massive receive buffer to reliably handle large trajectory messages without dropping packets. You must configure your Host OS kernel to allow this, or the Docker container will crash on boot.
```bash
sudo sysctl -w net.core.rmem_max=2147483647
```
>Note: To make this persist across reboots, add `net.core.rmem_max=2147483647` to your `/etc/sysctl.conf file`.

## Step 2: The Middleware Configuration (XML Profiles)
Because of undocumented schema changes in recent CycloneDDS versions, the official ROS 2 documentation XMLs will fail. We need two distinct XML profiles.

1. The Primary Shared Memory Profile (env/cyclone_dds.xml)
This is the modern, syntax-corrected profile that enables Iceoryx as a first-class citizen.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
        </Internal>
        <General>
            <MaxMessageSize>65535B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```
2. The UDP Fallback Profile (env/cyclone_dds_udp.xml)
This explicitly disables shared memory and forces standard loopback networking. (See The Mixed-DDS Strategy below for why this is necessary).

```bash
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
        </Internal>
        <General>
            <MaxMessageSize>65535B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
    </Domain>
</CycloneDDS>
```

### Step 3: Docker & Daemon Automation
To make shared memory work, a background daemon (iox-roudi) must allocate the RAM pools. Instead of manually opening a second terminal, we automate this in the Dockerfile using an `entrypoint.sh` script.

entrypoint.sh (Create this next to your Dockerfile):
```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash

echo "Starting Iceoryx RouDi Daemon..."
iox-roudi &
sleep 1 # Give RouDi 1 second to allocate the massive RAM pools

echo "RouDi is ready. Executing main command..."
exec "$@"
Dockerfile additions:
```

Dockerfile
```bash

...

# Copy the networking profiles
COPY env/cyclone_dds.xml /home/${USER}/cyclone_dds.xml
COPY env/cyclone_dds_udp.xml /home/${USER}/cyclone_dds_udp.xml
RUN chown ${USER}:${USER} /home/${USER}/cyclone_dds*.xml

# Set default DDS implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/${USER}/cyclone_dds.xml
...
# Automate the Daemon
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

**Docker Run Flags**: When running the container, you must use `--network=host` to bypass Docker's virtual bridge, and `--ipc=host` if communicating with nodes outside the container.

## Step 4: The Mixed-DDS Launch Strategy 

#### The Problem: 
Iceoryx requires knowing data typing/sizes at runtime to allocate memory blocks. Dynamically sized data types (like std::string and std::vector) cannot be passed via zero-copy operations. If a node tries to publish the massive URDF string (/robot_description) or a spawner expects a dynamic array back from the /list_controllers service, Iceoryx will reject it, the message will drop, and the entire launch file will hang indefinitely waiting for discovery.

#### The Fix: 
We use a "Mixed-DDS" architecture. CycloneDDS allows Node A (UDP) and Node B (Shared Memory) to talk to each other. We use the launch API's `additional_env` parameter to force offending nodes to use the UDP profile, leaving the heavy controllers on Iceoryx.

Example launch.py injection:

```python
    # The spawner uses std::vector for controller lists, which breaks Iceoryx.
    # We bypass this by forcing the spawner node to publish through standard UDP.
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=[name] + [a for a in args] + 
                        ["--param-file", controllers_yaml],
            # INJECT THE UDP PROFILE ONLY FOR THIS SPECIFIC NODE
            additional_env={'CYCLONEDDS_URI': 'file:///home/fer_ros2_sim/cyclone_dds_udp.xml'} 
        )

    # Note: Apply this same additional_env trick to `robot_state_publisher` 
```
