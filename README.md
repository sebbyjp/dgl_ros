# Deep Grasp Library for ROS

As of right now, only GPD is implemented: https://github.com/atenpas/gpd

# Requirements
This has been tested with the following dependencies
1. Ubuntu 20.04
2. ROS2 Humble
3. C++ 17

# Usage
1. Follow [instructions](https://github.com/atenpas/gpd) to install GPD
2. Clone this library into your `$ROS_WS/src` directory: `git clone https://github.com/sebbyjp/dgl_ros.git`
3. Build this library from your `$ROS_WS` directory: ` colcon build --symlink-install --base-paths  src/dgl_ros/`
4. In a terminal, run `ros2 run ros_dgl  gpd_grasp_detection_server --ros-args -p src_topic:=$ROS_TOPIC_GENERATING_POINTCLOUDS -p gpd_config_path:=$PATH_TO_YOUR_GPD_CONFIG`
5. In a separate terminal, run `ros2 action send_goal ros_dgl_interfaces/action/SampleGraspPoses "{action_name: 'sample_grasp_poses'}"`

# Concepts

## Data Representations
**Src**: A ROS message typically published from simulation or ROS sensor control plugins (e.g. sensor_msgs/msg/Image, sensor_msgs/msg/Pointcloud2)

**Observation**: A ROS message that has all the data needed to input into a deep learning model

## Environment 
The following classes make up the `environment` interface in RL libraries. They are the bridge between
the real world and the learning model.

**Observer**: A ROS node that subscribes to a set of **Src** topics and stores a buffer of **Observation**s which
can be accessed through the `observer.Get()` and `observer.GetLatest()` member functions.

**Actor**: A ROS node that produces an action (e.g. a list of grasp poses) from an **Observer**. This class wraps a
deep learning model (representing the policy in RL) and packages the output into a ROS message. It is accessed via the [ROS action server](https://design.ros2.org/articles/actions.html) interface.

**Supervisor**: A ROS node that produces a label or reward signal for an **Observation** and ROS action. It is accessed
via the [ROS service](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) interface or the the `supervisor.supervise()` member function. 



# Design Decisions
- Objects should follow Reinforcement Learning concepts used by (OpenAI's gym)[https://gymnasium.farama.org/] and [Google's TF-Agents](https://github.com/tensorflow/agents). By committing to the RL framework, data is represented
as flexibly as possible and includes open-loop models and datasets like [Contact Graspnet](https://github.com/NVlabs/contact_graspnet).

- The primary interfaces for this library are ROS topics and services. This allows for easy integration with existing
ROS pipelines. Additionally, ROS messages can be accessed by either python or C++.

- To integrate a new deep learning model, minimum extra configuration should be required:
    - Define a new ROS action (e.g. `ros_dgl_interfaces/action/SampleGraspPoses.action`)
    - Define a new ROS message representing an **Observation**
    - Extend the `Agent` class and implement the `ObsFromSrcs()` and `ActionFromObs()` methods

# Roadmap
1. [Contact Graspnet](https://github.com/NVlabs/contact_graspnet) Availability
2. [Google's RT-X](https://robotics-transformer-x.github.io/) (Robot Transformers) Availability
3. "Supervisor" component to label samples with reward or success/failure 
4. "DataSetGenerator" component to create a dataset for offline learning
5. Switch to light-weight header-only library and load classes at runtime via C++ plugins.



