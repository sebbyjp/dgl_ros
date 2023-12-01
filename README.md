# Deep Grasp Library for ROS2

[![Code Coverage](https://codecov.io/gh/ros-planning/sebbyjp/dgl_ros/branch/main/graph/badge.svg?token=9225d677-c4f2-4607-a9dd-8c22446f13bc)](https://codecov.io/gh/sebbyjp/dgl_ros)

The primary philosophy behind this repo is that a full, deployable, and
usable AI solution extends beyond the AI model itself. **dgl_ros** is a light-weight C++ library for researchers to package their
deep grasp models into a ROS node, ready to be run and experimented with by the robotics community. **dgl_ros_models** contains examples of popular deep grasp models such as [GPD](https://github.com/atenpas/gpd) and
[Contact Graspnet](https://github.com/NVlabs/contact_graspnet) packed and ready to use.

# Overview
**dgl_ros** addresses the daunting heterogeneity in deep grasp model inputs, action spaces, and library dependencies through abstraction and composition. Specifically, we package a minimum solution as an *agent* that composes an *observer* service and an *actor* service. This is hardly enough, however, for any realistic use-case which requires an additional *supervisor* service to provide a label or reward signal for fine-tuning.

# Features
- Base class with two virtual methods, **ObsFromSrcs** and **ActionFromObserver**, to create a Ros2 action server for an arbitrary deep grasp model
- End-to-end working inference nodes for [GPD](https://github.com/atenpas/gpd) and
[Contact Graspnet](https://github.com/NVlabs/contact_graspnet)
- No configuration needed to run the provided deep grasp action servers outside of a yaml file.
- MoveIt Task Constructor stage to interface with the action server from a higher-level planning pipeline


## Coming Soon
1. Inference node for pretrained [Google's RT-X](https://robotics-transformer-x.github.io/) (Robot Transformers) 
2. Supervisor service to label samples with reward or success/failure 
3. DatasetGenerator class to create a dataset for offline learning in pytorch
4. Training module to collect and label data from parallel simulations
5. Python bindings

## Demos in Simulation and Real Life
<iframe width="560" height="315" src="https://www.youtube.com/embed/IMWCQYi5f5I?si=YJP1FD8orh7pKEAY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/3lCs003SFpo?si=53M_dCZ1zpryMCQY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

# Requirements
This has been tested with the following dependencies
1. Ubuntu 20.04
2. ROS2 Humble
3. C++ 17

# Example Usage
1. Follow [instructions](https://github.com/atenpas/gpd) to install GPD
2. Clone this library into your `$ROS_WS/src` directory: `git clone https://github.com/sebbyjp/dgl_ros.git`
3. Install dependencies: `rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y`
4. Build this library from your `$ROS_WS` directory: ` colcon build --symlink-install --base-paths  src/dgl_ros/`
5. In a terminal, run `ros2 run dgl_ros_models gpd --ros-args -p src_topic0:=$ROS_TOPIC_GENERATING_POINTCLOUDS -p gpd_config_path:=$PATH_TO_YOUR_GPD_CONFIG`

**Note:** Make sure you set `use_sim_time:=true` if you are running a simulation. For this example we send a goal from another terminal
so it should be omitted.

6. In a separate terminal, run `ros2 action send_goal /sample_grasp_poses dgl_ros_interfaces/action/SampleGraspPoses "{action_name: 'sample_grasp_poses'}"`

# Parameters for the action server node:
- src0_topic, src0_frame, ... srcN_topic, srcN_frame
- world_frame
- TODO: List in more detail


# Concepts
## Classes to Represent Data
**Src**: A ROS message typically published from simulation or ROS sensor control plugins (e.g. sensor_msgs/msg/Image, sensor_msgs/msg/Pointcloud2)

**Observation**: A ROS message produced from one or more **Src** messages that has all the data needed to input into a deep learning model.

## Classes to Interact with the Environment 
The following classes make up the `environment` interface in RL libraries such as[OpenAI's gym](https://gymnasium.farama.org/) and [Google's TF-Agents](https://github.com/tensorflow/agents). They are the bridge between the real world and the learning model.

**Observer**: A ROS node that subscribes to one or more **Src** topics, creates an **Observation** when requested, and stores a cache of **Observation**s which can be accessed by id.

**Actor**: A ROS action server that requests **Observation**s from an **Observer** and produces an action (e.g. a list of grasp poses) . This class wraps a
deep learning model (representing the policy in RL) and converts the into a ROS message. It is accessed via the [ROS action server](https://design.ros2.org/articles/actions.html) interface.

**Supervisor**: A ROS server that produces a label or reward signal for an **Observation** and ROS action. It is called
via the [ROS service](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) interface or directly through member functions. 


# Design Decisions
- To integrate a new deep learning model, minimum extra configuration should be required:
    - Define a new ROS action (e.g. `ros_dgl_interfaces/action/SampleGraspPoses.action`)
    - Define a new ROS message representing an **Observation**
    - Extend the `Agent` class and implement the `ObsFromSrcs()` and `ActionFromObs()` methods

- The primary interface for a *Deep Grasp Agent* is ROS2 topics and services. That way inference can be run by any compatible ROS2 project.

- Class structure should follow Reinforcement Learning concepts used by [OpenAI's gym](https://gymnasium.farama.org/) and [Google's TF-Agents](https://github.com/tensorflow/agents). By committing to the RL framework, data is represented
as flexibly as possible and includes open-loop models and datasets like [Contact Graspnet](https://github.com/NVlabs/contact_graspnet).



