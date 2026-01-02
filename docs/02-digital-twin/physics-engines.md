---
sidebar_label: "2.1 Physics Engines (Gazebo)"
sidebar_position: 2
title: "Physics Engines: Configuring Gravity, Friction and Collision"
description: "Deep dive into physics simulation with Gazebo Fortress"
keywords: [gazebo, physics, simulation, collision, friction, ode, dart]
---

# 2.1 Physics Engines in Gazebo

> *"Good physics simulation is the difference between a robot that falls gracefully and one that clips through the floor."*

---

## Learning Objectives

- Understand physics engine options (ODE, DART, Bullet)
- Configure gravity, friction, and contact parameters
- Create collision geometries for accurate physics
- Build a complete Gazebo world file

---

## Gazebo Fortress Overview

**Gazebo** (formerly Ignition) is the de-facto ROS 2 simulation platform:

```bash
# Install Gazebo Fortress
sudo apt install ros-humble-ros-gz

# Launch empty world
gz sim empty.sdf

# Launch with ROS 2 bridge
ros2 launch ros_gz_sim gz_sim.launch.py world_sdf_file:=my_world.sdf
```

---

## Physics Configuration

### World Physics Settings

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="robot_world">
    
    <!-- Physics Engine Configuration -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      
      <!-- DART-specific settings -->
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>
    
    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>
    
    <!-- Magnetic field (for compass sensors) -->
    <magnetic_field>5.5645e-6 22.8758e-6 -42.3884e-6</magnetic_field>
    
    <!-- Atmosphere -->
    <atmosphere type="adiabatic"/>
    
  </world>
</sdf>
```

### Physics Engine Comparison

| Engine | Pros | Cons | Use Case |
|--------|------|------|----------|
| **DART** | Accurate dynamics | Slower | Manipulation, walking |
| **ODE** | Fast, stable | Less accurate | Mobile robots |
| **Bullet** | Good collision | Contact issues | General purpose |

---

## Friction & Surface Properties

```xml
<model name="floor">
  <static>true</static>
  <link name="ground">
    <collision name="ground_collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      
      <!-- Surface properties -->
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>      <!-- Friction coefficient -->
            <mu2>0.8</mu2>    <!-- Secondary friction -->
            <slip1>0.0</slip1>
            <slip2>0.0</slip2>
          </ode>
          <torsional>
            <coefficient>0.5</coefficient>
          </torsional>
        </friction>
        
        <contact>
          <ode>
            <kp>1e6</kp>     <!-- Contact stiffness -->
            <kd>100</kd>     <!-- Contact damping -->
            <max_vel>0.1</max_vel>
            <min_depth>0.001</min_depth>
          </ode>
        </contact>
      </surface>
    </collision>
  </link>
</model>
```

:::tip Friction Tuning
Start with `mu = 0.5` for smooth surfaces and `mu = 1.0+` for rubber. Test by placing objects on slopes!
:::

---

## Complete World SDF

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="obstacle_world">
    
    <!-- Physics -->
    <physics name="physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <gravity>0 0 -9.81</gravity>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>
    
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 1 -->
    <model name="wall_1">
      <static>true</static>
      <pose>5 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 10 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 10 2</size></box></geometry>
          <material>
            <ambient>0.6 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Obstacle Box -->
    <model name="obstacle_1">
      <static>false</static>
      <pose>2 1 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
          <surface>
            <friction><ode><mu>0.6</mu></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
          <material><ambient>0.2 0.6 0.2 1</ambient></material>
        </visual>
      </link>
    </model>
    
    <!-- Cylinder Obstacle -->
    <model name="obstacle_2">
      <static>true</static>
      <pose>-2 2 0.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>1.5</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.6 1</ambient></material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

---

## Spawning Your Robot

```bash
# Spawn URDF robot into Gazebo
ros2 run ros_gz_sim create -file pai_humanoid.urdf -name pai_robot -z 1.0

# Or use ROS 2 spawn service
ros2 service call /world/obstacle_world/create \
  ros_gz_interfaces/srv/SpawnEntity \
  "{name: 'robot', sdf: '$(cat robot.sdf)'}"
```

---

## Exercises

### Exercise 2.1.1: Slope Test
Create a ramp at 30° and test different friction values. At what `mu` does a box start sliding?

### Exercise 2.1.2: Collision Shapes
Create a world with primitive shapes (box, sphere, cylinder) and mesh obstacles.

### Exercise 2.1.3: Dynamic Objects
Add pushable boxes that the robot can interact with.

---

<div style={{textAlign: 'center', marginTop: '2rem'}}>

[← Back to Module 2](./index.md) | [Next: Unity Rendering →](./unity-rendering.md)

</div>
