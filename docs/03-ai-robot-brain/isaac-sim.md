---
sidebar_label: "3.1 Isaac Sim & Synthetic Data"
sidebar_position: 2
title: "Isaac Sim: Generating Synthetic Training Data"
description: "Create photorealistic synthetic datasets for robot perception models"
keywords: [isaac sim, synthetic data, replicator, domain randomization, training]
---

# 3.1 Isaac Sim & Synthetic Data

> *"Why collect 10,000 real images when you can generate 1,000,000 synthetic ones overnight?"*

---

## Learning Objectives

- Set up Isaac Sim for robotics simulation
- Use Replicator for synthetic data generation
- Apply domain randomization for robust models
- Export datasets in standard formats (COCO, KITTI)

---

## Why Synthetic Data?

| Challenge | Real Data | Synthetic Data |
|-----------|-----------|----------------|
| **Cost** | Expensive (cameras, labeling) | Cheap (compute only) |
| **Scale** | Limited by collection time | Unlimited |
| **Labels** | Manual annotation | Automatic ground truth |
| **Edge Cases** | Rare, hard to capture | Easy to generate |
| **Privacy** | Concerns with humans | No privacy issues |

---

## Isaac Sim Setup

```python
# Launching Isaac Sim from Python
from omni.isaac.kit import SimulationApp

# Configure simulation
config = {
    "headless": False,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracedLighting"
}

simulation_app = SimulationApp(config)

# Now import Isaac modules
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
```

---

## Loading Your Robot

```python
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Create simulation world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Load robot from URDF
from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False

result = urdf_interface.parse_urdf(
    "/path/to/pai_humanoid.urdf",
    import_config
)

# Spawn robot
robot_prim_path = urdf_interface.import_robot(
    "/path/to/pai_humanoid.urdf",
    import_config,
    "/World/Robot"
)
```

---

## Replicator: Data Generation Pipeline

```python
import omni.replicator.core as rep

# Setup camera for data capture
camera = rep.create.camera(
    position=(3, 3, 2),
    look_at=(0, 0, 0.5),
    focal_length=24
)

# Create render product
render_product = rep.create.render_product(camera, (640, 480))

# Setup writers for different annotation types
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/synthetic_dataset",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_camera=True,
    instance_segmentation=True
)
writer.attach([render_product])

# Define randomization
with rep.trigger.on_frame(num_frames=1000):
    # Randomize lighting
    with rep.create.light(
        light_type="Dome",
        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        intensity=rep.distribution.uniform(500, 2000)
    ):
        pass
    
    # Randomize object positions
    with rep.get.prims(semantics=[("class", "obstacle")]):
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0), (2, 2, 0)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
        )

# Run data generation
rep.orchestrator.run()
```

---

## Domain Randomization

```python
def setup_domain_randomization():
    """Configure randomization for training robustness"""
    
    # Texture randomization
    textures = [
        "/textures/wood_floor.png",
        "/textures/concrete.png",
        "/textures/carpet.png",
        "/textures/tiles.png"
    ]
    
    with rep.trigger.on_frame():
        # Randomize floor texture
        floor = rep.get.prims(path_pattern="/World/Ground")
        with floor:
            rep.randomizer.materials(
                materials=rep.create.material_omnipbr(
                    diffuse_texture=rep.distribution.choice(textures)
                )
            )
        
        # Randomize camera position (simulating different viewpoints)
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform(
                    (2, -1, 1.5),
                    (4, 1, 2.5)
                )
            )
        
        # Add motion blur randomly
        rep.modify.attribute(
            "motionBlurEnabled",
            rep.distribution.choice([True, False])
        )
```

---

## Exporting Datasets

### COCO Format

```python
# Use COCO writer for object detection
coco_writer = rep.WriterRegistry.get("COCOWriter")
coco_writer.initialize(
    output_dir="/data/coco_dataset",
    semantic_types=["class"],
    image_output_format="png"
)
coco_writer.attach([render_product])
```

### KITTI Format (for autonomous driving)

```python
kitti_writer = rep.WriterRegistry.get("KittiWriter")
kitti_writer.initialize(
    output_dir="/data/kitti_dataset",
    semantic_types=["class"],
    bbox_height_threshold=10
)
kitti_writer.attach([render_product])
```

---

## Complete Data Generation Script

```python
#!/usr/bin/env python3
"""
Synthetic Data Generation Pipeline for PAI Humanoid
"""

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from omni.isaac.core import World
import numpy as np

def generate_training_data(num_frames: int = 10000):
    """Generate synthetic training dataset"""
    
    # Initialize world
    world = World()
    world.scene.add_default_ground_plane()
    
    # Load environment assets
    rep.create.from_usd("/environments/warehouse.usd")
    
    # Create multiple cameras
    cameras = []
    for i, pos in enumerate([(3,0,2), (0,3,2), (-3,0,2), (0,-3,2)]):
        cam = rep.create.camera(
            name=f"cam_{i}",
            position=pos,
            look_at=(0, 0, 0.5)
        )
        cameras.append(cam)
    
    # Setup render products
    render_products = [
        rep.create.render_product(cam, (640, 480)) 
        for cam in cameras
    ]
    
    # Multi-format writer
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="/data/pai_dataset",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        depth=True,
        normals=True
    )
    writer.attach(render_products)
    
    # Object classes for detection
    CLASSES = ["robot", "human", "box", "chair", "table"]
    
    # Run generation
    with rep.trigger.on_frame(num_frames=num_frames):
        # Randomize environment
        setup_domain_randomization()
        
        # Spawn random obstacles
        for _ in range(np.random.randint(5, 15)):
            class_name = np.random.choice(CLASSES[2:])  # Random object
            rep.create.from_usd(
                f"/objects/{class_name}.usd",
                semantics=[("class", class_name)],
                position=rep.distribution.uniform((-4,-4,0), (4,4,0))
            )
    
    rep.orchestrator.run()
    print(f"Generated {num_frames} frames with annotations")


if __name__ == "__main__":
    generate_training_data(num_frames=10000)
    simulation_app.close()
```

---

## Exercises

### Exercise 3.1.1: Custom Objects
Add 5 custom 3D objects (furniture) to the randomization pool.

### Exercise 3.1.2: Weather Effects
Implement rain and fog domain randomization for outdoor scenes.

### Exercise 3.1.3: Human Poses
Generate data with animated human characters in various poses.

---

<div style={{textAlign: 'center', marginTop: '2rem'}}>

[← Back to Module 3](./index.md) | [Next: Visual SLAM →](./visual-slam.md)

</div>
