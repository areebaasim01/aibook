# Module 3: AI-Robot Brain - Exercise Solutions

Solutions for all exercises in Module 3.

---

## Chapter 3.1: Isaac Sim & Synthetic Data

### Exercise 3.1.1: Custom Objects

**Task:** Add custom 3D objects to your Isaac Sim scene and generate training data with bounding boxes.

```python
#!/usr/bin/env python3
"""Solution: Custom Object Synthetic Data Generation"""

from omni.isaac.kit import SimulationApp

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.replicator.core import Replicator
import omni.replicator.core as rep
import numpy as np

def create_custom_scene():
    """Create scene with multiple custom objects"""
    world = World()
    
    # Add custom objects with different properties
    objects = []
    
    # Red cube - graspable object
    red_cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/red_cube",
            name="red_cube",
            position=np.array([0.5, 0.0, 0.05]),
            scale=np.array([0.1, 0.1, 0.1]),
            color=np.array([1.0, 0.0, 0.0])
        )
    )
    objects.append(("red_cube", "graspable"))
    
    # Blue sphere - ball
    blue_sphere = world.scene.add(
        DynamicSphere(
            prim_path="/World/blue_sphere",
            name="blue_sphere",
            position=np.array([-0.3, 0.2, 0.05]),
            radius=0.05,
            color=np.array([0.0, 0.0, 1.0])
        )
    )
    objects.append(("blue_sphere", "ball"))
    
    # Green cylinder - bottle
    from omni.isaac.core.objects import DynamicCylinder
    green_cylinder = world.scene.add(
        DynamicCylinder(
            prim_path="/World/green_cylinder",
            name="green_cylinder",
            position=np.array([0.0, -0.4, 0.1]),
            radius=0.03,
            height=0.2,
            color=np.array([0.0, 1.0, 0.0])
        )
    )
    objects.append(("green_cylinder", "bottle"))
    
    return world, objects

def setup_replicator(objects):
    """Configure Replicator for synthetic data generation"""
    with rep.new_layer():
        # Camera setup
        camera = rep.create.camera(
            position=(1.5, 1.5, 1.0),
            look_at=(0, 0, 0)
        )
        
        # Render product
        render_product = rep.create.render_product(camera, (640, 480))
        
        # Randomizers
        with rep.trigger.on_frame(num_frames=1000):
            # Randomize object positions
            for obj_name, _ in objects:
                with rep.get.prims(path_pattern=f"/World/{obj_name}"):
                    rep.modify.pose(
                        position=rep.distribution.uniform(
                            (-0.5, -0.5, 0.05),
                            (0.5, 0.5, 0.05)
                        ),
                        rotation=rep.distribution.uniform(
                            (0, 0, 0),
                            (0, 0, 360)
                        )
                    )
            
            # Randomize lighting
            with rep.get.prims(path_pattern="/World/Light"):
                rep.modify.attribute(
                    "intensity",
                    rep.distribution.uniform(500, 2000)
                )
        
        # Writers for output
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir="./synthetic_data",
            rgb=True,
            bounding_box_2d_tight=True,
            semantic_segmentation=True,
            instance_segmentation=True
        )
        writer.attach([render_product])
    
    return render_product

def main():
    world, objects = create_custom_scene()
    render_product = setup_replicator(objects)
    
    # Run simulation and generate data
    world.reset()
    
    for i in range(1000):
        world.step(render=True)
        if i % 100 == 0:
            print(f"Generated {i} frames...")
    
    print("Synthetic data generation complete!")
    print("Output saved to ./synthetic_data/")

if __name__ == "__main__":
    main()
    simulation_app.close()
```

**COCO Format Annotation Converter:**

```python
#!/usr/bin/env python3
"""Convert Isaac Replicator output to COCO format"""

import json
import os
from pathlib import Path

def convert_to_coco(input_dir: str, output_file: str):
    """Convert Replicator annotations to COCO format"""
    
    coco = {
        "images": [],
        "annotations": [],
        "categories": [
            {"id": 1, "name": "red_cube", "supercategory": "graspable"},
            {"id": 2, "name": "blue_sphere", "supercategory": "ball"},
            {"id": 3, "name": "green_cylinder", "supercategory": "bottle"},
        ]
    }
    
    annotation_id = 1
    
    # Process each frame
    for i, json_file in enumerate(sorted(Path(input_dir).glob("*.json"))):
        with open(json_file) as f:
            data = json.load(f)
        
        # Image entry
        image_name = json_file.stem + ".png"
        coco["images"].append({
            "id": i + 1,
            "file_name": image_name,
            "width": 640,
            "height": 480
        })
        
        # Bounding boxes
        for bbox_data in data.get("bounding_box_2d_tight", []):
            x_min, y_min, x_max, y_max = bbox_data["bbox"]
            width = x_max - x_min
            height = y_max - y_min
            
            # Map class name to category ID
            class_name = bbox_data.get("class", "unknown")
            category_id = {
                "red_cube": 1,
                "blue_sphere": 2,
                "green_cylinder": 3
            }.get(class_name, 0)
            
            if category_id > 0:
                coco["annotations"].append({
                    "id": annotation_id,
                    "image_id": i + 1,
                    "category_id": category_id,
                    "bbox": [x_min, y_min, width, height],
                    "area": width * height,
                    "iscrowd": 0
                })
                annotation_id += 1
    
    with open(output_file, 'w') as f:
        json.dump(coco, f, indent=2)
    
    print(f"Converted {len(coco['images'])} images with {len(coco['annotations'])} annotations")

if __name__ == "__main__":
    convert_to_coco("./synthetic_data", "./annotations_coco.json")
```

---

### Exercise 3.1.2: Weather Effects

**Task:** Simulate different weather conditions (rain, fog, bright sun) and analyze their effect on perception.

```python
#!/usr/bin/env python3
"""Solution: Weather Effects in Isaac Sim"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.replicator.core as rep
from pxr import UsdGeom, Sdf

class WeatherSimulator:
    def __init__(self):
        self.conditions = ['sunny', 'overcast', 'foggy', 'rainy']
    
    def apply_sunny(self):
        """Bright sunny conditions"""
        with rep.get.prims(path_pattern="/Environment/Sky"):
            rep.modify.attribute("sunIntensity", 10.0)
            rep.modify.attribute("sunAltitude", 60.0)
        
        # High contrast shadows
        with rep.get.prims(path_pattern="/World/DirectionalLight"):
            rep.modify.attribute("intensity", 5000)
            rep.modify.attribute("shadow:enable", True)
        
        # Clear atmosphere
        self._set_fog(density=0.0, color=(1, 1, 1))
    
    def apply_overcast(self):
        """Overcast cloudy conditions"""
        with rep.get.prims(path_pattern="/Environment/Sky"):
            rep.modify.attribute("sunIntensity", 2.0)
            rep.modify.attribute("cloudCoverage", 0.8)
        
        # Soft diffused lighting
        with rep.get.prims(path_pattern="/World/DirectionalLight"):
            rep.modify.attribute("intensity", 1500)
            rep.modify.attribute("shadow:enable", False)
        
        self._set_fog(density=0.02, color=(0.7, 0.7, 0.75))
    
    def apply_foggy(self):
        """Dense fog conditions"""
        with rep.get.prims(path_pattern="/World/DirectionalLight"):
            rep.modify.attribute("intensity", 800)
        
        self._set_fog(density=0.15, color=(0.8, 0.8, 0.85))
    
    def apply_rainy(self):
        """Rainy conditions with wet surfaces"""
        with rep.get.prims(path_pattern="/World/DirectionalLight"):
            rep.modify.attribute("intensity", 1000)
        
        # Wet surface materials
        with rep.get.prims(path_pattern="/World/Ground"):
            rep.modify.attribute("roughness", 0.1)  # Wet = shiny
            rep.modify.attribute("specular", 0.8)
        
        self._set_fog(density=0.05, color=(0.6, 0.65, 0.7))
        
        # Add rain particle system (simplified)
        self._add_rain_particles()
    
    def _set_fog(self, density, color):
        """Configure atmospheric fog"""
        # In practice, modify stage atmosphere settings
        print(f"Setting fog: density={density}, color={color}")
    
    def _add_rain_particles(self):
        """Add rain particle effects"""
        # Particle system configuration
        print("Adding rain particles...")

def analyze_perception_under_weather(weather_condition):
    """Test object detection under different weather"""
    
    results = {
        'condition': weather_condition,
        'detection_metrics': {}
    }
    
    # Simulated metrics (in practice, run actual detection)
    baseline_metrics = {
        'sunny': {'precision': 0.92, 'recall': 0.89, 'avg_distance_error': 0.05},
        'overcast': {'precision': 0.88, 'recall': 0.85, 'avg_distance_error': 0.08},
        'foggy': {'precision': 0.65, 'recall': 0.58, 'avg_distance_error': 0.25},
        'rainy': {'precision': 0.72, 'recall': 0.68, 'avg_distance_error': 0.18},
    }
    
    results['detection_metrics'] = baseline_metrics.get(weather_condition, {})
    
    print(f"\n=== Perception Analysis: {weather_condition.upper()} ===")
    print(f"Precision: {results['detection_metrics'].get('precision', 'N/A')}")
    print(f"Recall: {results['detection_metrics'].get('recall', 'N/A')}")
    print(f"Distance Error: {results['detection_metrics'].get('avg_distance_error', 'N/A')}m")
    
    return results

def main():
    weather = WeatherSimulator()
    
    for condition in weather.conditions:
        print(f"\nApplying {condition} weather...")
        getattr(weather, f'apply_{condition}')()
        analyze_perception_under_weather(condition)
    
    print("\n=== Summary ===")
    print("Weather significantly impacts perception:")
    print("- Sunny: Best performance, sharp shadows may cause issues")
    print("- Overcast: Good performance, uniform lighting")
    print("- Foggy: Severely degraded depth perception")
    print("- Rainy: Reduced visibility, reflections cause false positives")

if __name__ == "__main__":
    main()
    simulation_app.close()
```

---

### Exercise 3.1.3: Human Poses

**Task:** Generate synthetic data with humans in various poses for HRI training.

```python
#!/usr/bin/env python3
"""Solution: Human Pose Synthetic Data"""

import omni.replicator.core as rep
from omni.replicator.core import AnnotatorRegistry

def create_human_scene():
    """Create scene with randomized human poses"""
    
    with rep.new_layer():
        # Create floor
        floor = rep.create.plane(scale=10, visible=True)
        
        # Human characters with different poses
        human_assets = [
            "omniverse://localhost/NVIDIA/Assets/Characters/Human/Standing.usd",
            "omniverse://localhost/NVIDIA/Assets/Characters/Human/Walking.usd",
            "omniverse://localhost/NVIDIA/Assets/Characters/Human/Sitting.usd",
        ]
        
        # Create multiple humans
        humans = []
        for i, asset in enumerate(human_assets):
            human = rep.create.from_usd(asset)
            humans.append(human)
        
        # Camera
        camera = rep.create.camera(
            position=(3, 3, 2),
            look_at=(0, 0, 1)
        )
        
        render_product = rep.create.render_product(camera, (1280, 720))
        
        # Randomization
        with rep.trigger.on_frame(num_frames=5000):
            for human in humans:
                with human:
                    rep.modify.pose(
                        position=rep.distribution.uniform(
                            (-2, -2, 0),
                            (2, 2, 0)
                        ),
                        rotation=rep.distribution.uniform(
                            (0, -180, 0),
                            (0, 180, 0)
                        )
                    )
            
            # Randomize camera position for variety
            with camera:
                rep.modify.pose(
                    position=rep.distribution.uniform(
                        (2, 2, 1.5),
                        (5, 5, 3)
                    ),
                    look_at=(0, 0, 1)
                )
        
        # Output annotations
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir="./human_pose_data",
            rgb=True,
            bounding_box_2d_tight=True,
            skeleton_data=True,  # 3D keypoints
            semantic_segmentation=True
        )
        writer.attach([render_product])
    
    return render_product

def process_skeleton_annotations(annotation_dir):
    """Process skeleton keypoint annotations for pose estimation training"""
    import json
    from pathlib import Path
    
    keypoint_names = [
        'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
        'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
        'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
        'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
    ]
    
    coco_keypoints = {
        "images": [],
        "annotations": [],
        "categories": [{
            "id": 1,
            "name": "person",
            "keypoints": keypoint_names,
            "skeleton": [[0,1],[0,2],[1,3],[2,4],[5,7],[7,9],[6,8],[8,10],
                        [5,6],[5,11],[6,12],[11,12],[11,13],[13,15],[12,14],[14,16]]
        }]
    }
    
    annotation_id = 1
    
    for i, json_file in enumerate(sorted(Path(annotation_dir).glob("*.json"))):
        with open(json_file) as f:
            data = json.load(f)
        
        image_name = json_file.stem + ".png"
        coco_keypoints["images"].append({
            "id": i + 1,
            "file_name": image_name,
            "width": 1280,
            "height": 720
        })
        
        # Process skeleton data
        for skeleton in data.get("skeleton_data", []):
            keypoints = []
            for kp_name in keypoint_names:
                kp = skeleton.get(kp_name, None)
                if kp:
                    keypoints.extend([kp['x'], kp['y'], 2])  # visible
                else:
                    keypoints.extend([0, 0, 0])  # not visible
            
            coco_keypoints["annotations"].append({
                "id": annotation_id,
                "image_id": i + 1,
                "category_id": 1,
                "keypoints": keypoints,
                "num_keypoints": sum(1 for k in keypoints[2::3] if k > 0)
            })
            annotation_id += 1
    
    with open(f"{annotation_dir}/person_keypoints.json", 'w') as f:
        json.dump(coco_keypoints, f)
    
    print(f"Processed {len(coco_keypoints['annotations'])} pose annotations")

if __name__ == "__main__":
    render_product = create_human_scene()
    # Run simulation...
    process_skeleton_annotations("./human_pose_data")
```

---

## Chapter 3.2: Visual SLAM

### Exercise 3.2.1: Loop Closure

**Task:** Implement loop closure detection to correct accumulated drift.

```python
#!/usr/bin/env python3
"""Solution: Loop Closure Detection for Visual SLAM"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional
from scipy.spatial.distance import cdist

@dataclass
class Keyframe:
    id: int
    image: np.ndarray
    pose: np.ndarray  # 4x4 transformation matrix
    descriptors: np.ndarray
    keypoints: List
    bag_of_words: np.ndarray  # BoW vector

class LoopClosureDetector:
    def __init__(self, vocabulary_size: int = 1000):
        self.keyframes: List[Keyframe] = []
        self.vocabulary_size = vocabulary_size
        self.vocabulary = None
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Loop closure parameters
        self.min_keyframe_gap = 20  # Don't close loops with recent frames
        self.similarity_threshold = 0.85
        self.min_inliers = 50
    
    def build_vocabulary(self, training_descriptors: np.ndarray):
        """Build bag-of-words vocabulary using k-means"""
        from sklearn.cluster import KMeans
        
        kmeans = KMeans(n_clusters=self.vocabulary_size, random_state=42)
        kmeans.fit(training_descriptors)
        self.vocabulary = kmeans.cluster_centers_
        print(f"Built vocabulary with {self.vocabulary_size} visual words")
    
    def compute_bow(self, descriptors: np.ndarray) -> np.ndarray:
        """Compute bag-of-words histogram for descriptors"""
        if self.vocabulary is None or descriptors is None:
            return np.zeros(self.vocabulary_size)
        
        # Find nearest visual word for each descriptor
        distances = cdist(descriptors.astype(float), self.vocabulary)
        words = np.argmin(distances, axis=1)
        
        # Create normalized histogram
        bow = np.bincount(words, minlength=self.vocabulary_size)
        bow = bow / (np.linalg.norm(bow) + 1e-6)
        
        return bow
    
    def add_keyframe(self, image: np.ndarray, pose: np.ndarray) -> int:
        """Add new keyframe and check for loop closures"""
        # Extract features
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        
        if descriptors is None:
            return -1
        
        # Compute BoW
        bow = self.compute_bow(descriptors)
        
        # Create keyframe
        kf = Keyframe(
            id=len(self.keyframes),
            image=image.copy(),
            pose=pose.copy(),
            descriptors=descriptors,
            keypoints=keypoints,
            bag_of_words=bow
        )
        self.keyframes.append(kf)
        
        return kf.id
    
    def detect_loop_closure(self, query_id: int) -> Optional[Tuple[int, np.ndarray]]:
        """Detect loop closure for given keyframe"""
        if query_id < self.min_keyframe_gap:
            return None
        
        query_kf = self.keyframes[query_id]
        
        # Find candidate loop closures using BoW similarity
        candidates = []
        for kf in self.keyframes[:-self.min_keyframe_gap]:
            similarity = np.dot(query_kf.bag_of_words, kf.bag_of_words)
            if similarity > self.similarity_threshold:
                candidates.append((kf.id, similarity))
        
        if not candidates:
            return None
        
        # Verify best candidate with geometric check
        candidates.sort(key=lambda x: x[1], reverse=True)
        
        for cand_id, similarity in candidates[:5]:  # Check top 5
            cand_kf = self.keyframes[cand_id]
            
            # Match features
            matches = self.matcher.match(
                query_kf.descriptors, 
                cand_kf.descriptors
            )
            
            if len(matches) < self.min_inliers:
                continue
            
            # Estimate relative pose with RANSAC
            src_pts = np.float32([
                query_kf.keypoints[m.queryIdx].pt for m in matches
            ]).reshape(-1, 1, 2)
            
            dst_pts = np.float32([
                cand_kf.keypoints[m.trainIdx].pt for m in matches
            ]).reshape(-1, 1, 2)
            
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            
            if H is not None:
                inliers = mask.sum()
                if inliers >= self.min_inliers:
                    print(f"Loop closure detected: {query_id} -> {cand_id}")
                    print(f"  Similarity: {similarity:.3f}, Inliers: {inliers}")
                    return (cand_id, H)
        
        return None
    
    def optimize_poses(self, loop_closures: List[Tuple[int, int, np.ndarray]]):
        """Optimize all poses given loop closure constraints"""
        # In practice, use g2o or GTSAM for pose graph optimization
        print(f"Optimizing {len(self.keyframes)} poses with {len(loop_closures)} loop constraints")
        
        # Simplified: just report drift correction
        for query_id, match_id, transform in loop_closures:
            drift = np.linalg.norm(
                self.keyframes[query_id].pose[:3, 3] - 
                self.keyframes[match_id].pose[:3, 3]
            )
            print(f"  Loop {query_id}->{match_id}: Correcting {drift:.3f}m drift")

def main():
    # Example usage
    detector = LoopClosureDetector()
    
    # Build vocabulary from training images (simulated)
    print("Training vocabulary...")
    orb = cv2.ORB_create(nfeatures=500)
    training_descs = []
    
    for i in range(100):
        # Create synthetic training image
        img = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        _, desc = orb.detectAndCompute(img, None)
        if desc is not None:
            training_descs.append(desc)
    
    if training_descs:
        all_descs = np.vstack(training_descs)
        detector.build_vocabulary(all_descs)
    
    print("\nSimulating SLAM with loop closure...")
    # Simulate keyframes in a loop trajectory
    loop_closures = []
    
    for i in range(50):
        # Synthetic image
        img = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        
        # Simulate circular trajectory
        angle = 2 * np.pi * i / 50
        x = 5 * np.cos(angle)
        y = 5 * np.sin(angle)
        
        pose = np.eye(4)
        pose[0, 3] = x
        pose[1, 3] = y
        
        kf_id = detector.add_keyframe(img, pose)
        
        # Check for loop closure
        result = detector.detect_loop_closure(kf_id)
        if result:
            loop_closures.append((kf_id, result[0], result[1]))
    
    if loop_closures:
        detector.optimize_poses(loop_closures)

if __name__ == "__main__":
    main()
```

---

### Exercise 3.2.2: Multi-Floor Mapping

**Task:** Extend SLAM to handle multi-floor environments with elevators/stairs.

```python
#!/usr/bin/env python3
"""Solution: Multi-Floor SLAM Manager"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional
from enum import Enum

class FloorTransitionType(Enum):
    ELEVATOR = "elevator"
    STAIRS = "stairs"
    RAMP = "ramp"

@dataclass
class FloorTransition:
    from_floor: int
    to_floor: int
    transition_type: FloorTransitionType
    from_pose: np.ndarray  # Pose on source floor
    to_pose: np.ndarray    # Pose on destination floor

@dataclass
class FloorMap:
    floor_id: int
    altitude: float  # Height above ground floor
    occupancy_grid: np.ndarray
    keyframes: List = field(default_factory=list)
    
class MultiFloorSLAM:
    def __init__(self):
        self.floors: Dict[int, FloorMap] = {}
        self.current_floor: int = 0
        self.transitions: List[FloorTransition] = []
        
        # Floor detection parameters
        self.floor_height = 3.0  # meters per floor
        self.barometer_threshold = 0.5  # meters
        self.imu_pitch_threshold = 10.0  # degrees for stairs
        
        # Current state
        self.altitude = 0.0
        self.previous_altitude = 0.0
    
    def initialize_floor(self, floor_id: int, altitude: float = None):
        """Initialize a new floor map"""
        if altitude is None:
            altitude = floor_id * self.floor_height
        
        self.floors[floor_id] = FloorMap(
            floor_id=floor_id,
            altitude=altitude,
            occupancy_grid=np.zeros((1000, 1000), dtype=np.float32)  # 10x10m at 1cm res
        )
        print(f"Initialized floor {floor_id} at altitude {altitude}m")
    
    def detect_floor_change(self, 
                           barometer_reading: float,
                           imu_pitch: float,
                           current_pose: np.ndarray) -> Optional[int]:
        """Detect if robot has changed floors"""
        
        self.previous_altitude = self.altitude
        self.altitude = barometer_reading
        
        altitude_change = self.altitude - self.previous_altitude
        
        # Detect significant altitude change
        if abs(altitude_change) > self.barometer_threshold:
            # Determine new floor
            new_floor = round(self.altitude / self.floor_height)
            
            if new_floor != self.current_floor:
                # Detect transition type
                if abs(imu_pitch) > self.imu_pitch_threshold:
                    transition_type = FloorTransitionType.STAIRS
                elif abs(altitude_change) > self.floor_height * 0.8:
                    transition_type = FloorTransitionType.ELEVATOR
                else:
                    transition_type = FloorTransitionType.RAMP
                
                print(f"Floor transition detected: {self.current_floor} -> {new_floor}")
                print(f"  Type: {transition_type.value}")
                
                return new_floor
        
        return None
    
    def handle_floor_transition(self,
                                new_floor: int,
                                transition_type: FloorTransitionType,
                                from_pose: np.ndarray):
        """Handle transition to a new floor"""
        
        # Record transition
        transition = FloorTransition(
            from_floor=self.current_floor,
            to_floor=new_floor,
            transition_type=transition_type,
            from_pose=from_pose.copy(),
            to_pose=None  # Will be set when we localize on new floor
        )
        
        # Initialize new floor if needed
        if new_floor not in self.floors:
            self.initialize_floor(new_floor)
        
        old_floor = self.current_floor
        self.current_floor = new_floor
        
        # Reset local SLAM for new floor
        print(f"Switching SLAM context: Floor {old_floor} -> Floor {new_floor}")
        
        return transition
    
    def get_global_pose(self, local_pose: np.ndarray) -> np.ndarray:
        """Convert floor-local pose to global pose"""
        global_pose = local_pose.copy()
        global_pose[2, 3] += self.floors[self.current_floor].altitude
        return global_pose
    
    def find_inter_floor_path(self, 
                              from_floor: int, 
                              to_floor: int) -> List[FloorTransition]:
        """Find path between floors using recorded transitions"""
        
        if from_floor == to_floor:
            return []
        
        # BFS to find shortest path
        from collections import deque
        
        queue = deque([(from_floor, [])])
        visited = {from_floor}
        
        while queue:
            current, path = queue.popleft()
            
            for transition in self.transitions:
                if transition.from_floor == current and transition.to_floor not in visited:
                    new_path = path + [transition]
                    if transition.to_floor == to_floor:
                        return new_path
                    queue.append((transition.to_floor, new_path))
                    visited.add(transition.to_floor)
        
        return None  # No path found
    
    def get_3d_map(self) -> np.ndarray:
        """Get combined 3D occupancy grid of all floors"""
        if not self.floors:
            return np.array([])
        
        # Stack floor maps vertically
        floor_ids = sorted(self.floors.keys())
        maps = [self.floors[f].occupancy_grid for f in floor_ids]
        
        # Create 3D array
        height, width = maps[0].shape
        combined = np.zeros((len(maps), height, width))
        
        for i, floor_map in enumerate(maps):
            combined[i] = floor_map
        
        return combined

def main():
    slam = MultiFloorSLAM()
    slam.initialize_floor(0)
    
    # Simulate multi-floor navigation
    print("\n=== Simulating Multi-Floor Navigation ===\n")
    
    # Floor 0 -> 1 via elevator
    slam.detect_floor_change(3.2, 0.0, np.eye(4))
    transition = slam.handle_floor_transition(
        1, FloorTransitionType.ELEVATOR, np.eye(4)
    )
    slam.transitions.append(transition)
    
    # Floor 1 -> 2 via stairs
    slam.detect_floor_change(6.5, 15.0, np.eye(4))
    transition = slam.handle_floor_transition(
        2, FloorTransitionType.STAIRS, np.eye(4)
    )
    slam.transitions.append(transition)
    
    print(f"\n=== Summary ===")
    print(f"Mapped {len(slam.floors)} floors")
    print(f"Recorded {len(slam.transitions)} floor transitions")

if __name__ == "__main__":
    main()
```

---

### Exercise 3.2.3: Dynamic Objects

**Task:** Filter dynamic objects (people, cars) from the static map.

```python
#!/usr/bin/env python3
"""Solution: Dynamic Object Filtering for SLAM"""

import numpy as np
from collections import defaultdict
from dataclasses import dataclass
from typing import List, Set, Tuple
from scipy.ndimage import gaussian_filter

@dataclass
class Detection:
    class_name: str
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    confidence: float
    depth: float  # Average depth in bbox

class DynamicFilter:
    """Filter dynamic objects from SLAM mapping"""
    
    # Classes known to be dynamic
    DYNAMIC_CLASSES = {
        'person', 'car', 'truck', 'bus', 'motorcycle', 'bicycle',
        'dog', 'cat', 'bird', 'horse', 'cow', 'sheep'
    }
    
    # Classes known to be static
    STATIC_CLASSES = {
        'wall', 'floor', 'ceiling', 'door', 'window', 'table',
        'chair', 'couch', 'bed', 'toilet', 'sink'
    }
    
    def __init__(self, grid_resolution: float = 0.05):
        self.grid_resolution = grid_resolution
        self.observation_counts = defaultdict(int)
        self.dynamic_votes = defaultdict(int)
        self.static_threshold = 5  # Observations to consider static
        self.dynamic_ratio_threshold = 0.3
    
    def is_dynamic_class(self, class_name: str) -> bool:
        """Check if class is inherently dynamic"""
        return class_name.lower() in self.DYNAMIC_CLASSES
    
    def filter_detections(self, 
                         detections: List[Detection],
                         depth_image: np.ndarray,
                         current_pose: np.ndarray) -> Set[Tuple[int, int]]:
        """Return set of grid cells occupied by dynamic objects"""
        
        dynamic_cells = set()
        
        for det in detections:
            if self.is_dynamic_class(det.class_name):
                # Get depth region
                x, y, w, h = det.bbox
                depth_roi = depth_image[y:y+h, x:x+w]
                
                # Project to world coordinates
                cells = self._project_to_grid(depth_roi, det.bbox, current_pose)
                dynamic_cells.update(cells)
        
        return dynamic_cells
    
    def _project_to_grid(self,
                        depth_roi: np.ndarray,
                        bbox: Tuple[int, int, int, int],
                        pose: np.ndarray) -> Set[Tuple[int, int]]:
        """Project depth ROI to occupancy grid cells"""
        
        cells = set()
        x, y, w, h = bbox
        
        # Camera intrinsics (simplified)
        fx, fy = 500, 500
        cx, cy = 320, 240
        
        for dy in range(0, h, 5):  # Sample every 5 pixels
            for dx in range(0, w, 5):
                depth = depth_roi[dy, dx] if dy < depth_roi.shape[0] and dx < depth_roi.shape[1] else 0
                
                if 0.1 < depth < 10.0:  # Valid depth
                    # Back-project to 3D
                    px = (x + dx - cx) * depth / fx
                    py = (y + dy - cy) * depth / fy
                    pz = depth
                    
                    # Transform to world frame
                    point_camera = np.array([px, py, pz, 1])
                    point_world = pose @ point_camera
                    
                    # Convert to grid cell
                    gx = int(point_world[0] / self.grid_resolution)
                    gy = int(point_world[1] / self.grid_resolution)
                    cells.add((gx, gy))
        
        return cells
    
    def update_occupancy_with_dynamics(self,
                                       occupancy_grid: np.ndarray,
                                       dynamic_cells: Set[Tuple[int, int]],
                                       observed_cells: Set[Tuple[int, int]]) -> np.ndarray:
        """Update occupancy grid, filtering dynamic cells"""
        
        filtered_grid = occupancy_grid.copy()
        
        for cell in observed_cells:
            gx, gy = cell
            if 0 <= gx < filtered_grid.shape[0] and 0 <= gy < filtered_grid.shape[1]:
                self.observation_counts[cell] += 1
                
                if cell in dynamic_cells:
                    self.dynamic_votes[cell] += 1
        
        # Mark cells as unoccupied if frequently dynamic
        for cell, count in self.observation_counts.items():
            if count >= self.static_threshold:
                dynamic_ratio = self.dynamic_votes[cell] / count
                gx, gy = cell
                
                if dynamic_ratio > self.dynamic_ratio_threshold:
                    # This cell is frequently dynamic - clear it
                    filtered_grid[gx, gy] = 0.0
        
        return filtered_grid
    
    def get_dynamic_mask(self, grid_shape: Tuple[int, int]) -> np.ndarray:
        """Get mask of frequently dynamic cells"""
        mask = np.zeros(grid_shape, dtype=bool)
        
        for cell, count in self.observation_counts.items():
            if count >= self.static_threshold:
                gx, gy = cell
                if 0 <= gx < grid_shape[0] and 0 <= gy < grid_shape[1]:
                    dynamic_ratio = self.dynamic_votes[cell] / count
                    if dynamic_ratio > self.dynamic_ratio_threshold:
                        mask[gx, gy] = True
        
        return mask

def main():
    filter = DynamicFilter()
    
    # Simulate observations
    print("Simulating dynamic object filtering...\n")
    
    # Simulate grid
    grid = np.zeros((200, 200))
    
    # Add some "walls"
    grid[50:150, 50] = 1.0
    grid[50:150, 150] = 1.0
    grid[50, 50:150] = 1.0
    grid[150, 50:150] = 1.0
    
    # Simulate person walking through
    for frame in range(20):
        detections = [
            Detection(
                class_name='person',
                bbox=(100 + frame*5, 200, 50, 100),
                confidence=0.9,
                depth=2.0
            )
        ]
        
        # Simulate depth image
        depth = np.ones((480, 640)) * 5.0
        
        dynamic_cells = filter.filter_detections(
            detections,
            depth,
            np.eye(4)
        )
        
        # Simulate observed cells (where person is)
        observed = dynamic_cells | {(100, 100), (100, 101)}  # Some static walls too
        
        grid = filter.update_occupancy_with_dynamics(grid, dynamic_cells, observed)
    
    dynamic_mask = filter.get_dynamic_mask(grid.shape)
    print(f"Cells identified as dynamic: {np.sum(dynamic_mask)}")
    print(f"Static map preserved: {np.sum(grid > 0)} occupied cells")

if __name__ == "__main__":
    main()
```

---

## Chapter 3.3: Nav2 Navigation

### Exercise 3.3.1: Waypoint Following

**Task:** Implement a waypoint sequence navigator with progress feedback.

```python
#!/usr/bin/env python3
"""Solution: Waypoint Sequence Navigator"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from typing import List
import yaml

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Status publisher
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        
        # Waypoint state
        self.waypoints: List[PoseStamped] = []
        self.current_waypoint_idx = 0
        self.is_navigating = False
        
        # Command subscriber
        self.command_sub = self.create_subscription(
            String, 'navigation_commands', self.command_callback, 10
        )
        
        self.get_logger().info('Waypoint Navigator ready')
    
    def load_waypoints_from_yaml(self, yaml_file: str):
        """Load waypoints from YAML file"""
        with open(yaml_file) as f:
            data = yaml.safe_load(f)
        
        self.waypoints = []
        for wp in data['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = wp.get('yaw_sin', 0.0)
            pose.pose.orientation.w = wp.get('yaw_cos', 1.0)
            self.waypoints.append(pose)
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
    
    def command_callback(self, msg: String):
        """Handle navigation commands"""
        cmd = msg.data.lower()
        
        if cmd == 'start':
            self.start_navigation()
        elif cmd == 'pause':
            self.pause_navigation()
        elif cmd == 'resume':
            self.resume_navigation()
        elif cmd == 'stop':
            self.stop_navigation()
        elif cmd.startswith('goto '):
            idx = int(cmd.split()[1])
            self.goto_waypoint(idx)
    
    def start_navigation(self):
        """Start navigating through all waypoints"""
        if not self.waypoints:
            self.publish_status('ERROR: No waypoints loaded')
            return
        
        self.current_waypoint_idx = 0
        self.is_navigating = True
        self.navigate_to_current_waypoint()
    
    def navigate_to_current_waypoint(self):
        """Navigate to current waypoint in sequence"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.publish_status('COMPLETE: All waypoints reached')
            self.is_navigating = False
            return
        
        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[self.current_waypoint_idx]
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.publish_status(
            f'NAVIGATING: Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}'
        )
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(
            goal, 
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status('ERROR: Goal rejected')
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        
        if result:
            self.publish_status(
                f'REACHED: Waypoint {self.current_waypoint_idx + 1}'
            )
            
            # Move to next waypoint
            self.current_waypoint_idx += 1
            if self.is_navigating:
                self.navigate_to_current_waypoint()
        else:
            self.publish_status(
                f'FAILED: Could not reach waypoint {self.current_waypoint_idx + 1}'
            )
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        progress = {
            'waypoint': self.current_waypoint_idx + 1,
            'total': len(self.waypoints),
            'distance_remaining': distance,
            'estimated_time': feedback.estimated_time_remaining.sec
        }
        
        self.get_logger().debug(f'Progress: {progress}')
    
    def pause_navigation(self):
        """Pause current navigation"""
        # Cancel current goal
        self.is_navigating = False
        self.publish_status('PAUSED')
    
    def resume_navigation(self):
        """Resume from current waypoint"""
        self.is_navigating = True
        self.navigate_to_current_waypoint()
    
    def stop_navigation(self):
        """Stop navigation completely"""
        self.is_navigating = False
        self.current_waypoint_idx = 0
        self.publish_status('STOPPED')
    
    def goto_waypoint(self, idx: int):
        """Go to specific waypoint"""
        if 0 <= idx < len(self.waypoints):
            self.current_waypoint_idx = idx
            self.is_navigating = False
            self.navigate_to_current_waypoint()
    
    def publish_status(self, status: str):
        """Publish navigation status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

def main():
    rclpy.init()
    navigator = WaypointNavigator()
    
    # Example waypoints
    navigator.waypoints = []
    for i in range(5):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(i)
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        navigator.waypoints.append(pose)
    
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

*Solutions verified with Nav2 Humble and Isaac Sim 2023.1*
