---
title: "Synthetic Data Generation (SDG) via Replicator"
sidebar_label: "Synthetic Data"
---

# Synthetic Data Generation (SDG) via Replicator

One of the most significant bottlenecks in developing robust AI for robotics is the acquisition of large, diverse, and well-labeled datasets. Collecting and annotating real-world data is often time-consuming, expensive, and limited by environmental conditions. This is where **Synthetic Data Generation (SDG)**, particularly through NVIDIA Omniverse Replicator in Isaac Sim, offers a powerful solution.

## The Challenge of Real-World Data

Consider training an object detection model for a robotic arm picking up various items.
-   You need images of each object from multiple angles, under different lighting conditions.
-   Objects might be occluded by each other or by the robot's own arm.
-   The background needs to vary to prevent overfitting to a specific lab setup.
-   Each object instance needs precise bounding box and segmentation mask annotations.

This process can take weeks or months for even a modest set of objects.

## What is Synthetic Data Generation?

SDG is the process of creating artificial datasets that mimic the properties of real-world data, but are generated entirely in a simulated environment. Because the data comes from a simulator, all ground truth information (e.g., object positions, bounding boxes, segmentation masks, depth maps) is known and can be automatically extracted with perfect accuracy.

### Advantages of SDG:

1.  **Cost-Effective**: Drastically reduces the cost of data collection and annotation.
2.  **Scalable**: Generate millions of unique data points simply by varying parameters in the simulation.
3.  **Diverse**: Easily introduce variations in lighting, textures, object placement, camera angles, and environmental conditions that might be difficult or dangerous to create in the real world.
4.  **Perfect Ground Truth**: Obtain pixel-perfect annotations automatically for tasks like instance segmentation, 3D pose estimation, and depth prediction.
5.  **Edge Cases**: Systematically generate data for rare or challenging scenarios (e.g., specific types of occlusions, extreme lighting) that are hard to capture in reality.

## NVIDIA Omniverse Replicator

NVIDIA Omniverse Replicator is a powerful framework for generating synthetic data from physically accurate virtual worlds within Isaac Sim. It allows you to programmatically control every aspect of your scene and automatically export various types of ground truth data.

### Key Features of Replicator:

-   **Domain Randomization**: Randomly varies non-essential aspects of the simulation (e.g., textures, lighting, object positions, camera properties) to improve the generalization of models trained on synthetic data. This makes the models less sensitive to the "reality gap."
-   **Structured Randomization**: Systematically varies important parameters, such as object pose or type, to ensure full coverage of critical scenarios.
-   **Multiple Annotators**: Simultaneously extract various ground truth data types for each generated frame:
    -   Bounding Boxes (2D and 3D)
    -   Instance Segmentation Masks
    -   Semantic Segmentation Masks
    -   Depth Maps
    -   Normals
    -   Object ID Maps
-   **Python API**: Replicator exposes a Python API, allowing you to script complex data generation pipelines and integrate them into your existing workflows.

## Workflow for SDG with Replicator

1.  **Scene Setup**: Load your robot model (as USD), relevant objects, and define your environment in Isaac Sim.
2.  **Annotator Configuration**: Specify which types of ground truth data you want to extract (e.g., `bounding_box_2d_tight`, `instance_segmentation`).
3.  **Randomization Rules**: Define the parameters you want to randomize and their ranges (e.g., "randomize the `prim_path` of all objects under `/World/Props`," "randomize the `intensity` of all lights").
4.  **Data Capture**: Programmatically trigger the capture of data frames, typically in a loop, while randomizing the scene.
5.  **Export**: Replicator automatically exports the rendered images and all corresponding ground truth annotations to a specified output directory.

### Simple Python Example (Conceptual)

```python
import omni.isaac.core.utils.carb as carb_utils
from omni.isaac.kit import SimulationApp

# Start the simulation app
# config = carb_utils.get_carb_setting("/app/async_rendering", False)
# simulation_app = SimulationApp({"headless": False}) # For GUI
simulation_app = SimulationApp({"headless": True}) # For faster data gen

from omni.isaac.core import World
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.core.objects import DynamicCuboid
import omni.replicator.core as rep
import numpy as np

# Create a world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Add a simple cuboid
cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        position=np.array([0.0, 0.0, 0.5]),
        size=0.1,
        color=np.array([0, 0, 1.0]),
    )
)

# Setup a camera for data capture
camera_prim = rep.create.camera(position=(0, 0, 1.0), look_at=(0, 0, 0))
render_product = rep.create.render_product(camera_prim, (512, 512))

# Define annotators (e.g., bounding box)
rep.orchestrator.set_default_output_directory("omniverse_data")
rep.orchestrator.step() # Initialize replicator

# Randomization over position of the cube
with rep.trigger.on_frame():
    with rep.create.node(name="Randomizer", node_type="omni.replicator.core.trigger"):
        rep.modify.pose(cube, position=rep.distribution.uniform((-0.5, -0.5, 0.2), (0.5, 0.5, 0.8)))

    rep.Writer(
        output_dir="synthetic_dataset",
        rp_node=render_product,
        annotators=[
            rep.AnnotatorRegistry.get_annotator("rgb"),
            rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight"),
            rep.AnnotatorRegistry.get_annotator("instance_segmentation"),
        ],
    ).step()

# Simulate and generate data
my_world.reset()
for i in range(100): # Generate 100 frames
    my_world.step(render=True) # Advance physics and render
    rep.orchestrator.step() # Trigger replicator and capture data

simulation_app.close()
```

SDG with Omniverse Replicator is a game-changer for AI in robotics, enabling the rapid development of robust perception models that are critical for autonomous systems. By mastering this technique, you can overcome one of the biggest hurdles in bringing AI from the lab to the real world.
