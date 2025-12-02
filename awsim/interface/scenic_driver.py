'''
This is a placeholder for a Scenic to AWSIM driver used for early testing, tt does NOT implement true AWSIM communication but instead, it:
- Loads a Scenic scenario file
- Extracts a Scenic generated scene and objects
- Registers those objects in AwsimSimulator (currently out)
- Generates simple control inputs for testing (throttle/steer for ego)
- Advances the Awesome simulator for a few time steps and prints dummy states

Once Awesome/ros2 integration is available, this file will drive the real simulation loop and send real control commands to AWSIM every step.
'''

from __future__ import annotations
from pathlib import Path
from typing import Dict
from scenic import scenarioFromFile
from awsim.interface.awsim_simulator import AwsimSimulator

# Return the 216Project root (two levels above this file)
def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]

# Load a Scenic scenario from awsim/scenic_scenarios and generate a scene
def load_scenic_scene(filename: str = "minimal.scenic"):
    root = _project_root()
    scenic_file = root / "awsim" / "scenic_scenarios" / filename

    if not scenic_file.exists():
        raise FileNotFoundError(f"Scenic file not found: {scenic_file}")

    print(f"[ScenicDriver] Loading Scenic scenario: {scenic_file}")
    scenario = scenarioFromFile(str(scenic_file))
    scene, _ = scenario.generate()
    return scene

# Register all Scenic objects in the AwsimSimulator (Graeme: you will extend this once AWSIM/ROS2 is connected.)
def register_scene_objects(sim: AwsimSimulator, scene) -> None:
    # For now we just assign keys "obj0", "obj1", ... with "obj0" treated as ego, later we can use Scenic object names or properties if needed
    print("[ScenicDriver] Registering Scenic objects with AwsimSimulator")
    for idx, obj in enumerate(scene.objects):
        key = "ego" if idx == 0 else f"npc{idx}"
        sim.register_object(key, kind="vehicle")
        handle = sim.objects[key]
        pos = obj.position   # usually something like (x, y) or (x, y, z)
        handle.x = float(pos[0])
        handle.y = float(pos[1])
        handle.heading = float(obj.heading)
        handle.speed = float(getattr(obj, "speed", 0.0))
        print(
            f"  - Scenic object {idx} mapped to key '{key}', "
            f"pos=({handle.x:.2f}, {handle.y:.2f}), "
            f"heading={handle.heading:.1f} deg"
        )

# Very simple hardcoded control policy for testing; ego: some throttle and a gentle steer, others: low throttle, straight
def simple_control_policy(step_index: int) -> Dict[str, Dict[str, float]]:
    throttle_ego = 0.3
    steer_ego = 0.05 if step_index < 10 else 0.0

    controls: Dict[str, Dict[str, float]] = {
        "ego": {"throttle": throttle_ego, "steer": steer_ego},
        # we won't explicitly control NPCs yet. You (Graeme) can extend this later.
    }
    return controls

# End-to-end demo: load Scenic scenario, create AwsimSimulator, register Scenic objects, run a few steps with dummy controls
def run_minimal_demo(steps: int = 5, dt: float = 0.1) -> None:
    scene = load_scenic_scene("minimal.scenic")
    sim = AwsimSimulator(config={"source": "scenic_driver_demo"})

    register_scene_objects(sim, scene)

    print(f"[ScenicDriver] Running demo for {steps} steps (dt={dt})")
    for i in range(steps):
        controls = simple_control_policy(i)
        sim.step(dt=dt, controls=controls)

        # Query ego state (currently dummy values until AWSIM is wired).
        ego_state = sim.get_object_state("ego")
        print(f"[ScenicDriver] Step {i}: ego_state={ego_state}")

    sim.shutdown()
    print("[ScenicDriver] Demo complete")


if __name__ == "__main__":
    run_minimal_demo()
