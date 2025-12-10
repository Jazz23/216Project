import rclpy
import math
from scenic import scenarioFromFile
from awsim.interface.awsim_simulator import AwsimSimulator


# -------------------------------------------------------
# Load Scenic and sample a scene
# -------------------------------------------------------
def load_scenic_scene(filename):
    print("[ScenicDriver] Loading Scenic scenario...")
    scenario = scenarioFromFile(f"awsim/scenic_scenarios/{filename}")
    scene, iterations = scenario.generate()

    print(f"[ScenicDriver] Scene generated in {iterations} iterations.")
    return scene, scenario


# -------------------------------------------------------
# Register Scenic objects into AWSim
# -------------------------------------------------------
def register_scenic_objects(sim: AwsimSimulator, scene):
    print("[ScenicDriver] Registering Scenic objects with AwsimSimulator")

    for i, obj in enumerate(scene.objects):
        name = getattr(obj, "name", f"obj{i}")

        sim.register_object(name)

        print(
            f"  - Scenic object {i} mapped to key '{name}', "
            f"pos=({obj.position.x:.2f}, {obj.position.y:.2f}), "
            f"heading={obj.heading:.1f} deg"
        )


# -------------------------------------------------------
# Generate control commands each step
# -------------------------------------------------------
def generate_controls(scene, scenario):
    """
    Generate controls from Scenic objects.

    Uses Scenic parameters:
      - egoSpeed : desired speed for the ego (m/s)
      - egoAccel : optional desired acceleration for the ego (m/s^2)
    """
    ego = scene.objects[0]

    # Scenic-defined speed (default 0 if missing)
    scenic_speed = scenario.params.get("egoSpeed", 0.0)

    # Optional Scenic-defined acceleration
    scenic_accel = scenario.params.get("egoAccel", 0.0)

    # Scenic heading is in degrees; AWSim expects a steering tire angle in radians.
    heading_deg = float(scenario.params.get("egoSteer", 0.0))
    heading_rad = math.radians(heading_deg)

    # For now, we keep steering simple (straight); you can add behavior later.
    steer = heading_deg

    controls = {
        ego.name: {
            "throttle": scenic_speed,  # interpreted as speed in AwsimSimulator
            "steer": steer,
            "accel": scenic_accel,
        }
    }

    # Only include 'accel' if Scenic actually provided egoAccel
    if scenic_accel is not None:
        controls[ego.name]["accel"] = scenic_accel

    print(f"[LOOKIE HERE] {controls}")

    return controls


# -------------------------------------------------------
# Main demonstration run
# -------------------------------------------------------
def run_minimal_demo():
    rclpy.init()

    scene, scenario = load_scenic_scene("minimal.scenic")

    sim = AwsimSimulator()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sim)

    register_scenic_objects(sim, scene)

    dt = 0.1
    steps = 5
    print(f"[ScenicDriver] Running demo for {steps} steps (dt={dt})")

    for step in range(steps):
        controls = generate_controls(scene, scenario)
        sim.step(dt, controls)

        ego_name = scene.objects[0].name
        # state = sim.get_object_state(ego_name)
        # print(f"[ScenicDriver] Step {step}: ego_state={state}")

    sim.destroy()
    print("[ScenicDriver] Demo complete")
    rclpy.shutdown()


if __name__ == "__main__":
    run_minimal_demo()