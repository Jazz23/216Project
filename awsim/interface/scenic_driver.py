import rclpy
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

        handle = sim.register_object(name)

        print(
            f"  - Scenic object {i} mapped to key '{name}', "
            f"pos=({obj.position.x:.2f}, {obj.position.y:.2f}), "
            f"heading={obj.heading:.1f} deg"
        )


# -------------------------------------------------------
# Generate control commands each step
# (placeholder until Scenic behaviors are added)
# -------------------------------------------------------
def generate_controls(scene):
    """Simple fixed control policy. Later replaced with Scenic behaviors."""
    controls = {}
    ego = scene.objects[0]

    controls[ego.name] = {
        "throttle": 2.5,
        "steer": 0.0,
    }
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
        executor.spin_once(timeout_sec=0)
        controls = generate_controls(scene)
        sim.step(dt, controls)

        ego_name = scene.objects[0].name
        state = sim.get_object_state(ego_name)
        print(f"[ScenicDriver] Step {step}: ego_state={state}")

    sim.destroy()
    print("[ScenicDriver] Demo complete")
    rclpy.shutdown()


if __name__ == "__main__":
    run_minimal_demo()