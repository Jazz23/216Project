import math
import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand


class AwsimObjectHandle:
    """Represents a Scenic object mapped into AWSim."""
    def __init__(self, scenic_name, awsim_id, kind="vehicle"):
        self.scenic_name = scenic_name
        self.awsim_id = awsim_id
        self.kind = kind
        self.throttle = 0.0
        self.steering = 0.0

    def __repr__(self):
        return (f"AwsimObjectHandle(scenic_name='{self.scenic_name}', "
                f"awsim_id='{self.awsim_id}', kind='{self.kind}')")


class AwsimSimulator(Node):
    """Bridge between Scenic and AWSim via ROS2."""

    def __init__(self):
        super().__init__("awsim_scenic_interface")

        # Publishers for Autoware/AWSim control inputs
        self.control_pub = self.create_publisher(
            Control, "/control/command/control_cmd", 10
        )
        self.gear_pub = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", 10
        )

        # Internal object storage
        self.objects = {}
        self._counter = 0

        self.get_logger().info("AwsimSimulator ready (control publishers online).")

    # -------------------------------------------------------
    # Scenic compatibility: register object from Scenic Scene
    # -------------------------------------------------------
    def register_object(self, name, kind="vehicle"):
        awsim_id = f"awsim_{name}"
        handle = AwsimObjectHandle(name, awsim_id, kind)
        self.objects[name] = handle

        self.get_logger().info(
            f"[AwsimSimulator] Registered object '{name}' as {handle}"
        )
        return handle

    # -------------------------------------------------------
    # Control Application
    # -------------------------------------------------------
    def apply_controls(self, name, throttle, steering):
        """Publish throttle/steering for the given object."""
        if name not in self.objects:
            raise KeyError(f"No object named '{name}' registered.")

        handle = self.objects[name]
        handle.throttle = throttle
        handle.steering = steering

        # Prepare ROS2 control message
        msg = Control()
        msg.longitudinal.velocity = throttle * 6.0  # simple scaling
        msg.lateral.steering_tire_angle = steering

        # Publish autoware control command
        self.control_pub.publish(msg)

        self.get_logger().info(
            f"[CONTROL] {name} vel={msg.longitudinal.velocity:.2f} "
            f"steer={steering:.2f}"
        )

    # -------------------------------------------------------
    # Simulation Step (Scenic expects this hook)
    # -------------------------------------------------------
    def step(self, dt, controls):
        """
        Scenic calls this once per timestep.
        We simply publish controls to AWSim.
        """
        for name, ctrl in controls.items():
            throttle = ctrl.get("throttle", 0.0)
            steer = ctrl.get("steer", 0.0)
            self.apply_controls(name, throttle, steer)

        return {}  # Scenic expects a dict

    # -------------------------------------------------------
    # Scenic createSimulation() — patched stub
    # -------------------------------------------------------
    def createSimulation(self, scene, **kwargs):
        """
        Scenic expects a Simulation-like object. We return a harmless
        dummy simulation that Scenic will not use to advance physics.
        """

        class DummySimulation:
            def __init__(self, scene):
                self.scene = scene
                self.timestep = 0.1

            def step(self):
                # Scenic behavior engines will not call this when using AWSim as external simulator
                return {}

        return DummySimulation(scene)

    # -------------------------------------------------------
    # Scenic convenience API
    # -------------------------------------------------------
    def get_object_state(self, name):
        """Return dummy state (since AWSim is authoritative)."""
        return {
            "awsim_id": self.objects[name].awsim_id,
            "kind": self.objects[name].kind,
            "note": "AWSim authoritative; state not simulated here.",
        }

    # -------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------
    def destroy(self):
        """Explicit cleanup."""
        for name in list(self.objects.keys()):
            handle = self.objects[name]
            self.get_logger().info(
                f"[AwsimSimulator] Removed object '{name}' (awsim_id={handle.awsim_id})"
            )
        self.objects.clear()