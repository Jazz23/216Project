from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict

from rclpy.node import Node

from autoware_control_msgs.msg import Control           # ✓ valid import
from autoware_vehicle_msgs.msg import GearCommand
from builtin_interfaces.msg import Time


# ------------------------------
#  Awsim Object Handle
# ------------------------------
@dataclass
class AwsimObjectHandle:
    scenic_name: str
    awsim_id: str
    kind: str

    # local desired control inputs (not state)
    throttle: float = 0.0
    steering: float = 0.0


# ------------------------------
#  Awsim Simulator Node
# ------------------------------
class AwsimSimulator(Node):

    def __init__(self):
        super().__init__("awsim_scenic_interface")

        self.objects: Dict[str, AwsimObjectHandle] = {}

        # -------------------------
        # REAL AWSIM control topics
        # -------------------------
        self.control_pub = self.create_publisher(Control, "/control/command/control_cmd", 10)
        self.gear_pub = self.create_publisher(GearCommand, "/control/command/gear_cmd", 10)

        # Send DRIVE gear once
        gear = GearCommand()
        gear.command = GearCommand.DRIVE
        self.gear_pub.publish(gear)

        self.get_logger().info("AwsimSimulator ready (control publishers online).")


    # ------------------------------
    # Register a new Scenic object
    # ------------------------------
    def register_object(self, key: str, kind: str = "vehicle"):
        if key in self.objects:
            raise ValueError(f"Object '{key}' already exists.")

        # CHANGED — no dummy state, AWSim is authoritative
        awsim_id = f"awsim_{key}"

        handle = AwsimObjectHandle(
            scenic_name=key,
            awsim_id=awsim_id,
            kind=kind
        )

        self.objects[key] = handle
        print(f"[AwsimSimulator] Registered object '{key}' as {handle}")


    # ------------------------------
    # Remove object
    # ------------------------------
    def remove_object(self, key: str):
        handle = self.objects.pop(key, None)
        if handle is None:
            print(f"[AwsimSimulator] Tried to remove unknown object '{key}'")
            return

        print(f"[AwsimSimulator] Removed object '{key}' (awsim_id={handle.awsim_id})")


    # ------------------------------
    # STEP — Option C: ONLY publish control inputs
    # ------------------------------
    def step(self, dt: float, controls: Dict[str, Dict[str, float]]):
        """
        dt ignored — AWSim is authoritative.
        Scenic provides throttle/steering.
        """

        for key, ctrl in controls.items():
            if key not in self.objects:
                print(f"[AwsimSimulator] Warning: unknown object '{key}'")
                continue

            # Get control inputs
            throttle = float(ctrl.get("throttle", 0.0))
            steering = float(ctrl.get("steer", 0.0))

            self.apply_controls(key, throttle, steering)

        # CHANGED — We no longer simulate state; only publish commands
        for obj in self.objects.values():
            self._publish_control(obj)


    # ------------------------------
    # Publish appropriate Autoware control messages
    # ------------------------------
    def _publish_control(self, obj: AwsimObjectHandle):

        # Only ego receives /control/command messages
        if obj.scenic_name != "ego":
            return

        MAX_SPEED = 6.0  # scale Scenic throttle to AWSIM velocity

        cmd = Control()
        now = self.get_clock().now().to_msg()
        cmd.stamp = now

        # FIXED — assign correct fields for Control message
        cmd.longitudinal.velocity = obj.throttle * MAX_SPEED
        cmd.longitudinal.acceleration = 0.0
        cmd.lateral.steering_tire_angle = obj.steering

        self.control_pub.publish(cmd)

        self.get_logger().info(
            f"[CONTROL] ego vel={cmd.longitudinal.velocity:.2f} "
            f"steer={cmd.lateral.steering_tire_angle:.2f}"
        )


    # ------------------------------
    # Update desired control inputs
    # ------------------------------
    def apply_controls(self, name: str, throttle: float, steering: float):
        if name not in self.objects:
            raise ValueError(f"Unknown object '{name}'")

        obj = self.objects[name]
        obj.throttle = throttle
        obj.steering = steering


    # ------------------------------
    # Query AWSIM state (placeholder)
    # ------------------------------
    def get_object_state(self, key: str):
        """
        AWSim should be queried through ROS2 topics.
        Scenic should NOT simulate state.
        """
        if key not in self.objects:
            raise ValueError(f"Unknown object '{key}'")

        return {
            "awsim_id": self.objects[key].awsim_id,
            "kind": self.objects[key].kind,
            "note": "AWSim authoritative; state not simulated here."
        }


    def get_all_states(self):
        return {k: self.get_object_state(k) for k in self.objects}


    # ------------------------------
    # Shutdown
    # ------------------------------
    def shutdown(self):
        for key in list(self.objects.keys()):
            self.remove_object(key)

        print("[AwsimSimulator] Shutdown complete")
