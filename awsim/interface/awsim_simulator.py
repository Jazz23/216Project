from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Optional #leave optional in case we need it for later

import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
from builtin_interfaces.msg import Time

@dataclass
class AwsimObjectHandle:
    def __init__(self, scenic_name: str, awsim_id: str, kind: str):
        self. scenic_name = scenic_name
        self.awsim_id = awsim_id
        self.kind = kind
        
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.steering = 0.0
        self.throttle = 0.0
        
    def __repr__(self):
        return (f"AwsimObjectHandle(id={self.awsim_id}, kind={self.kind}, "
                f"pos=({self.x}, {self.y}, {self.heading}), "
                f"speed={self.speed})")
    
class AwsimSimulator(Node):
    # Initialize AWSIM simulator connection (Graeme: replace placeholders with real ROS2/AWSIM setup).
    def __init__(self):
        super().__init__("awsim_scenic_interface")
        self.objects: Dict[str, AwsimObjectHandle] = {}
        self.state: Dict[str, Dict[str, float]] = {}
        
        # TO-DO(Graeme): initialize AWSIM / ROS2 connection here
        self.control_pub = self.create_publisher(Control, "/control/command/control_cmd", 10)
        self.gear_pub = self.create_publisher(GearCommand, "/control/command/gear_cmd", 10)

        gear = GearCommand()
        gear.command = GearCommand.DRIVE
        self.gear_pub.publish(gear)

        self.get_logger().info("AwsimSimulator ready (control publishers online).")

        # print("[AwsimSimulator] Initialized with config:", self.config)
        
        
    # Stores object metadata (Graeme: create matching object in AWSIM)
    def register_object(self, key: str, kind: str = "vehicle") -> None:
        if key in self.objects:
            raise ValueError(f"Object with key '{key}' is already registered.")

        # TO-DO(Graeme): call into AWSIM to spawn the object and get its handle/id.
        awsim_id = f"awsim_{key}"  # placeholder

        handle = AwsimObjectHandle(
            scenic_name=key,
            awsim_id=awsim_id,
            kind=kind,
        )
        self.objects[key] = handle
        print(f"[AwsimSimulator] Registered object '{key}' as {handle}")
    
    #Remove an object from AWSIM (Graeme: Despawn the matching object inside AWSIM.)
    def remove_object(self, key: str) -> None:
        handle = self.objects.pop(key, None)
        if handle is None:
            print(f"[AwsimSimulator] Tried to remove unknown object '{key}'")
            return

        # TO-DO(Graeme): despawn object in AWSIM.
        print(f"[AwsimSimulator] Removed object '{key}' (awsim_id={handle.awsim_id})")
    
    # Advance AWSIM one time step (Graeme: Apply control commands (throttle, steering, braking) to AWSIM via ROS2)   
    def step(self, dt: float, controls: Dict[str, Dict[str, float]]) -> None:
        
        # Part 1) We apply controls for each registered object.
        for key, ctrl in controls.items():
            if key not in self.objects:
                print(f"[AwsimSimulator] Warning: control for unknown object '{key}'")
                continue

            self.apply_controls(
                key,
                ctrl.get("throttle", 0.0),
                ctrl.get("steer", 0.0)
            )

            # handle = self.objects[key]
            # throttle = ctrl.get("throttle", 0.0)
            # steering = ctrl.get("steer", 0.0)
            
            # st = self.state.get(key, {})
            # speed = throttle * 10.0
            # st["speed"] = speed
            # st["x"] = st.get("x", 0.0) + speed * dt
            # self.state[key] = st

        # TO-DO (Graeme): send throttle/steering/etc. to AWSIM/AUTOWARE via ROS2.
        for obj in self.objects.values():
            self._apply_controls(obj, dt)
        print(
            f"[AwsimSimulator] Applying controls to '{key}' "
            f"(awsim_id={obj.awsim_id}): throttle={obj.throttle}, steering={obj.steering}"
        )
            
        for name, obj in self.objects.items():
            st = self.state.setdefault(key, {})
            speed = obj.throttle * 6.0
            st["speed"] = speed
            st["x"] = st.get("x", 0.0) + speed * dt
        print(
            f"[AwsimSimulator] New dummy state for '{key}': "
            f"x={st['x']:.2f}, y={st.get('y', 0.0):.2f}, "
            f"heading_deg={st.get('heading_deg', 0.0):.1f}, speed={st['speed']:.2f}"
        )

        # Part 2) Step the AWSIM simulation forward by dt.
        # TO-DO (Graeme): call AWSIM to advance simulation time by dt.
        print(f"[AwsimSimulator] Stepping simulation by dt={dt} seconds (dummy)")

        # Part 3) After stepping, AWSIM will have updated states for each object.
        # TO-DO (Graeme): read and cache updated state from AWSIM.  

    def _apply_controls(self, obj: AwsimObjectHandle, dt: float):

        # --- CHANGED: Only publish commands for ego
        if obj.scenic_name != "ego":
            return

        # --- ADDED: Convert Scenic throttle (0–1) to AWSIM velocity (V1 scaling)
        MAX_SPEED = 6.0  # m/s                     # --- ADDED
        velocity = float(obj.throttle) * MAX_SPEED # --- ADDED

        # --- ADDED: Steering directly applied
        steering = float(obj.steering)             # --- ADDED

        # --- ADDED: Create Control message
        cmd = Control()
        now = self.get_clock().now().to_msg()

        cmd.stamp = now

        cmd.lateral.steering_tire_angle = steering

        cmd.longitudinal.velocity = velocity
        cmd.longitudinal.acceleration = 0.0
        
        # --- ADDED: Publish to AWSim
        self.control_pub.publish(cmd)

        # Debug log
        self.get_logger().info(
            f"[CONTROL] ego vel={velocity:.2f} m/s steer={steering:.2f} rad"
        )

    def apply_controls(self, name: str, throttle: float, steering: float):
        if name in self.objects:
            self.objects[name].throttle = throttle
            self.objects[name].steering = steering


    # Query the latest known state for an object from AWSIM (Graeme: Replace placeholder with ROS2 callbacks/calls to AWSIM)
    # Returns a dictionary like: {"x": 10.0,"y": 5.0,"z": 0.0, "heading degree": 90.0,"speed": 3.0}
    def get_object_state(self, key: str) -> Dict[str, Any]:
        
        if key not in self.objects:
            raise ValueError(f"Unknown object key '{key}'")

        handle = self.objects[key]
        st = self.state.get(key, {})

        # TO-DO (Graeme): query AWSIM or return cached state.
        # For now we just return dummy values for testing.
        state = {
            "x": st.get("x", 0.0),
            "y": st.get("y", 0.0),
            "z": 0.0,
            "heading_deg": st.get("heading_deg", 0.0),
            "speed": st.get("speed", 0.0),
            "awsim_id": handle.awsim_id,
            "kind": handle.kind,
        }

        print(f"[AwsimSimulator] get_object_state('{key}') -> {state}")
        return state
    
    # Return state for all registered objects (Graeme: Make sure this matches how AWSIM publishes multi-vehicle states)
    def get_all_states(self) -> Dict[str, Dict[str, Any]]:
        
        return {key: self.get_object_state(key) for key in self.objects.keys()}
    
    # Clean up AWSIM connection and despawn all objects (Graeme: Implement ROS2 node shutdown/teardown)
    def shutdown(self) -> None:
        
        for key in list(self.objects.keys()):
            self.remove_object(key)

        # TO-DO (Graeme): shutdown ROS2/AWSIM resources if needed.
        print("[AwsimSimulator] Shutdown complete")