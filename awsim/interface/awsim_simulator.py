from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, VelocityReport


class AwsimObjectHandle:
    """Represents a Scenic object mapped into AWSim."""

    def __init__(self, scenic_name, awsim_id, kind="vehicle"):
        self.scenic_name = scenic_name
        self.awsim_id = awsim_id
        self.kind = kind
        # These are just “last command” values, not a real state
        self.throttle = 0.0   # interpreted as desired speed (m/s)
        self.steering = 0.0

    def __repr__(self):
        return (
            f"AwsimObjectHandle(scenic_name='{self.scenic_name}', "
            f"awsim_id='{self.awsim_id}', kind='{self.kind}')"
        )


class AwsimSimulator(Node):
    """Bridge between Scenic and AWSim via ROS2."""

    def __init__(self):
        super().__init__("awsim_scenic_interface")

        # Sim time from /clock (simulated time)
        self.sim_time = Time(sec=0, nanosec=0)

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers for Autoware/AWSim control inputs
        self.control_pub = self.create_publisher(
            Control, "/control/command/control_cmd", cmd_qos
        )
        self.gear_pub = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", cmd_qos
        )

        vel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._last_velocity_report: VelocityReport | None = None

        self.velocity_sub = self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self._velocity_status_callback,
            vel_qos,
        )

        # Subscribe to /clock so we can time-stamp messages correctly
        self.create_subscription(Clock, "/clock", self._clock_callback, 10)

        # Immediately put the vehicle into DRIVE
        gear = GearCommand()
        gear.command = GearCommand.DRIVE
        gear.stamp = self._now()
        self.gear_pub.publish(gear)

        # Internal object storage
        self.objects = {}
        self._counter = 0

        self.get_logger().info("AwsimSimulator ready (control publishers online).")

    # -------------------------------------------------------
    # Time handling
    # -------------------------------------------------------
    def _clock_callback(self, msg: Clock):
        self.sim_time = msg.clock

    def _now(self):
        # Use simulation time from AWSim
        return self.sim_time

    # -------------------------------------------------------
    # Velocity status subscription
    # -------------------------------------------------------
    def _velocity_status_callback(self, msg: VelocityReport) -> None:
        """Callback for /vehicle/status/velocity_status."""
        self._last_velocity_report = msg
        print(f'{self._last_velocity_report}')
        # Optional debug:
        # self.get_logger().info(
        #     f"[AwsimSimulator] Velocity update: "
        #     f"longitudinal={msg.longitudinal_velocity:.2f}, "
        #     f"lateral={msg.lateral_velocity:.2f}, "
        #     f"heading_rate={msg.heading_rate:.2f}"
        # )

    def get_current_velocity(self):
        """Return the most recent velocity as a simple dict.

        {
            "longitudinal": float m/s,
            "lateral":      float m/s,
            "heading_rate": float rad/s
        }
        """
        if self._last_velocity_report is None:
            return {
                "longitudinal": 0.0,
                "lateral": 0.0,
                "heading_rate": 0.0,
            }

        rep = self._last_velocity_report
        return {
            "longitudinal": rep.longitudinal_velocity,
            "lateral": rep.lateral_velocity,
            "heading_rate": rep.heading_rate,
        }


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
    def apply_controls(self, name, throttle, steering, acceleration=None):
        """
        Apply controls for a Scenic object.

        Parameters
        ----------
        name : str
            Scenic object name (e.g., "ego").
        throttle : float
            Interpreted as desired *speed* from Scenic (e.g., scene.params.egoSpeed).
            Units should match what AWSim expects (m/s).
        steering : float
            Steering tire angle (radians).
        acceleration : float | None
            Optional acceleration command from Scenic (e.g., scene.params.egoAccel).
            If None, we use a default that preserves previous behavior.
        """
        if name not in self.objects:
            raise KeyError(f"No object named '{name}' registered.")

        handle = self.objects[name]
        handle.throttle = float(throttle)
        handle.steering = float(steering)

        msg = Control()
        msg.stamp = self._now()

        # LATERAL -------------------------------
        msg.lateral.steering_tire_angle = float(steering)
        msg.lateral.is_defined_steering_tire_rotation_rate = False

        # LONGITUDINAL --------------------------
        # Scenic "throttle" is really desired speed from the scenario
        msg.longitudinal.velocity = float(throttle)

        # Acceleration:
        # - If Scenic provided an explicit acceleration (egoAccel), use it.
        # - Otherwise, retain the old behavior (accel=1.0 when moving, 0 when stopped),
        #   which we already know makes AWSim move.
        # if acceleration is None:
        #     if throttle != 0.0:
        #         accel = 1.0
        #     else:
        #         accel = 0.0
        # else:
        accel = float(acceleration)

        msg.longitudinal.acceleration = accel
        msg.longitudinal.is_defined_acceleration = False

        msg.longitudinal.jerk = 0.0
        msg.longitudinal.is_defined_jerk = False

        self.control_pub.publish(msg)

        self.get_logger().info(
            f"[CONTROL] {name} vel={msg.longitudinal.velocity:.2f} "
            f"accel={msg.longitudinal.acceleration:.2f} steer={steering:.2f}"
        )

    # -------------------------------------------------------
    # Simulation Step (Scenic expects this hook)
    # -------------------------------------------------------
    def step(self, dt, controls):
        """
        Scenic calls this once per timestep.
        We simply publish controls to AWSim.

        `controls` is a dict mapping object name → { "throttle", "steer", "accel"? }.
        """
        for name, ctrl in controls.items():
            throttle = ctrl.get("throttle", 0.0)
            steer = ctrl.get("steer", 0.0)
            accel = ctrl.get("accel", None)
            self.apply_controls(name, throttle, steer, accel)

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