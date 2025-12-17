import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.parameter import Parameter


class RoverTwistRelay(Node):
    def __init__(self) -> None:
        super().__init__('rover_twist_relay')

        # ----------------------------
        # Parameters (declare all)
        # ----------------------------
        self.publish_rate_hz = self.declare_parameter('publish_rate', 30.0).value

        # rover selector
        self.rover = self.declare_parameter('rover', 'x40a').value

        # fallback (default)
        self.rover_d = self.declare_parameter('rover_d', 0.125).value
        self.rover_hb = self.declare_parameter('rover_hb', 0.125).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.14).value

        # per-rover parameters (flat names)
        self.declare_parameter('x40a.rover_d', 0.125)
        self.declare_parameter('x40a.rover_hb', 0.125)
        self.declare_parameter('x40a.wheel_radius', 0.14)

        self.declare_parameter('x120a.rover_d', 0.165)
        self.declare_parameter('x120a.rover_hb', 0.175)
        self.declare_parameter('x120a.wheel_radius', 0.165)

        # ----------------------------
        # Apply rover-specific dimensions
        # ----------------------------
        self.apply_rover_dimensions()

        # dynamics
        self.steer_time_constant = self.declare_parameter('steer_tau', 0.08).value
        self.wheel_time_constant = self.declare_parameter('wheel_tau', 0.05).value

        self.steer_limit_deg = self.declare_parameter('steer_limit_deg', 95.0).value
        self.steer_limit_rad = self.steer_limit_deg * math.pi / 180.0

        # topics
        self.cmd_topic = self.declare_parameter('cmd_topic', '/rover_twist').value
        self.steer_cmd_topic = self.declare_parameter(
            'steer_cmd_topic', '/steer_position_controller/commands'
        ).value
        self.wheel_cmd_topic = self.declare_parameter(
            'wheel_cmd_topic', '/wheel_velocity_controller/commands'
        ).value

        # wheel index
        self.F_R = 0
        self.F_L = 1
        self.R_L = 2
        self.R_R = 3

        # states
        self.avr_v = [0.0] * 4
        self.steer_pos_now = [0.0] * 4
        self.steer_center = [0.0] * 4
        self.wheel_angle = [0.0] * 4

        self.last_cmd = None
        self.last_cmd_time = None

        self.timer_period = 1.0 / self.publish_rate_hz
        self.prev_time = None

        # ROS
        self.create_subscription(Twist, self.cmd_topic, self.twist_callback, 10)
        self.pub_steer_cmd = self.create_publisher(Float64MultiArray, self.steer_cmd_topic, 10)
        self.pub_wheel_cmd = self.create_publisher(Float64MultiArray, self.wheel_cmd_topic, 10)

        self.timer = self.create_timer(self.timer_period, self.on_timer)

    # ----------------------------------------
    def apply_rover_dimensions(self) -> None:
        # Explicitly map rover-specific parameters to active parameters

        prefix = f'{self.rover}.'

        params = self._parameters  # internal storage (rclpy)

        def get_param(name, default):
            full = prefix + name
            if full in params:
                return params[full].value
            return default

        new_rover_d = get_param('rover_d', self.rover_d)
        new_rover_hb = get_param('rover_hb', self.rover_hb)
        new_wheel_radius = get_param('wheel_radius', self.wheel_radius)

        self.set_parameters([
            Parameter('rover_d', Parameter.Type.DOUBLE, new_rover_d),
            Parameter('rover_hb', Parameter.Type.DOUBLE, new_rover_hb),
            Parameter('wheel_radius', Parameter.Type.DOUBLE, new_wheel_radius),
        ])

        # update cached values
        self.rover_d = new_rover_d
        self.rover_hb = new_rover_hb
        self.wheel_radius = new_wheel_radius

    # ----------------------------------------
    def twist_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    # ----------------------------------------
    def on_timer(self) -> None:
        now = self.get_clock().now()
        if self.prev_time is None:
            dt = self.timer_period
        else:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt <= 0:
                dt = self.timer_period
        self.prev_time = now

        if self.last_cmd is None:
            self.publish_commands()
            return

        cmd_vx = float(self.last_cmd.linear.x)
        cmd_vy = float(self.last_cmd.linear.y)
        cmd_w = float(self.last_cmd.angular.z)

        is_pure_rot = (abs(cmd_vx) < 1e-6 and abs(cmd_vy) < 1e-6 and abs(cmd_w) > 1e-6)

        rot_gain = ((self.rover_hb * 0.5) + self.rover_d) / math.sqrt(
            (self.rover_hb * 0.5) ** 2 + self.rover_d ** 2
        )

        if is_pure_rot:
            rot_gain *= 1.88

        pos_x = [0.0] * 4
        pos_y = [0.0] * 4
        pos_x[self.F_R] = +self.rover_hb / 2.0
        pos_y[self.F_R] = -self.rover_d
        pos_x[self.F_L] = +self.rover_hb / 2.0
        pos_y[self.F_L] = +self.rover_d
        pos_x[self.R_L] = -self.rover_hb / 2.0
        pos_y[self.R_L] = +self.rover_d
        pos_x[self.R_R] = -self.rover_hb / 2.0
        pos_y[self.R_R] = -self.rover_d

        target_avr_v = [0.0] * 4
        target_steer = [0.0] * 4

        for i in range(4):
            vwx = cmd_vx - (rot_gain * 0.75 * cmd_w) * pos_y[i]
            vwy = cmd_vy + (rot_gain * 0.75 * cmd_w) * pos_x[i]

            speed = math.hypot(vwx, vwy)
            if speed > 1e-6:
                direction = math.atan2(vwy, vwx)
            else:
                direction = math.atan2(-pos_x[i], pos_y[i])

            target_avr_v[i] = speed
            target_steer[i] = direction

        alpha_s = 1.0 - math.exp(-dt / max(1e-6, self.steer_time_constant))
        alpha_w = 1.0 - math.exp(-dt / max(1e-6, self.wheel_time_constant))

        for i in range(4):
            s_out, v_out = self.normalize_with_limit_and_speed(
                current_angle=self.steer_pos_now[i],
                center_angle=self.steer_center[i],
                target_angle=target_steer[i],
                limit_rad=self.steer_limit_rad,
                speed=target_avr_v[i],
            )

            self.steer_pos_now[i] = self.step_toward_angle(
                self.steer_pos_now[i], s_out, alpha_s
            )
            self.avr_v[i] = self.avr_v[i] + alpha_w * (v_out - self.avr_v[i])

        if self.wheel_radius > 1e-9:
            for i in range(4):
                self.wheel_angle[i] += (self.avr_v[i] / self.wheel_radius) * dt

        self.publish_commands()

    # ----------------------------------------
    def publish_commands(self) -> None:
        map_idx = [self.F_L, self.F_R, self.R_R, self.R_L]

        steer_msg = Float64MultiArray()
        steer_msg.data = [self.steer_pos_now[idx] for idx in map_idx]
        self.pub_steer_cmd.publish(steer_msg)

        wheel_msg = Float64MultiArray()
        for idx in map_idx:
            ang_vel = self.avr_v[idx] / self.wheel_radius if self.wheel_radius > 1e-9 else 0.0
            if abs(ang_vel) < 1e-8:
                ang_vel = 0.0
            wheel_msg.data.append(ang_vel)
        self.pub_wheel_cmd.publish(wheel_msg)

    # ----------------------------------------
    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a <= -math.pi:
            a += 2 * math.pi
        return a

    def step_toward_angle(self, cur, target, alpha):
        cur = self.normalize_angle(cur)
        tgt = self.normalize_angle(target)
        diff = tgt - cur
        if diff > math.pi:
            diff -= 2 * math.pi
        if diff < -math.pi:
            diff += 2 * math.pi
        return self.normalize_angle(cur + alpha * diff)

    def normalize_with_limit_and_speed(
        self, current_angle, center_angle, target_angle, limit_rad, speed
    ):
        def wrap_to_near(tgt, cur):
            while tgt - cur > math.pi:
                tgt -= 2 * math.pi
            while tgt - cur < -math.pi:
                tgt += 2 * math.pi
            return tgt

        def signed_diff(ang):
            d = ang - center_angle
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            return d

        a = wrap_to_near(target_angle, current_angle)
        b = wrap_to_near(target_angle + math.pi, current_angle)

        use_ang = a
        use_v = speed
        if abs(signed_diff(b)) < abs(signed_diff(a)):
            use_ang = b
            use_v = -speed

        d = signed_diff(use_ang)
        if d > limit_rad:
            use_ang = center_angle + limit_rad
        if d < -limit_rad:
            use_ang = center_angle - limit_rad

        return use_ang, use_v


def main() -> None:
    rclpy.init()
    node = RoverTwistRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

