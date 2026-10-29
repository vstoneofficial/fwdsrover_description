#!/usr/bin/env python3
"""Four independent wheel units, configured exclusively by robot_description."""
import math
import signal
import xml.etree.ElementTree as ET
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def mv(matrix, vector):
    return tuple(sum(a*b for a, b in zip(row, vector)) for row in matrix)


def mm(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def model_units(description):
    """Read the actual frames, radii, actuator rates and limits from URDF."""
    robot = ET.fromstring(description)
    joints = {j.get('name'): j for j in robot.findall('joint')}
    by_child = {j.find('child').get('link'): j for j in joints.values()}
    controls = {p.findtext('joint_name'): p for p in robot.findall(
        "gazebo/plugin[@name='gz::sim::systems::JointController']")}
    identity = rotation((0., 0., 0.))

    def transform(link):
        if link not in by_child:
            return identity, (0., 0., 0.)
        joint = by_child[link]
        parent_r, parent_t = transform(joint.find('parent').get('link'))
        origin = joint.find('origin')
        xyz = tuple(map(float, origin.get('xyz', '0 0 0').split()))
        rpy = tuple(map(float, origin.get('rpy', '0 0 0').split()))
        delta = mv(parent_r, xyz)
        return mm(parent_r, rotation(rpy)), tuple(a+b for a, b in zip(parent_t, delta))

    units = []
    for prefix in ('front_left', 'front_right', 'back_right', 'back_left'):
        sn, wn = prefix + '_steering_joint', prefix + '_wheel_joint'
        steering, wheel = joints[sn], joints[wn]
        sr, pivot = transform(steering.find('child').get('link'))
        axis = mv(sr, tuple(map(float, steering.find('axis').get('xyz').split())))
        if abs(axis[2] - 1.) > 1e-5:
            raise ValueError(f'{sn}: expected an upward steering axis')
        wr, wheel_origin = transform(wheel.find('child').get('link'))
        axis = mv(wr, tuple(map(float, wheel.find('axis').get('xyz').split())))
        heading = math.atan2(-axis[0], axis[1])  # rolling direction: axis cross up
        link = robot.find(f"link[@name='{wheel.find('child').get('link')}']")
        collision = link.find('collision')
        cylinder = collision.find('geometry/cylinder')
        radius = float(cylinder.get('radius'))
        origin = collision.find('origin')
        local_center = tuple(map(float, origin.get('xyz', '0 0 0').split())) if origin is not None else (0., 0., 0.)
        center = tuple(a+b for a, b in zip(wheel_origin, mv(wr, local_center)))
        center_offset = tuple(center[i] - pivot[i] for i in (0, 1))
        slimit, wlimit = steering.find('limit'), wheel.find('limit')
        units.append(dict(steer=sn, wheel=wn, pivot=pivot[:2], heading=heading, radius=radius,
                          center_offset=center_offset, half_width=float(cylinder.get('length')) * .5,
                          lower=float(slimit.get('lower')), upper=float(slimit.get('upper')),
                          steer_speed=float(slimit.get('velocity')), wheel_speed=float(wlimit.get('velocity')),
                          steer_acceleration=float(controls[sn].findtext('max_acceleration')),
                          wheel_acceleration=float(controls[wn].findtext('max_acceleration')),
                          position_gain=float(controls[sn].findtext('position_p_gain'))))
    return units


def wheel_targets(units, vx, vy, wz, current):
    targets = []
    for unit, position in zip(units, current):
        x, y = unit['pivot']
        wx, wy = vx-wz*y, vy+wz*x
        speed = math.hypot(wx, wy) / unit['radius']
        if speed < 1e-9:
            angle = position
        else:
            angle = math.atan2(wy, wx) - unit['heading']
            choices = []
            for sign, target in ((1., angle), (-1., angle+math.pi)):
                for turns in range(-2, 3):
                    value = target + turns*2*math.pi
                    if unit['lower'] <= value <= unit['upper']:
                        choices.append((abs(value-position), value, sign*speed))
            _, angle, _ = min(choices)
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = unit['center_offset']
        cx, cy = x + c*dx - s*dy, y + s*dx + c*dy
        heading = unit['heading'] + angle
        rolling_speed = ((vx-wz*cy)*math.cos(heading)
                         + (vy+wz*cx)*math.sin(heading))
        # A finite tire contacts the floor on an axial line. Use the limiting
        # rolling demand at its endpoints, including the actual mounting offset.
        edge_speed = wz * unit['half_width']
        speed = max((rolling_speed-edge_speed, rolling_speed+edge_speed), key=abs) / unit['radius']
        targets.append((angle, speed))
    scale = max([1.] + [abs(speed)/u['wheel_speed'] for u, (_, speed) in zip(units, targets)])
    return [(angle, speed/scale) for angle, speed in targets]


def slew(previous, target, acceleration, dt):
    limit = acceleration * dt
    return previous + max(-limit, min(limit, target-previous))


class RoverTwistRelay(Node):
    def __init__(self):
        super().__init__('rover_twist_relay')
        self.units = model_units(self.declare_parameter('robot_description', '').value)
        self.positions = {}
        self.command = (0., 0., 0.)
        self.previous = {name: 0. for u in self.units for name in (u['steer'], u['wheel'])}
        self.publishers_by_joint = {name: self.create_publisher(Float64, '/fwds/'+name+'/cmd_vel', 10)
                                   for name in self.previous}
        self.targets_publisher = self.create_publisher(JointState, '/actuator_targets', 10)
        self.create_subscription(JointState, '/joint_states', self.state, 10)
        self.create_subscription(Twist, '/rover_twist', self.command_callback, 10)
        self.last_time = None
        self.create_timer(.005, self.tick)

    def state(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def command_callback(self, msg):
        self.command = (msg.linear.x, msg.linear.y, msg.angular.z)

    def tick(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds-self.last_time)/1e9 if self.last_time is not None else .005
        self.last_time = now.nanoseconds
        if dt <= 0 or not all(u['steer'] in self.positions for u in self.units):
            return
        current = [self.positions[u['steer']] for u in self.units]
        targets = wheel_targets(self.units, *self.command, current)
        diagnostic = JointState()
        diagnostic.header.stamp = now.to_msg()
        for unit, position, (angle, speed) in zip(self.units, current, targets):
            steering = max(-unit['steer_speed'], min(unit['steer_speed'], (angle-position)*unit['position_gain']))
            steering = slew(self.previous[unit['steer']], steering, unit['steer_acceleration'], dt)
            speed = slew(self.previous[unit['wheel']], speed, unit['wheel_acceleration'], dt)
            for name, velocity, target in ((unit['steer'], steering, angle), (unit['wheel'], speed, 0.)):
                self.previous[name] = velocity
                self.publishers_by_joint[name].publish(Float64(data=velocity))
                diagnostic.name.append(name)
                diagnostic.position.append(target)
                diagnostic.velocity.append(velocity)
        self.targets_publisher.publish(diagnostic)


def main():
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RoverTwistRelay()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
