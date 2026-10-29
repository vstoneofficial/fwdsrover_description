"""Drain native rendering before Gazebo stops; invoked once by the launch file."""
import json
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node


def main():
    deadline = float(sys.argv[1])
    node = Node()
    armed = threading.Event()
    drained = threading.Event()
    paused_updates = 0

    def statistics(message):
        nonlocal paused_updates
        if armed.is_set() and message.paused:
            paused_updates += 1
            if paused_updates >= 2:
                drained.set()

    def request(service, message):
        remaining_ms = int((deadline - time.monotonic()) * 1000)
        if remaining_ms <= 0:
            raise RuntimeError('Gazebo shutdown deadline reached')
        success, response = node.request(
            service, message, type(message), Boolean, remaining_ms)
        if not success or not response.data:
            raise RuntimeError(f'Gazebo did not acknowledge {service}')

    topic = '/world/fwdsrover/stats'
    request('/world/fwdsrover/control', WorldControl(pause=True))
    # Finish the synchronous request before installing Python callbacks.
    node.subscribe(WorldStatistics, topic, statistics)
    armed.set()
    if not drained.wait(max(0, deadline - time.monotonic())):
        raise RuntimeError('Gazebo did not report two paused updates')
    node.unsubscribe(topic)
    print(json.dumps({'paused_updates': paused_updates}), flush=True)


if __name__ == '__main__':
    main()
