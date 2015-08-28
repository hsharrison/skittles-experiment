from collections import defaultdict, deque
from contextlib import contextmanager
import functools
import logging
import time
import numpy as np
from touchtable.vrpn import PolhemusLibertyLatus, VrpnServer

log = logging.getLogger(__name__)


class TrackerJoint:
    # Lower number tracker should be placed on pivot point.
    initial_position_average_over_n_samples = 5

    def __init__(self, callback=None):
        self.history = []
        self.user_callback = callback

        self._marker_map = {}
        self._temporary_physical_marker_raw_data = {}
        self._raw_data = {}
        self._physical_markers_seen = set()
        self._initial_positions = defaultdict(list)
        self._active_callback = self.get_marker_map
        self._later_callbacks = deque([
            self.register_raw_data,
        ])

        self._tracker = PolhemusLibertyLatus(2)
        self._tracker.set_handler('position', self.handle_input)
        log.info('waiting to see all markers')

    @property
    def angle(self):
        return self.history[-1]['angle']

    @property
    def radius(self):
        return self.history[-1]['radius']

    def handle_input(self, data, _):
        result = self._active_callback(data, _)
        if result == 'COMPLETE':
            self._active_callback = self._later_callbacks.popleft()

    @contextmanager
    def running(self):
        with VrpnServer(self._tracker, sentinel='ready'):
            yield

    def get_marker_map(self, data, _):
        physical_marker = data.pop('sensor', 0)
        self._temporary_physical_marker_raw_data[physical_marker] = data

        if physical_marker not in self._physical_markers_seen:
            log.info('marker {} detected'.format(physical_marker))
            self._physical_markers_seen.add(physical_marker)

        if len(self._physical_markers_seen) == self._tracker.n_sensors:
            log.info('all markers seen')
            self._marker_map = {
                physical_number: logical_number
                for logical_number, physical_number in enumerate(sorted(self._physical_markers_seen))
            }

            inverse_marker_map = {logical: physical for physical, logical in self._marker_map.items()}
            self._raw_data = {logical_marker:
                              self._temporary_physical_marker_raw_data[inverse_marker_map[logical_marker]]
                              for logical_marker in self._marker_map.values()}

            return 'COMPLETE'

    def append_data(self, angle, radius):
        self.history.append({
            'time': time.time(),
            'angle': angle,
            'radius': radius,
        })

        if self.user_callback:
            self.user_callback(angle)

    @extract_logical_marker
    def register_raw_data(self, marker, data):
        self._raw_data[marker] = data
        self.append_data(*angle_and_radius(
            np.array(self._raw_data[0]['position']),
            np.array(self._raw_data[1]['position']),
        ))


def angle_and_radius(fulcrum, tip):
    diff = tip - fulcrum
    angle = np.arctan2(diff[1], diff[0])
    radius = np.linalg.norm(diff)
    return angle, radius


def extract_logical_marker(func):
    """Change a method of TrackerJoint from the format
    method(self, data, _)
    into
    method(self, logical_marker, data)

    """
    @functools.wraps
    def wrapped(self, data, _):
        physical_marker = data.pop('sensor')
        logical_marker = self.marker_map[physical_marker]
        return func(self, logical_marker, data)
    return wrapped
