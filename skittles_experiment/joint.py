from collections import defaultdict, deque
from contextlib import contextmanager
import functools
import logging
import time
import numpy as np
from pyglet.event import EVENT_UNHANDLED, EVENT_HANDLED, EventDispatcher
from pyglet.window import mouse
from touchtable.vrpn import PolhemusLibertyLatus, VrpnServer

log = logging.getLogger(__name__)


class SkittlesController:
    def __init__(self, skittles, joint):
        self.skittles = skittles
        self.joint = joint

    @property
    def within_tolerance(self):
        return abs(self.joint.history[0] - self.skittles.joint_angle) < self.skittles.settings['angle_tolerance']

    def on_mouse_press(self, x, y, button, modifiers):
        # Do nothing if not left button.
        if button is not mouse.LEFT:
            return EVENT_UNHANDLED

        if not self.joint.active:
            return EVENT_UNHANDLED

        if self.skittles.released:
            return EVENT_UNHANDLED

        if self.skittles.controlled:
            # Should never happen this way.
            return EVENT_UNHANDLED

        if self.within_tolerance:
            self.skittles.activate_paddle()
            return EVENT_HANDLED

        return EVENT_UNHANDLED

    def on_mouse_release(self, x, y, button, modifiers):
        if button is not mouse.LEFT:
            return EVENT_UNHANDLED

        if not self.joint.active:
            return EVENT_UNHANDLED

        if self.skittles.released:
            return EVENT_UNHANDLED

        if self.skittles.controlled:
            self.skittles.release_ball()
            return EVENT_HANDLED

        return EVENT_UNHANDLED

    def on_joint_move(self, d_angle):
        if self.skittles.controlled:
            self.skittles.angle += d_angle


class TrackerJoint(EventDispatcher):
    # Lower number tracker should be placed on pivot point.
    def __init__(self):
        self.history = []
        self.active = False

        self._marker_map = {}
        self._temporary_physical_marker_raw_data = {}
        self._raw_data = {}
        self._physical_markers_seen = set()
        self._initial_positions = defaultdict(list)
        self._callbacks = deque([
            self.get_marker_map,
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
        if self._callbacks:
            result = self._callbacks[0](data, _)

            if result == 'COMPLETE':
                self._callbacks.popleft()

    @contextmanager
    def running(self):
        with VrpnServer(self._tracker, sentinel='ready'):
            self.active = True
            yield
            self.active = False

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
        self.dispatch_event('on_joint_move', angle - self.angle)
        self.history.append({
            'time': time.time(),
            'angle': angle,
            'radius': radius,
        })

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
