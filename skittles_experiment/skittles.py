import logging
from time import time
import numpy as np
from pyglet2d import Shape
from cocos.cocosnode import CocosNode
from cocos.layer import Layer
from cocos.director import director

log = logging.getLogger(__name__)


class ShapeNode(Shape, CocosNode):
    def __init__(self, *args, **kwargs):
        Shape.__init__(self, *args, **kwargs)
        CocosNode.__init__(self)


class Skittles(Layer):
    def __init__(self, target_pos, target_size, **settings):
        super().__init__()

        self.target_pos = target_pos
        self.target_size = target_size
        self.settings = settings

        self.screen_center = np.array(director.get_window_size()) / 2
        self.pivot_point = self.screen_center[0], 0
        self.angle_history = [settings['initial_paddle_angle']]
        self.time_history = [time()]
        self.ball_pos_history = []

        self.controlled = False
        self.released = False
        self.release_time = None
        self.release_angle = None
        self.release_ang_vel = None
        self.energy = None
        self.amplitude = None
        self.phase = None
        self.success = None

        self.center_post = None
        self.target = None
        self.paddle = None
        self.ball = None
        self.initial_setup()

    @property
    def paddle_angle(self):
        return self.angle_history[-1]

    @paddle_angle.setter
    def paddle_angle(self, angle):
        self._rotate_paddle(angle - self.paddle_angle, with_ball=not self.released)
        self.time_history.append(time())
        self.angle_history.append(angle)

    def relative_position(self, position):
        return (position - self.screen_center) * [1, -1]

    def absolute_position(self, position):
        return self.screen_center + position * [1, -1]

    def initial_setup(self):
        self.center_post = ShapeNode.circle(self.screen_center,
                                            radius=self.settings['post_size'],
                                            color=self.settings['post_color'])
        self.target = ShapeNode.circle(self.screen_center + self.target_pos,
                                       radius=self.target_size,
                                       color=self.settings['target_color'])
        self.paddle = ShapeNode.rectangle(
            [(self.screen_center[0] - self.settings['paddle_width'], 0),
             (self.screen_center[0] + self.settings['paddle_width'], self.settings['paddle_length'])],
            color=self.settings['paddle_color_inactive']
        )
        self.ball = ShapeNode.circle((self.screen_center[0], self.settings['paddle_length']),
                                     radius=self.settings['ball_radius'],
                                     color=self.settings['ball_color'])

        self.paddle.rotate(np.deg2rad(self.paddle_angle), center=self.pivot_point)
        self.ball.rotate(np.deg2rad(self.paddle_angle), center=self.pivot_point)
        self.ball_pos_history.append(self.ball.center)

        self.add(self.center_post)
        self.add(self.target)
        self.add(self.paddle)
        self.add(self.ball)

    def release_ball(self):
        reference_ix = -1 - self.settings.get('release_velocity_over_n_samples', 1)
        self.release_time = time()
        dt = self.release_time - self.time_history[reference_ix]

        self.release_angle = self.paddle_angle
        self.release_ang_vel = (self.paddle_angle - self.angle_history[reference_ix]) / dt

        ball_pos = self.relative_position(self.ball.center)
        ball_vel = (ball_pos - self.relative_position(self.ball_pos_history[reference_ix])) / dt

        ball_mass = self.settings['ball_mass']
        spring_constant = self.settings['spring_constant']
        self.energy = 0.5 * (ball_mass * ball_vel**2 + spring_constant * ball_pos**2)
        self.amplitude = np.sqrt(2 * self.energy / spring_constant)
        self.phase = np.arcsin(ball_pos / self.amplitude)

        self.released = True
        self.schedule(self.update_ball_trajectory)

    def update_ball_trajectory(self, dt):
        t = time() - self.release_time
        self.ball.center = self.absolute_position(
            self.amplitude *
            np.sin(self.settings['frequency'] * t + self.phase) *
            np.exp(-t / self.settings['relaxation_time'])
        )
        self.check_for_collisions()

    def check_for_collisions(self):
        if self.ball.overlaps(self.target):
            self.end_trial(success=True)
        if self.ball.overlaps(self.center_post):
            self.end_trial(success=False)

    def end_trial(self, success):
        self.success = success
        director.pop()

    def _rotate_paddle(self, angle, with_ball=True):
        self.paddle.rotate(angle, center=self.pivot_point)
        self.paddle_angle += angle
        if with_ball:
            self.ball.rotate(angle, center=self.pivot_point)
            self.ball_pos_history.append(self.ball.center)

    def activate_paddle(self):
        self.controlled = True
        self.paddle.color = self.settings['paddle_color_active']

    def deactivate_paddle(self):
        self.controlled = False
        self.paddle.color = self.settings['self.paddle_color_inactive']
