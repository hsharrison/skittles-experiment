import logging
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
    is_event_handler = True

    def __init__(self, target_pos, target_size, **settings):
        super().__init__()

        self.target_pos = target_pos
        self.target_size = target_size
        self.settings = settings

        self.screen_center = np.array(director.get_window_size()) / 2
        self.pivot_point = self.screen_center[0], 0
        self.paddle_angle = 0

        self.center_post = None
        self.target = None
        self.paddle = None
        self.ball = None

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
            color=self.settings['paddle_color']
        )
        self.ball = ShapeNode.circle((self.screen_center[0], self.settings['paddle_length']),
                                     radius=self.settings['ball_radius'],
                                     color=self.settings['ball_color'])

        self.paddle.rotate(np.deg2rad(self.settings['initial_paddle_angle']), center=self.pivot_point)
        self.ball.rotate(np.deg2rad(self.settings['initial_paddle_angle']), center=self.pivot_point)
        self.pivot_point = self.settings['initial_paddle_angle']

        self.add(self.center_post)
        self.add(self.target)
        self.add(self.paddle)
        self.add(self.ball)

    def rotate_paddle(self, angle, with_ball=True):
        self.paddle.rotate(angle, center=self.pivot_point)
        self.paddle_angle += angle
        if with_ball:
            self.ball.rotate(angle, center=self.pivot_point)
