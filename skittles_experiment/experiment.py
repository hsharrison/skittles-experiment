from contextlib import contextmanager
import sys
import yaml
import pyglet
from cocos.director import director
from cocos.scene import Scene

from skittles import Skittles
from joint import TrackerJoint, SkittlesController


import logging
logging.basicConfig(level=logging.INFO)


def get_screen(screen_ix):
    display = pyglet.window.get_platform().get_default_display()
    return display.get_screens()[screen_ix]

def trial(*args, screen=0, **kwargs):
    director.init(fullscreen=True, screen=get_screen(screen))
    with exclusive_mode():
        skittles = Skittles(*args, **kwargs)
        joint = TrackerJoint()
        controller = SkittlesController(skittles, joint)
        with joint.running():
            with handlers(joint, controller):
                director.run(Scene(skittles))

    return skittles.release_angle, skittles.release_ang_vel, skittles.success


@contextmanager
def exclusive_mode():
    try:
        director.window.set_exclusive_keyboard(True)
        director.window.set_exclusive_mouse(True)
        director.window.set_mouse_visible(False)
        yield

    finally:
        director.window.set_exclusive_keyboard(False)
        director.window.set_exclusive_mouse(False)
        director.window.set_mouse_visible(True)


@contextmanager
def handlers(joint, controller):
    director.window.push_handlers(controller)
    joint.push_handlers(controller)
    yield
    joint.remove_handlers(controller)
    director.window.remove_handlers(controller)


def main():
    target_x = int(sys.argv[1])
    target_y = int(sys.argv[2])
    target_size = 10

    with open('settings.yml') as f:
        settings = yaml.load(f)

    # Colors must be tuples, not lists (they must be hashable).
    for color in ['post_color', 'target_color', 'paddle_color_inactive', 'paddle_color_active', 'ball_color']:
        settings[color] = tuple(settings[color])

    angle, ang_vel, success = trial([target_x, target_y], target_size, **settings)
    print('angle @ release:', angle)
    print('angular velocity @ release:', ang_vel)
    print('success': success)


if __name__ == '__main__':
    main()
