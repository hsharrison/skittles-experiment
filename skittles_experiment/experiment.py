from contextlib import contextmanager
from cocos.director import director
from cocos.scene import Scene

from .skittles import Skittles
from .joint import TrackerJoint, SkittlesController


def trial(*args, **kwargs):
    director.init(fullscreen=True)
    with exclusive_mode():
        skittles = Skittles(*args, **kwargs)
        joint = TrackerJoint()
        controller = SkittlesController(skittles, joint)
        with joint.running():
            with handlers(joint, controller):
                director.run(Scene(Skittles))


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
    director.window.remove_handler(controller)