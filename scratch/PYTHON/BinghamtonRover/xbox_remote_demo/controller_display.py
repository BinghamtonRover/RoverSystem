"""
This script displays the current button press status of an Xbox controller.
"""

import pyglet
import time
import threading
import math
from controller_state import ControllerState


# Hardcoded window width and height.
GN_WINDOW_WIDTH = 1280
GN_WINDOW_HEIGHT = 720

# Hardcoded difference between radius of background and foreground joystick parts.
GF_JOYSTICK_RADIUS_DIFF = 23.9495

# Highest value of joystick axis.
GN_JOYSTICK_RANGE = 32000

# Threshold of joystick pressedness.
GN_JOYSTICK_THRESHOLD = 10000

# Prefix and suffix for the image import path.
# This will reduce string clutter when the images are loaded.
GS_IMAGE_NAME_PREFIX = "display_resources/final/"
GS_IMAGE_NAME_SUFFIX = ".png"


def load_image(as_image_name):
    """
    Loads the image with the given name. The image relative path is constructed with
    IMAGE_NAME_PREFIX + image_name + IMAGE_NAME_SUFFIX.

    :param as_image_name: The name of the image to load.
    :return: A pyglet.image.Image of the given name.
    """

    lo_image = pyglet.image.load(GS_IMAGE_NAME_PREFIX + as_image_name + GS_IMAGE_NAME_SUFFIX)
    return lo_image


def create_sprite(ao_image):
    """
    Creates a pyglet.sprite.Sprite from the given image. The sprite is placed at (0, 0).

    :param ao_image: The image to use as the Sprite's first image.
    :return: A new Sprite with the given image and the position (0, 0).
    """

    lo_sprite = pyglet.sprite.Sprite(ao_image, x=0, y=0)
    return lo_sprite


class Button:
    """
    Button represents a single Xbox controller button that has two states: on and off.
    These buttons have one sprite with two images: one for the on state and one for off.
    """

    def __init__(self, as_on_name, as_off_name):
        """
        Initializes the Button by loading its images and creating its sprite.

        :param as_on_name: The name of the on image.
        :param as_off_name: The name of the off image.
        """

        # Load and store the two Button images.
        self.co_on_image = load_image(as_on_name)
        self.co_off_image = load_image(as_off_name)

        # Load and store the Sprite.
        self.co_sprite = create_sprite(self.co_off_image)

        # Set our initial state.
        self.cb_pressed = False

    def update(self, ab_pressed):
        """
        Updates the button pressed state, which changes the Sprite's image if there is a change.

        :param ab_pressed: A boolean indicating whether the button should be pressed or not.
        """

        if ab_pressed and not self.cb_pressed:
            # We are not pressed. Swap for pressed image.
            self.cb_pressed = True
            self.co_sprite.image = self.co_on_image
        elif not ab_pressed and self.cb_pressed:
            # We are pressed. Swap for not pressed image.
            self.cb_pressed = False
            self.co_sprite.image = self.co_off_image

    def draw(self):
        """
        Draws the button.
        """

        self.co_sprite.draw()


class Dpad:
    """
    Represents a Dpad of an Xbox controller.
    """

    def __init__(self, as_off_name, as_up_name, as_down_name, as_left_name, as_right_name):
        """
        Initializes a Dpad by loading images and creating the Sprite.

        :param as_off_name: Image name of the off image.
        :param as_up_name: Image name of the up pressed image.
        :param as_down_name: Image name of the down pressed image.
        :param as_left_name: Image name of the left pressed image.
        :param as_right_name: Image name of the right pressed image.
        """

        self.co_off_image = load_image(as_off_name)
        self.co_up_image = load_image(as_up_name)
        self.co_down_image = load_image(as_down_name)
        self.co_left_image = load_image(as_left_name)
        self.co_right_image = load_image(as_right_name)

        self.co_sprite = create_sprite(self.co_off_image)

    def update(self, ab_up, ab_down, ab_left, ab_right):
        """
        Updates the state of the Dpad.

        :param ab_up: Whether up is pressed.
        :param ab_down: Whether down is pressed.
        :param ab_left: Whether left is pressed.
        :param ab_right: Whether right is pressed.
        """

        if ab_up:
            self.co_sprite.image = self.co_up_image
        elif ab_down:
            self.co_sprite.image = self.co_down_image
        elif ab_left:
            self.co_sprite.image = self.co_left_image
        elif ab_right:
            self.co_sprite.image = self.co_right_image
        else:
            # We need to show unpressed.
            self.co_sprite.image = self.co_off_image

    def draw(self):
        """
        Draws the dpad.
        """

        self.co_sprite.draw()


class Joystick:
    """
    Represents a joystick on an Xbox controller. It has two sprites: the center and the background.
    """

    def __init__(self, as_background_name, as_pressed_name, as_unpressed_name):
        """
        Initializes the Joystick, loading images and creating sprites.

        :param as_background_name: Name of background image.
        :param as_pressed_name: Name of pressed center image.
        :param as_unpressed_name: Name of unpressed center image.
        """

        self.co_background_image = load_image(as_background_name)
        self.co_pressed_image = load_image(as_pressed_name)
        self.co_unpressed_image = load_image(as_unpressed_name)

        self.co_background_sprite = create_sprite(self.co_background_image)
        self.co_foreground_sprite = create_sprite(self.co_unpressed_image)

        # Set our initial normalized x and y axes.
        self.co_x_pos = 0.0
        self.co_y_pos = 0.0

    def update(self, af_x_axis, af_y_axis):
        """
        Updates the Joystick. If x_axis and y_axis both are 0, then the Joystick is neutral.

        :param af_x_axis: Normalized (-1 to 1) Joystick x position.
        :param af_y_axis: Normalized (-1 to 1) Joystick y position.
        """

        if af_x_axis == af_y_axis == 0:
            # We are neutral. Switch to unpressed image.
            self.co_x_pos = 0.0
            self.co_y_pos = 0.0
            self.co_foreground_sprite.image = self.co_unpressed_image
            self.co_foreground_sprite.x = 0
            self.co_foreground_sprite.y = 0
        else:
            # TODO: possible speed concerns here! Setting image every update.
            # We set the correct positions.
            self.co_foreground_sprite.image = self.co_pressed_image
            self.co_x_pos = af_x_axis
            self.co_y_pos = af_y_axis

    def draw(self):
        """
        Draws the Joystick.
        """

        # Find how far from center the x and y positions are.
        lf_diff_x = GF_JOYSTICK_RADIUS_DIFF * self.co_x_pos
        lf_diff_y = GF_JOYSTICK_RADIUS_DIFF * self.co_y_pos

        # Floor for consistency.
        self.co_foreground_sprite.x = math.floor(lf_diff_x)
        self.co_foreground_sprite.y = math.floor(lf_diff_y)

        # Draw the sprites.
        self.co_background_sprite.draw()
        self.co_foreground_sprite.draw()


class Trigger:
    def __init__(self, as_on_name, as_off_name):
        """
        Initializes the Trigger by loading its images and creating its sprite.

        :param as_on_name: The name of the on image.
        :param as_off_name: The name of the off image.
        """

        # Load and store the two Button images.
        self.co_on_image = load_image(as_on_name)
        self.co_off_image = load_image(as_off_name)

        # Load and store the Sprite.
        self.co_on_sprite = create_sprite(self.co_on_image)
        self.co_off_sprite = create_sprite(self.co_off_image)

        # Set our initial state.
        self.cb_value = 0

    def update(self, ab_value):
        """
        Updates the button pressed state, which changes the Sprite's image if there is a change.

        :param ab_pressed: A boolean indicating whether the button should be pressed or not.
        """

        self.co_on_sprite.opacity = ab_value

    def draw(self):
        """
        Draws the button.
        """

        self.co_off_sprite.draw()
        self.co_on_sprite.draw()


class Controls:
    """
    A dummy class to hold all the controls of a Display.
    """
    pass


class Display:
    """
    Represents a display of a remote Xbox controller. Only one of these should be initialized.
    """

    def __init__(self):
        """
        Initializes the Display by loading all necessary images and sprites.
        """

        # A Controls containing all Buttons, Joysticks, and Dpads to be displayed.
        self.co_controls = Controls()
        self.co_controls.co_a = Button("a_pressed", "a_unpressed")
        self.co_controls.co_b = Button("b_pressed", "b_unpressed")
        self.co_controls.co_x = Button("x_pressed", "x_unpressed")
        self.co_controls.co_y = Button("y_pressed", "y_unpressed")

        self.co_controls.co_lb = Button("lb_pressed", "lb_unpressed")
        self.co_controls.co_rb = Button("rb_pressed", "rb_unpressed")
        self.co_controls.co_lt = Trigger("lt_pressed", "lt_unpressed")
        self.co_controls.co_rt = Trigger("rt_pressed", "rt_unpressed")

        self.co_controls.co_menu = Button("menu_pressed", "menu_unpressed")
        self.co_controls.co_view = Button("view_pressed", "view_unpressed")
        self.co_controls.co_xbox = Button("xbox_pressed", "xbox_unpressed")

        self.co_controls.co_dpad = Dpad("dpad_unpressed", "dpad_pressed_up", "dpad_pressed_down", "dpad_pressed_left", "dpad_pressed_right")
        self.co_controls.co_ljs = Joystick("js_left_background", "js_left_pressed", "js_left_unpressed")
        self.co_controls.co_rjs = Joystick("js_right_background", "js_right_pressed", "js_right_unpressed")

        # The pyglet window on which the Display is rendered.
        self.co_window = pyglet.window.Window(width=GN_WINDOW_WIDTH, height=GN_WINDOW_HEIGHT)

        # Register the draw event on the window.
        @self.co_window.event
        def on_draw():
            self.co_window.clear()

            # Loop through variables in self.controls.
            for ls_control_name in vars(self.co_controls):
                # Draw that control's sprite.
                vars(self.co_controls)[ls_control_name].draw()

    def update(self, ao_state: ControllerState):
        """
        Updates the visual state of all buttons according to the given ControllerState.

        :param ao_state: The new ControllerState.
        """

        # These buttons are 0 if off, 1 if on.
        self.co_controls.co_a.update(ao_state.cn_a == 1)
        self.co_controls.co_b.update(ao_state.cn_b == 1)
        self.co_controls.co_x.update(ao_state.cn_x == 1)
        self.co_controls.co_y.update(ao_state.cn_y == 1)

        self.co_controls.co_lb.update(ao_state.cn_left_bumper == 1)
        self.co_controls.co_rb.update(ao_state.cn_right_bumper == 1)

        # The triggers may be continuous.
        self.co_controls.co_lt.update(ao_state.cn_left_trigger)
        self.co_controls.co_rt.update(ao_state.cn_right_trigger)

        self.co_controls.co_menu.update(ao_state.cn_back == 1)
        self.co_controls.co_view.update(ao_state.cn_middle == 1)
        self.co_controls.co_xbox.update(ao_state.cn_start == 1)

        self.co_controls.co_dpad.update(ao_state.cn_dpad_up == 1, ao_state.cn_dpad_down == 1, ao_state.cn_dpad_left == 1, ao_state.cn_dpad_right == 1)

        # Take the threshold into account.
        lo_left_stick_x = ao_state.cn_left_stick_x if abs(ao_state.cn_left_stick_x) > GN_JOYSTICK_THRESHOLD else 0
        lo_left_stick_y = ao_state.cn_left_stick_y if abs(ao_state.cn_left_stick_y) > GN_JOYSTICK_THRESHOLD else 0

        lo_right_stick_x = ao_state.cn_right_stick_x if abs(ao_state.cn_right_stick_x) > GN_JOYSTICK_THRESHOLD else 0
        lo_right_stick_y = ao_state.cn_right_stick_y if abs(ao_state.cn_right_stick_y) > GN_JOYSTICK_THRESHOLD else 0

        # TODO: Magic numbers! These numbers will change eventually! Find the right ones!
        self.co_controls.co_ljs.update(lo_left_stick_x / GN_JOYSTICK_RANGE, -lo_left_stick_y / GN_JOYSTICK_RANGE)
        self.co_controls.co_rjs.update(lo_right_stick_x / GN_JOYSTICK_RANGE, -lo_right_stick_y / GN_JOYSTICK_RANGE)


def start(ao_cs: ControllerState):
    """
    Starts a single display, and shows updates based upon changes made in the given ControllerState.

    :param ao_cs: The ControllerState to watch for updates.
    """

    lo_display = Display()

    def update_state(_):
        lo_display.update(ao_cs)

    pyglet.clock.schedule(update_state)

    pyglet.app.run()


if __name__ == "__main__":
    cs = ControllerState()

    def mess_around(cs):
        time.sleep(2)
        cs.cn_a = 1
        time.sleep(1)
        cs.cn_dpad_left = 1
        time.sleep(1)
        cs.cn_dpad_left = 0
        cs.cn_dpad_right = 1

        t = 0
        while True:
            if t == 2 * 10 * math.pi:
                t = 0

            t += 1
            time.sleep(0.001)
            cs.cn_left_stick_x = 32000 * math.sin(t/10.0)
            cs.cn_left_stick_y = 32000 * math.cos(t/10.0)

            cs.cn_right_stick_x = 32000 * math.sin(-t / 10.0)
            cs.cn_right_stick_y = 32000 * math.cos(-t / 10.0)

    t = threading.Thread(target=mess_around, args=(cs,), daemon=True)
    t.start()

    start(cs)
