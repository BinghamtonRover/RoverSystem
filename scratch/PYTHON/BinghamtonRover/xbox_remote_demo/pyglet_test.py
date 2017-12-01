import pyglet


def load_image(file):
    image = pyglet.image.load(file)
    # image.anchor_x = image.width // 2
    # image.anchor_y = image.height // 2
    return image


def load_sprite(image, x, y, w, h):
    sprite = pyglet.sprite.Sprite(image, x=x, y=y)
    sprite.scale_x = w/sprite.width
    sprite.scale_y = h/sprite.height
    return sprite


path_prefix = "display_resources/final/"
path_suffix = ".png"


def load_at_zero(name):
    return load_sprite(load_image(path_prefix + name + path_suffix), 0, 0, 1280, 720)


window = pyglet.window.Window(width=1280, height=720)

sprites = [
    load_at_zero("a_pressed"),
    load_at_zero("b_pressed"),
    load_at_zero("x_pressed"),
    load_at_zero("y_pressed"),
    load_at_zero("dpad_pressed_down"),
    load_at_zero("js_left_background"),
    load_at_zero("js_right_background"),
    load_at_zero("js_left_pressed"),
    load_at_zero("js_right_pressed"),
    load_at_zero("menu_pressed"),
    load_at_zero("view_pressed"),
    load_at_zero("xbox_pressed"),
    load_at_zero("lb_pressed"),
    load_at_zero("lt_pressed"),
    load_at_zero("rb_pressed"),
    load_at_zero("rt_pressed"),
]

@window.event
def on_draw():
    window.clear()
    for sprite in sprites:
        sprite.draw()


pyglet.app.run()

"""import pyglet
import math


def load_image(file):
    image = pyglet.image.load(file)
    image.anchor_x = image.width // 2
    image.anchor_y = image.height // 2
    return image


def load_sprite(image, x, y, w, h):
    sprite = pyglet.sprite.Sprite(image, x=x, y=y)
    sprite.scale_x = w / sprite.width
    sprite.scale_y = h / sprite.height
    return sprite


class Joystick:
    def __init__(self, bg_image, unpressed_image, pressed_image, x, y, w, h, scale):
        self.pressed = False

        self.bg_sprite = load_sprite(bg_image, x, y, w, h)
        self.c_sprite = pyglet.sprite.Sprite(unpressed_image, x, y)

        self.pressed_image = pressed_image
        self.unpressed_image = unpressed_image

        self.c_sprite.scale_x = scale * self.bg_sprite.scale_x
        self.c_sprite.scale_y = scale * self.bg_sprite.scale_y

        self.c_sprite.x = self.bg_sprite.x
        self.c_sprite.y = self.bg_sprite.y

        self.c_initial_x = self.c_sprite.x
        self.c_initial_y = self.c_sprite.y

    def set_angle(self, angle):
        if not self.pressed:
            return

        self.c_sprite.x = self.c_initial_x + (self.bg_sprite.width - self.c_sprite.width) / 2 * math.cos(angle)
        self.c_sprite.y = self.c_initial_y + (self.bg_sprite.height - self.c_sprite.height) / 2 * math.sin(angle)

    def set_pressed(self, pressed):
        if pressed and not self.pressed:
            self.pressed = True
            self.c_sprite.image = self.pressed_image
        elif not pressed and self.pressed:
            self.c_sprite.x = self.bg_sprite.x
            self.c_sprite.y = self.bg_sprite.y
            self.pressed = False
            self.c_sprite.image = self.unpressed_image

    def draw(self):
        self.bg_sprite.draw()
        self.c_sprite.draw()


drawables = []

js = Joystick(
    load_image("display_resources/joystick_center_background.png"),
    load_image("display_resources/joystick_center_unpressed.png"),
    load_image("display_resources/joystick_center_pressed.png"),
    640, 360, 300, 300, 0.6
)

drawables.append(js)

window = pyglet.window.Window(width=1280, height=720)


@window.event
def on_draw():
    window.clear()
    for d in drawables:
        d.draw()


@window.event
def on_mouse_motion(x, y, dx, dy):
    global js
    js_center_x = js.c_initial_x
    js_center_y = js.c_initial_y

    new_angle = math.atan2(y - js_center_y, x - js_center_x)

    js.set_angle(new_angle)


@window.event
def on_key_press(symbol, modifiers):
    if symbol == 32:
        global js
        js.set_pressed(not js.pressed)


pyglet.app.run()
"""