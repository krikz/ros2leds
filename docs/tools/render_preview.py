#!/usr/bin/env python3
"""Render the README pictures of ros2leds from the code in this repository.

Every picture is produced by the real code, not drawn by hand:

* the panel layout and the SPI chain path come from the real
  ``LEDMatrixCompositor`` fed with the packaged YAML config;
* the animations come from the example publishers (``effects.py``,
  ``publish_image.py``, ``spiral_fill_publisher.py``), from the PNG
  frames in ``test_frames/`` and from the ``led_matrix_effects`` package;
* the current-limiter picture uses the driver's ``frame_math``.

ROS 2, ``sounddevice`` and ``pi5neo`` are replaced by small stubs, so the
script runs on any machine with Python 3.8+, numpy, Pillow and PyYAML::

    pip install numpy pillow pyyaml
    python docs/tools/render_preview.py            # writes docs/images/

The output is deterministic (fixed random seeds, fixed time steps), so
re-running the script after a code change shows the change in ``git diff``.
"""

import argparse
import importlib.util
import math
import random
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / 'docs' / 'images'

# ----------------------------------------------------------------- look
PITCH = 18          # px between LED centres
LED_R = 6           # px, radius of a lit LED
BOARD = (16, 18, 22)
BOARD_EDGE = (44, 48, 56)
OFF_LED = (34, 36, 42)
TEXT = (200, 205, 215)
DIM_TEXT = (130, 136, 148)
ACCENT = (255, 196, 64)


def font(size):
    try:
        return ImageFont.load_default(size)
    except TypeError:            # Pillow < 10.1 has no sized default font
        return ImageFont.load_default()


# ----------------------------------------------------------------- stubs
class _Logger:
    def info(self, *_a, **_k):
        pass

    warn = warning = error = debug = info


class _Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _StubNode:
    """Just enough of ``rclpy.node.Node`` for the nodes in this repository."""

    params = {}

    def __init__(self, name, **_kw):
        self._params = {}

    def declare_parameter(self, name, value=None, descriptor=None):
        self._params.setdefault(name, _StubNode.params.get(name, value))

    def declare_parameters(self, namespace, parameters):
        for p in parameters:
            self.declare_parameter(p[0], p[1])

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def create_publisher(self, *_a, **_k):
        return _Publisher()

    def create_subscription(self, *_a, **_k):
        return None

    def create_timer(self, *_a, **_k):
        return None

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass


class _Msg:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)
        self.data = b''


def install_stubs():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
        return mod

    rclpy = module('rclpy', init=lambda *a, **k: None, spin=lambda *a, **k: None,
                   shutdown=lambda *a, **k: None)
    rclpy.node = module('rclpy.node', Node=_StubNode)
    module('std_msgs').msg = module('std_msgs.msg', Int8MultiArray=_Msg,
                                    Header=lambda: SimpleNamespace(frame_id='', stamp=None))
    module('sensor_msgs').msg = module('sensor_msgs.msg', Image=_Msg)
    module('rcl_interfaces').msg = module('rcl_interfaces.msg',
                                          ParameterDescriptor=lambda **k: SimpleNamespace(**k))
    module('sounddevice')
    for pkg in ('led_matrix_driver', 'led_matrix_compositor', 'led_matrix_effects'):
        sys.path.insert(0, str(ROOT / pkg))


def load_example(name):
    """Import an example script by file name (repo root or ``examples/``)."""
    for folder in (ROOT / 'examples', ROOT):
        path = folder / name
        if path.exists():
            spec = importlib.util.spec_from_file_location(path.stem, path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise FileNotFoundError(name)


def as_rgb(msg):
    return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)


# ----------------------------------------------------------------- LED drawing
def draw_panel(canvas, glow, x0, y0, rgb):
    """Draw one panel (H x W x 3 uint8) with its top-left LED centre at (x0, y0)."""
    d = ImageDraw.Draw(canvas)
    g = ImageDraw.Draw(glow)
    h, w = rgb.shape[:2]
    pad = PITCH // 2
    d.rounded_rectangle((x0 - pad, y0 - pad, x0 + (w - 1) * PITCH + pad, y0 + (h - 1) * PITCH + pad),
                        radius=4, fill=BOARD, outline=BOARD_EDGE)
    for y in range(h):
        for x in range(w):
            cx, cy = x0 + x * PITCH, y0 + y * PITCH
            c = tuple(int(v) for v in rgb[y, x])
            if max(c) < 8:
                d.ellipse((cx - LED_R + 2, cy - LED_R + 2, cx + LED_R - 2, cy + LED_R - 2), fill=OFF_LED)
                continue
            g.ellipse((cx - LED_R - 4, cy - LED_R - 4, cx + LED_R + 4, cy + LED_R + 4), fill=c)
            d.ellipse((cx - LED_R, cy - LED_R, cx + LED_R, cy + LED_R), fill=c)
            hi = tuple(min(255, v + 90) for v in c)
            d.ellipse((cx - 2, cy - 3, cx + 1, cy), fill=hi)


def compose(size, panels, labels=(), title=None):
    """panels: [(x0, y0, rgb)], labels: [(x, y, text, color)] -> RGB image."""
    w, h = size
    base = Image.new('RGB', (w, h), (8, 9, 12))
    glow = Image.new('RGB', (w, h), (0, 0, 0))
    leds = Image.new('RGB', (w, h), (8, 9, 12))
    for x0, y0, rgb in panels:
        draw_panel(leds, glow, x0, y0, rgb)
    glow = glow.filter(ImageFilter.GaussianBlur(7))
    base = Image.blend(base, glow, 0.55)
    mask = leds.convert('L').point(lambda v: 255 if v > 12 else 0)
    base.paste(leds, (0, 0), mask)
    d = ImageDraw.Draw(base)
    if title:
        d.text((14, 10), title, font=font(16), fill=TEXT)
    for x, y, text, color in labels:
        d.text((x, y), text, font=font(13), fill=color, anchor='mm')
    return base


def save_gif(frames, path, ms):
    pal = [f.convert('P', palette=Image.ADAPTIVE, colors=255, dither=Image.NONE) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=ms, loop=0, optimize=True, disposal=1)


# ----------------------------------------------------------------- the rover
# Packaged config: main_display 25x5 and four 8x8 wheel panels. Placement as
# on the rover this package was written for: the face (two headlights and the
# mouth) at the front, two tail lights at the back. wheel_front_left is the
# robot's left headlight, i.e. on the viewer's RIGHT when facing the robot.
GROUPS = {
    'main_display': (25, 5),
    'wheel_front_left': (8, 8),
    'wheel_front_right': (8, 8),
    'wheel_rear_left': (8, 8),
    'wheel_rear_right': (8, 8),
}
MARGIN = 34
FRONT_AT = {   # LED-grid coordinates of the top-left LED of each group
    'wheel_front_right': (0, 0),
    'main_display': (10, 1.5),
    'wheel_front_left': (37, 0),
    'wheel_rear_left': (51, 0),
    'wheel_rear_right': (61, 0),
}
ROVER_W = int(MARGIN * 2 + 68 * PITCH)
ROVER_H = int(MARGIN + 34 + 8 * PITCH + 34)


def rover_panels(images):
    out = []
    for g, (gx, gy) in FRONT_AT.items():
        out.append((int(MARGIN + gx * PITCH), int(MARGIN + 30 + gy * PITCH), images[g]))
    return out


def rover_labels():
    y = MARGIN + 30 + 8 * PITCH + 14
    lab = []
    for g, (gx, gy) in FRONT_AT.items():
        w, h = GROUPS[g]
        cx = MARGIN + (gx + (w - 1) / 2) * PITCH
        cy = MARGIN + 30 + (gy + h) * PITCH + 6 if g == 'main_display' else y
        lab.append((cx, cy, g, DIM_TEXT))
    lab.append((MARGIN + 22 * PITCH, MARGIN + 12, 'FRONT (facing the robot)', TEXT))
    lab.append((MARGIN + 60.5 * PITCH, MARGIN + 12, 'REAR (from behind)', TEXT))
    return lab


def blank(group):
    w, h = GROUPS[group]
    return np.zeros((h, w, 3), dtype=np.uint8)


def hsv_ring(t, w=8, h=8, phase=0.0):
    """Rotating rainbow ring for the 8x8 panels (generated here, not by an example)."""
    yy, xx = np.mgrid[0:h, 0:w]
    ang = (np.arctan2(yy - (h - 1) / 2, xx - (w - 1) / 2) / (2 * math.pi) + t + phase) % 1.0
    r = np.hypot(yy - (h - 1) / 2, xx - (w - 1) / 2)
    ring = (r > 1.6) & (r < 4.1)
    rgb = np.stack([np.clip(np.abs((ang * 6 + k) % 6 - 3) - 1, 0, 1) for k in (0, 4, 2)], axis=-1)
    return (rgb * ring[..., None] * 255).astype(np.uint8)


# ----------------------------------------------------------------- generators
def mouth_modes(n_per_mode=24):
    """Frames of the five mouth modes of ``effects.py`` with a synthetic audio level."""
    fx = load_example('effects.py')
    cls = fx.RobotMouthPublisher
    gens = [cls.generate_sound_mouth, cls.generate_cyber_mouth, cls.generate_plasma_mouth,
            cls.generate_equalizer_mouth, cls.generate_hologram_mouth]
    names = ['Sound Wave Mouth', 'Cyberpunk Display', 'Plasma Fire Mouth', 'Audio Equalizer',
             'Holographic Mouth']
    random.seed(7)
    me = SimpleNamespace(width=25, height=5, time_offset=0.0)
    frames = []
    for name, gen in zip(names, gens):
        for i in range(n_per_mode):
            me.time_offset += 0.15
            amp = 0.55 + 0.45 * math.sin(i * 0.5)          # stands in for the audio level
            frames.append((name, gen(me, amp)))
    return frames


def plasma_frames(n=40):
    mod = load_example('publish_image.py')
    node = mod.PlasmaPublisher()
    out = []
    for _ in range(n):
        node.timer_callback()
        out.append(as_rgb(node.publisher_.sent[-1]))
    return out


def spiral_frames(n=150):
    random.seed(3)
    mod = load_example('spiral_fill_publisher.py')
    node = mod.SpiralFillPublisher()
    out = []
    for _ in range(n):
        node.step()
        out.append(as_rgb(node.publisher.sent[-1]))
    return out


def test_frames():
    out = []
    for p in sorted((ROOT / 'test_frames').glob('*.png')):
        out.append((p.name, np.asarray(Image.open(p).convert('RGB'))))
    return out


def compositor():
    from led_matrix_compositor import led_matrix_compositor as comp
    _StubNode.params = {'config_file': str(ROOT / 'led_matrix_compositor' / 'config' / 'led_matrix_compositor.yaml')}
    return comp.LEDMatrixCompositor()


def chain_order(node):
    """Strip index -> (group, x, y): feed each group a frame whose pixels encode their own (x, y)."""
    where = {}
    for g, (w, h) in GROUPS.items():
        img = np.zeros((h, w, 3), dtype=np.uint8)
        for y in range(h):
            for x in range(w):
                img[y, x] = (x + 1, y + 1, 200)
        msg = _Msg()
        msg.header.frame_id = g
        msg.width, msg.height, msg.encoding = w, h, 'rgb8'
        msg.data = img.tobytes()
        node.buffer = bytearray(len(node.buffer))
        node.image_callback(msg)
        strip = np.frombuffer(bytes(node.buffer), dtype=np.uint8).reshape(-1, 3)
        for i, (r, gg, b) in enumerate(strip):
            if b == 200:
                where[i] = (g, int(r) - 1, int(gg) - 1)
    return [where[i] for i in sorted(where)]


# ----------------------------------------------------------------- led_matrix_effects
#: Canvas places of the groups for the effects (whole LEDs): the same picture of the rover as FRONT_AT.
EFFECTS_AT = {'wheel_front_right': (0, 0), 'main_display': (10, 2), 'wheel_front_left': (37, 0),
              'wheel_rear_left': (51, 0), 'wheel_rear_right': (61, 0)}


def effects_frames():
    """(caption, {group: rgb}, ms) frames of the led_matrix_effects package on the packaged rover layout."""
    from led_matrix_effects import Boot, Canvas, Fire, Glitch, Layout, MatrixRain, PowerDown, Rainbow, Slash, Strip
    from led_matrix_effects import lamps, signals

    layout = Layout.from_compositor(ROOT / 'led_matrix_compositor' / 'config' / 'led_matrix_compositor.yaml',
                                    at=EFFECTS_AT)
    canvas, strip = Canvas(layout), Strip(layout.n_leds)
    rainbow, fire, rain = Rainbow(canvas, level=12.0), Fire(canvas, level=12.0), MatrixRain(canvas, seed=1, level=12.0)

    def police(t):
        left, right, segs = signals.police_pattern(t)
        f = canvas.new_frame()
        f.wheel_front_right = signals.flasher(signals.RED, left)
        f.wheel_front_left = signals.flasher(signals.BLUE, right)
        for k, c in enumerate(segs):
            if c is not None:
                f.main_display[:, 4 * k + 1:4 * k + 4] = c
        f.wheel_rear_left = signals.beacon(t, signals.RED, rpm=110)
        f.wheel_rear_right = signals.beacon(t, signals.BLUE, rpm=110, phase=3.14)
        return f

    def lamp(theme):
        def fn(t):
            f = canvas.new_frame()
            f.wheel_front_right, f.wheel_front_left = lamps.THEMES[theme](t)
            f.wheel_rear_left, f.wheel_rear_right = lamps.THEMES[theme](t + 0.5)
            return f
        return fn

    def pulse(p, base):
        return lambda t: p(t, base(t))

    # (caption, effect, seconds, frames per second): slow pictures at a low rate keep the GIF small, the short
    # pulses need the full rate to be seen at all
    clips = [
        ('backgrounds.Rainbow', rainbow, 1.6, 6.25),
        ('backgrounds.Fire', fire, 1.6, 6.25),
        ('backgrounds.MatrixRain', rain, 1.6, 6.25),
        ('signals: police_pattern, flasher, beacon', police, 1.6, 6.25),
        ("lamps.THEMES['new_year']", lamp('new_year'), 1.2, 6.25),
        ("lamps.THEMES['party']", lamp('party'), 1.2, 6.25),
        ('pulses.Boot', pulse(Boot(canvas), police), 1.2, 12.5),
        ('pulses.Slash', pulse(Slash(canvas), police), 0.32, 25.0),
        ('pulses.Glitch', pulse(Glitch(canvas, seed=2), police), 0.4, 12.5),
        ('pulses.PowerDown', pulse(PowerDown(canvas), police), 0.8, 12.5),
    ]
    out = []
    for caption, fn, seconds, fps in clips:
        for i in range(int(round(seconds * fps))):
            f = fn(i / fps)
            shown = {p: (255 * (strip.to_wire(a).astype(int) * strip.brightness // 255 / strip.brightness) ** (1 / 2.2))
                     for p, a in f.panels().items()}
            out.append((caption, {p: a.astype(np.uint8) for p, a in shown.items()}, int(round(1000 / fps))))
    return out


# ----------------------------------------------------------------- pictures
def render_hero(path_png, path_gif):
    modes = mouth_modes(18)
    frames = []
    for i, (_name, mouth) in enumerate(modes):
        t = i / 36.0
        imgs = {g: blank(g) for g in GROUPS}
        imgs['main_display'] = mouth
        imgs['wheel_front_left'] = hsv_ring(t)
        imgs['wheel_front_right'] = hsv_ring(t, phase=0.5)
        on = (i // 6) % 2 == 0
        tail = np.zeros((8, 8, 3), dtype=np.uint8)
        tail[1:7, 1:7] = (200, 0, 0) if on else (60, 0, 0)
        imgs['wheel_rear_left'] = imgs['wheel_rear_right'] = tail
        frames.append(compose((ROVER_W, ROVER_H), rover_panels(imgs), rover_labels()))
    frames[20].save(path_png)
    save_gif(frames[::2], path_gif, 80)


def render_effects(path):
    frames, ms = [], []
    for caption, imgs, dt in effects_frames():
        labels = rover_labels() + [(ROVER_W / 2, ROVER_H + 2, f'led_matrix_effects: {caption}', ACCENT)]
        frames.append(compose((ROVER_W, ROVER_H + 16), rover_panels(imgs), labels))
        ms.append(dt)
    save_gif(frames, path, ms)


def render_mouth_modes(path):
    modes = mouth_modes(24)
    w, h = 25 * PITCH + 2 * MARGIN, 5 * PITCH + MARGIN + 52
    frames = []
    for name, rgb in modes:
        frames.append(compose((w, h), [(MARGIN, MARGIN + 30, rgb)],
                              [(w / 2, h - 16, f'effects.py: {name}', DIM_TEXT)],
                              title='main_display 25x5'))
    save_gif(frames, path, 70)


def render_strip_gif(frames_rgb, path, caption, ms):
    w, h = 25 * PITCH + 2 * MARGIN, 5 * PITCH + MARGIN + 52
    frames = [compose((w, h), [(MARGIN, MARGIN + 30, rgb)], [(w / 2, h - 16, caption, DIM_TEXT)],
                      title='main_display 25x5') for rgb in frames_rgb]
    save_gif(frames, path, ms)


def render_test_frames(path):
    frames = test_frames()
    w, h = 25 * PITCH + 2 * MARGIN, 5 * PITCH + MARGIN + 52
    out = [compose((w, h), [(MARGIN, MARGIN + 30, rgb)],
                   [(w / 2, h - 16, f'test_frames/{name}', DIM_TEXT)], title='main_display 25x5')
           for name, rgb in frames]
    save_gif(out, path, 600)


def render_wiring(path):
    """Physical chain order on the rover, as computed by the real compositor."""
    node = compositor()
    order = chain_order(node)
    imgs = {g: blank(g) for g in GROUPS}
    n = len(order)
    for i, (g, x, y) in enumerate(order):
        hue = 0.8 * i / n
        rgb = [np.clip(abs((hue * 6 + k) % 6 - 3) - 1, 0, 1) for k in (0, 4, 2)]
        imgs[g][y, x] = [int(v * 235) + 20 for v in rgb]
    panels = rover_panels(imgs)
    extra_h = 70
    img = compose((ROVER_W, ROVER_H + extra_h), panels, rover_labels())
    d = ImageDraw.Draw(img)
    pos = {g: (x0, y0) for (x0, y0, _), g in zip(panels, FRONT_AT)}
    pts = [(pos[g][0] + x * PITCH, pos[g][1] + y * PITCH) for g, x, y in order]
    for a, b in zip(pts, pts[1:]):
        d.line((a, b), fill=(255, 255, 255), width=1)
    sx, sy = pts[0]
    d.ellipse((sx - 9, sy - 9, sx + 9, sy + 9), outline=ACCENT, width=2)
    d.text((sx, sy - 18), 'DIN (LED 0)', font=font(12), fill=ACCENT, anchor='mm')
    ex, ey = pts[-1]
    d.text((ex, ey + 20), f'LED {n - 1}', font=font(12), fill=ACCENT, anchor='mm')
    d.text((14, ROVER_H + 18),
           f'SPI chain order of all {n} LEDs (colour = position in the chain, red -> purple), '
           'computed by led_matrix_compositor', font=font(13), fill=TEXT)
    d.text((14, ROVER_H + 40),
           'from the packaged YAML: 5x5 panels wired as a snake, mouth reversed and mirrored, '
           'front headlights rotated 180 deg.', font=font(13), fill=DIM_TEXT)
    img.save(path)
    return n


def render_limiter(path):
    """Full white at brightness 255: what is asked vs what the driver writes."""
    from led_matrix_driver import frame_math as fm
    n, limit, ma_ch, idle = 381, 4500.0, 20.0, 1.0
    values = bytes([255] * n * 3)
    asked = fm.estimate_current_ma(values, n, ma_ch, idle)
    limited, scale = fm.limit_current(values, n, limit, ma_ch, idle)
    got = fm.estimate_current_ma(limited, n, ma_ch, idle)
    lv = limited[0]
    # LEDs look brighter than their PWM duty: draw with display gamma 2.2 so the
    # dimmed strip is shown roughly as the eye sees it (255 stays 255).
    shown = int(round(255 * (lv / 255) ** (1 / 2.2)))
    imgs_a = {g: np.full((GROUPS[g][1], GROUPS[g][0], 3), 255, np.uint8) for g in GROUPS}
    imgs_b = {g: np.full((GROUPS[g][1], GROUPS[g][0], 3), shown, np.uint8) for g in GROUPS}
    w = ROVER_W
    top = compose((w, ROVER_H), rover_panels(imgs_a), [],
                  title=f'requested: full white, brightness 255  ->  {asked / 1000:.1f} A estimated')
    bottom = compose((w, ROVER_H), rover_panels(imgs_b), [],
                     title=f'written to the strip: every channel x{scale:.3f} ({lv}/255)  ->  '
                           f'{got / 1000:.2f} A <= {limit / 1000:.1f} A limit  (drawn with gamma 2.2)')
    img = Image.new('RGB', (w, ROVER_H * 2), (8, 9, 12))
    img.paste(top, (0, 0))
    img.paste(bottom, (0, ROVER_H))
    img.save(path)
    return asked, scale, lv, got


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', type=Path, default=OUT_DIR, help='output directory (default: docs/images)')
    args = ap.parse_args(argv)
    install_stubs()
    out = args.out
    out.mkdir(parents=True, exist_ok=True)

    render_hero(out / 'rover.png', out / 'rover.gif')
    render_effects(out / 'led_matrix_effects.gif')
    render_mouth_modes(out / 'effects_mouth_modes.gif')
    render_strip_gif(plasma_frames(), out / 'plasma.gif', 'publish_image.py: plasma', 90)
    render_strip_gif(spiral_frames(), out / 'spiral.gif', 'spiral_fill_publisher.py', 60)
    render_test_frames(out / 'test_frames.gif')
    n = render_wiring(out / 'wiring.png')
    asked, scale, lv, got = render_limiter(out / 'current_limiter.png')

    print(f'chain: {n} LEDs')
    print(f'limiter: full white {asked:.0f} mA -> scale {scale:.4f} (channel {lv}) -> {got:.0f} mA')
    for p in sorted(out.iterdir()):
        shown = p.relative_to(ROOT).as_posix() if ROOT in p.resolve().parents else p
        print(f'{shown}  {p.stat().st_size // 1024} KiB')


if __name__ == '__main__':
    main()
