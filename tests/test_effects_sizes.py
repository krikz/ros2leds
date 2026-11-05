"""led_matrix_effects on panels other than 8x8, and on strips with another colour scale.

Signals and lamps are drawn for any ``size=(w, h)``; sprites are enlarged and refuse a panel smaller than
themselves; ``levels`` rescales the colours. The defaults (8x8, 25 levels) are pinned byte for byte.
"""

import hashlib
from pathlib import Path

import numpy as np
import pytest
from conftest import Image

from led_matrix_compositor import led_matrix_compositor as comp
from led_matrix_effects import Canvas, Layout, Rainbow, SpriteTooSmall, Strip, fit, lamp_grid, lamps, signals

OWN_YAML = Path(__file__).resolve().parents[1] / 'docs' / 'examples' / 'own_matrices.yaml'

SIZES = [(8, 8), (16, 16), (32, 8), (5, 5)]
TIMES = [i / 25 for i in range(0, 100, 3)]
SPRITE_THEMES = ['new_year', 'halloween', 'valentine', 'defender_day', 'womens_day', 'cosmonautics', 'birthday']
PROCEDURAL_THEMES = sorted(set(lamps.THEMES) - set(SPRITE_THEMES))


def procedural(size, levels=25.0):
    """Every procedural picture at a few moments: (name, array)."""
    fw = lamps.Fireworks(5, size=size)
    for t in TIMES:
        yield 'beacon', signals.beacon(t, signals.RED, size=size, levels=levels)
        yield 'flasher_on', signals.flasher(signals.AMBER, True, size=size, levels=levels)
        yield 'flasher_off', signals.flasher(signals.AMBER, False, size=size, levels=levels)
        for name in PROCEDURAL_THEMES:
            for k, a in enumerate(lamps.THEMES[name](t, size=size, levels=levels)):
                yield f'{name}{k}', a
        for name, fn in (('rings', lamps.speaker_rings), ('eq', lamps.eq_bars), ('disco', lamps.disco)):
            for k, a in enumerate(fn(t, 118.0, size=size, levels=levels)):
                yield f'{name}{k}', a
        for k, a in enumerate(lamps.turn_signal(t, 'left', size=size, levels=levels)):
            yield f'turn{k}', a
        for k, a in enumerate(fw(t, levels=levels)):
            yield f'fireworks{k}', a


@pytest.mark.parametrize('size', SIZES, ids=lambda s: f'{s[0]}x{s[1]}')
def test_procedural_pictures_fill_any_panel(size):
    w, h = size
    lit = set()
    for name, a in procedural(size):
        assert a.shape == (h, w, 3), name
        assert a.min() >= 0.0 and a.max() <= 25.0 + 1e-9, name
        if a.any():
            lit.add(name.rstrip('01'))
    assert {'beacon', 'flasher_on', 'default', 'rings', 'eq', 'disco', 'turn', 'party'} <= lit


@pytest.mark.parametrize('size', SIZES, ids=lambda s: f'{s[0]}x{s[1]}')
def test_lens_stays_inside_the_panel_ellipse(size):
    dist, _ = lamp_grid(size)
    out = signals.beacon(0.3, signals.BLUE, size=size)
    assert not out[dist > 3.95].any() and out[dist <= 1.0].any()
    h, w = size[1], size[0]
    # the lens reaches the middle of every edge: a long panel is used along its whole length
    assert out[h // 2, 0].any() and out[h // 2, w - 1].any()


@pytest.mark.parametrize('size', [(8, 8), (16, 16), (32, 8)], ids=lambda s: f'{s[0]}x{s[1]}')
@pytest.mark.parametrize('name', SPRITE_THEMES)
def test_sprites_are_enlarged_not_redrawn(size, name):
    w, h = size
    for t in TIMES[::4]:
        small = lamps.THEMES[name](t)
        big = lamps.THEMES[name](t, size=size)
        for s_, b in zip(small, big):
            assert b.shape == (h, w, 3)
            k = min(w // 8, h // 8)
            y0, x0 = (h - 8 * k) // 2, (w - 8 * k) // 2
            assert np.array_equal(b[y0:y0 + 8 * k:k, x0:x0 + 8 * k:k], s_)       # nearest neighbour
            assert b.sum() == pytest.approx(s_.sum() * k * k)                       # nothing outside the sprite


@pytest.mark.parametrize('name', SPRITE_THEMES)
def test_sprite_on_a_smaller_panel_says_so(name):
    with pytest.raises(SpriteTooSmall, match='5x5'):
        lamps.THEMES[name](0.5, size=(5, 5))


def test_fit_rejects_a_panel_narrower_than_the_sprite():
    with pytest.raises(SpriteTooSmall):
        fit(np.ones((8, 8, 3)), size=(32, 5))


@pytest.mark.parametrize('size', SIZES, ids=lambda s: f'{s[0]}x{s[1]}')
def test_levels_rescale_every_picture(size):
    base = list(procedural(size))
    for levels in (1.0, 255.0):
        for (name, a), (_n, b) in zip(base, procedural(size, levels=levels)):
            assert np.allclose(b, a * levels / 25.0), (name, levels)


def test_levels_255_reach_the_wire_at_brightness_255():
    a = signals.flasher(signals.RED, True, levels=255.0)
    wire = Strip(381, brightness=255).to_wire(a)
    assert wire.max() == 255 and wire[..., 0].max() == 255


def test_fire_color_and_sprites_take_levels():
    assert np.allclose(signals.fire_color(0.9, levels=50.0), signals.fire_color(0.9) * 2.0)
    assert np.allclose(lamps.THEMES['valentine'](0.1, levels=50.0)[0], lamps.THEMES['valentine'](0.1)[0] * 2.0)


# ----------------------------------------------------------------- the README example
def test_own_matrices_example_goes_through_the_real_compositor(params):
    """docs/examples/own_matrices.yaml: the effects draw on it and the compositor takes every frame whole."""
    layout = Layout.from_compositor(OWN_YAML)
    assert layout.size == {'square': (16, 16), 'bar': (32, 8)} and layout.n_leds == 512
    canvas, strip = Canvas(layout), Strip(layout.n_leds)
    frame = Rainbow(canvas, level=25.0)(0.4)
    frame.square = lamps.THEMES['new_year'](0.4, size=layout.size['square'])[0]
    frame.bar = signals.beacon(0.4, signals.RED, size=layout.size['bar'])

    params['config_file'] = str(OWN_YAML)
    node = comp.LEDMatrixCompositor()
    for panel, levels in frame.panels().items():
        msg = Image()
        msg.header.frame_id = layout.group_of[panel]
        msg.width, msg.height = layout.size[panel]
        msg.data = strip.to_wire(levels).tobytes()
        node.image_callback(msg)
    node.timers[0].callback()
    sent = node.publishers[0].sent[-1]
    strip_bytes = bytes((v + 128) % 256 for v in sent.data)       # wire format: channel - 128
    assert len(strip_bytes) == 512 * 3
    lit = sum(1 for i in range(512) if any(strip_bytes[3 * i:3 * i + 3]))
    want = sum(int(strip.to_wire(a).any(axis=2).sum()) for a in frame.panels().values())
    assert lit == want > 100                                     # every lit LED of the frame reached the strip


# ----------------------------------------------------------------- defaults are pinned
def _digest(arrays):
    """Digest of the wire bytes at brightness 25 (raw floats differ in the last bit between platforms)."""
    strip = Strip(381)
    h = hashlib.sha256()
    for a in arrays:
        h.update(strip.to_wire(a).tobytes())
    return h.hexdigest()[:16]


def _defaults():
    """Every lamp picture at the defaults, as the rover draws them (same call signatures as before sizes existed)."""
    fw = lamps.Fireworks(5)
    for t in [i / 25 + 0.013 for i in range(200)]:
        for name in sorted(lamps.THEMES):
            yield from lamps.THEMES[name](t)
        yield from lamps.speaker_rings(t, 118.0)
        yield from lamps.eq_bars(t, 118.0)
        yield from lamps.disco(t, 118.0)
        yield from lamps.turn_signal(t, 'right')
        yield from fw(t)
        yield signals.beacon(t, signals.AMBER, rpm=80)
        yield signals.flasher(signals.BLUE, int(t * 10) % 2 == 0)
        yield signals.fire_color((t % 1.0))


def test_defaults_are_the_8x8_pictures_byte_for_byte():
    """Digest taken from ros2leds a9013f7 (before sizes and levels existed) and from this code: the same."""
    assert _digest(_defaults()) == PINNED_DEFAULTS


PINNED_DEFAULTS = 'e8112445d747cc3e'
