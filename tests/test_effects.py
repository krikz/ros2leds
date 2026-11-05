"""led_matrix_effects: layout from the compositor YAML, canvas, backgrounds, pulses, signals, lamps, strip model.

Pure numpy, no ROS: the rover layout below is the packaged compositor config with the canvas of the rover face
(two headlights and the mouth in front, two tail lights behind).
"""

import hashlib
from pathlib import Path

import numpy as np
import pytest

from led_matrix_driver import frame_math
from led_matrix_effects import (
    Boot,
    Canvas,
    Fire,
    Frame,
    Glitch,
    Layout,
    MatrixRain,
    PowerDown,
    Rainbow,
    Slash,
    Strip,
    lamps,
    read_groups,
    signals,
    under,
)
from led_matrix_effects.layout import load_yaml

ROOT = Path(__file__).resolve().parents[1]
COMPOSITOR_YAML = ROOT / 'led_matrix_compositor' / 'config' / 'led_matrix_compositor.yaml'
DRIVER_YAML = ROOT / 'led_matrix_driver' / 'config' / 'led_matrix_driver.yaml'

GROUP_OF = {'eye_l': 'wheel_front_right', 'eye_r': 'wheel_front_left', 'mouth': 'main_display',
            'rear_l': 'wheel_rear_left', 'rear_r': 'wheel_rear_right'}
AT = {'eye_l': (0, 0), 'mouth': (12, 2), 'eye_r': (41, 0), 'rear_l': (61, 0), 'rear_r': (73, 0)}


@pytest.fixture(scope='module')
def rover():
    return Layout.from_compositor(COMPOSITOR_YAML, group_of=GROUP_OF, at=AT)


@pytest.fixture(scope='module')
def canvas(rover):
    return Canvas(rover)


def lit_frame(canvas, level=10.0):
    f = canvas.new_frame()
    for p in canvas.panels:
        a = getattr(f, p)
        a[::2, ::2] = (level, level / 2, 0.0)
    return f


# ----------------------------------------------------------------- layout
def test_groups_come_from_the_packaged_compositor_config():
    sizes, n = read_groups(load_yaml(COMPOSITOR_YAML))
    assert sizes == {'main_display': (25, 5), 'wheel_front_left': (8, 8), 'wheel_front_right': (8, 8),
                     'wheel_rear_left': (8, 8), 'wheel_rear_right': (8, 8)}
    assert n == 381


def test_without_names_every_group_is_a_panel_side_by_side():
    lay = Layout.from_compositor(COMPOSITOR_YAML)
    assert lay.group_of == {g: g for g in lay.size}
    xs = [lay.at[p][0] for p in lay.panels]
    assert xs == sorted(xs) and all(lay.at[p][1] == 0 for p in lay.panels)
    assert lay.group_size == lay.size


def test_application_panels_map_to_groups(rover):
    assert rover.panels == tuple(GROUP_OF)
    assert rover.size['mouth'] == (25, 5) and rover.shape['mouth'] == (5, 25)
    assert rover.group_size['wheel_front_left'] == (8, 8)
    assert rover.n_leds == 381


def test_a_group_missing_from_the_config_is_an_error():
    with pytest.raises(ValueError, match='nope'):
        Layout.from_compositor(COMPOSITOR_YAML, group_of={'mouth': 'nope'})


@pytest.mark.parametrize('name', ['panels', 'layout', '_x', 'not a name'])
def test_panel_names_must_be_plain_attributes(name):
    with pytest.raises(ValueError):
        Layout({name: (2, 2)})


def test_canvas_places_must_match_the_panels():
    with pytest.raises(ValueError):
        Layout({'a': (2, 2), 'b': (2, 2)}, at={'a': (0, 0)})


def test_frame_has_an_array_per_panel_in_panel_order(rover):
    f = Frame(rover)
    assert list(f.panels()) == list(GROUP_OF)
    assert f.mouth.shape == (5, 25, 3) and f.eye_l.shape == (8, 8, 3) and not f.mouth.any()


# ----------------------------------------------------------------- canvas
def test_canvas_geometry(canvas):
    assert (canvas.width, canvas.height) == (81, 8)
    assert canvas.panels == tuple(AT)
    assert len(canvas.X) == len(canvas.Y) == 381
    X, Y = canvas.xy['mouth']
    assert (X[0, 0], Y[0, 0], X[-1, -1], Y[-1, -1]) == (12, 2, 36, 6)


def test_flat_and_split_round_trip(canvas):
    f = lit_frame(canvas)
    g = canvas.split(canvas.flat(f))
    assert all(np.array_equal(getattr(f, p), getattr(g, p)) for p in canvas.panels)


def test_near_stays_on_its_own_panel(canvas):
    lit = np.zeros(len(canvas.X), bool)
    a, _b = canvas.spans['eye_l']
    lit[a + 7] = True                     # top-right LED of eye_l: its right neighbour would be the mouth
    near = canvas.near(lit)
    assert near.sum() == 3                # itself, left and down; nothing on another panel
    assert near[a + 6] and near[a + 15]


# ----------------------------------------------------------------- backgrounds
@pytest.mark.parametrize('make', [lambda c: Rainbow(c), lambda c: Fire(c), lambda c: MatrixRain(c, seed=1)])
def test_backgrounds_stay_under_their_level(canvas, make):
    bg = make(canvas)
    for i in range(50):
        flat = canvas.flat(bg(i / 25))
        assert flat.min() >= 0.0 and flat.max() <= bg.level + 1e-9


def test_rainbow_is_a_wave_that_moves(canvas):
    a, b = canvas.flat(Rainbow(canvas)(0.0)), canvas.flat(Rainbow(canvas)(0.5))
    assert not np.allclose(a, b)
    assert len({tuple(np.round(c, 3)) for c in a}) > 20          # many hues at once, not one colour


def test_fire_burns_from_the_bottom(canvas):
    flat = np.mean([canvas.flat(Fire(canvas)(i / 25)) for i in range(25)], axis=0).sum(axis=1)
    bottom, top = flat[canvas.Y == canvas.height - 1].mean(), flat[canvas.Y == 0].mean()
    assert bottom > 3 * top


def test_matrix_rain_is_set_by_its_seed(canvas):
    same = [canvas.flat(MatrixRain(canvas, seed=7)(1.3)) for _ in range(2)]
    other = canvas.flat(MatrixRain(canvas, seed=8)(1.3))
    assert np.array_equal(*same) and not np.array_equal(same[0], other)


def test_under_keeps_the_picture_and_outlines_it(canvas):
    f = lit_frame(canvas, level=20.0)
    lv_before = canvas.flat(f).max(axis=1)
    bg = Rainbow(canvas)(0.3)
    out = canvas.flat(under(canvas, f, bg))
    lit = lv_before >= 3.0
    assert np.allclose(out[lit].max(axis=1), lv_before[lit])     # lit pixels keep their brightness
    b = canvas.flat(bg)
    dark = ~lit & ~canvas.near(lit)
    assert np.allclose(out[dark], b[dark])                       # far from the picture: the background as is


# ----------------------------------------------------------------- pulses
def test_pulses_leave_the_frame_alone_outside_their_time(canvas):
    for pulse in (Slash(canvas), Glitch(canvas), Boot(canvas)):
        for t in (-0.1, pulse.duration + 0.01):
            f = lit_frame(canvas)
            ref = canvas.flat(f).copy()
            if isinstance(pulse, Boot) and t < 0:
                ref[:] = 0.0                      # before the boot every panel is still dark
            assert np.array_equal(canvas.flat(pulse(t, f)), ref), (type(pulse).__name__, t)


def test_slash_crosses_every_panel(canvas):
    s = Slash(canvas)
    hit = set()
    for i in range(int(s.duration * 25) + 1):
        f = s(i / 25, canvas.new_frame())
        hit |= {p for p in canvas.panels if getattr(f, p).any()}
    assert hit == set(canvas.panels)


def test_glitch_is_deterministic_per_seed(canvas):
    a = [canvas.flat(Glitch(canvas, seed=5)(i / 25, lit_frame(canvas))) for i in range(8)]
    b = [canvas.flat(Glitch(canvas, seed=5)(i / 25, lit_frame(canvas))) for i in range(8)]
    assert all(np.array_equal(x, y) for x, y in zip(a, b))


def test_boot_lights_panels_in_order(canvas):
    order = ('rear_r', 'eye_l', 'mouth', 'eye_r', 'rear_l')
    boot = Boot(canvas, order=order)
    assert boot.duration == pytest.approx(0.22 * 4 + 8 * 0.03)
    full = lit_frame(canvas)
    first = {}
    for i in range(int(boot.duration * 100) + 1):
        f = boot(i / 100, lit_frame(canvas))
        for p in canvas.panels:
            if getattr(f, p).any():
                first.setdefault(p, i)
    assert sorted(first, key=first.get) == list(order)
    done = boot(boot.duration, lit_frame(canvas))
    assert np.array_equal(canvas.flat(done), canvas.flat(full))


def test_boot_order_must_name_every_panel(canvas):
    with pytest.raises(ValueError):
        Boot(canvas, order=('mouth',))


def test_power_down_ends_dark(canvas):
    pd = PowerDown(canvas)
    mid = canvas.flat(pd(0.25, lit_frame(canvas)))
    assert mid.any() and (canvas.Y[mid.any(axis=1)] - pd.cy).__abs__().max() <= 1.0
    assert not canvas.flat(pd(pd.duration, lit_frame(canvas))).any()


# ----------------------------------------------------------------- signals and lamps
def test_roadworks_never_flashes_both_sides():
    steps = [signals.roadworks_pattern(i / 100) for i in range(240)]
    assert not any(left and right for left, right in steps)
    assert any(left for left, _ in steps) and any(right for _, right in steps)


def test_police_pattern_drives_six_segments():
    for i in range(240):
        left, right, segs = signals.police_pattern(i / 100)
        assert len(segs) == 6 and isinstance(left, bool) and isinstance(right, bool)


def test_beacon_and_flasher_stay_on_the_lens():
    for out in (signals.beacon(0.3, signals.RED), signals.flasher(signals.AMBER, True)):
        assert out.shape == (8, 8, 3) and not out[~signals.DISC].any() and out.max() <= 25.0


@pytest.mark.parametrize('name', sorted(lamps.THEMES))
def test_lamp_themes_are_two_8x8_pictures_in_levels(name):
    fn = lamps.THEMES[name]
    for i in range(0, 250, 5):
        left, right = fn(i / 25)
        assert left.shape == right.shape == (8, 8, 3)
        assert left.min() >= 0.0 and max(left.max(), right.max()) <= 25.0 + 1e-9, (name, i)


# ----------------------------------------------------------------- strip model
@pytest.mark.parametrize('brightness', [25, 64, 255])
def test_every_level_survives_the_driver(brightness):
    strip = Strip(381, brightness=brightness)
    levels = np.arange(brightness + 1, dtype=float)
    wire = strip.to_wire(levels)
    shown = frame_math.apply_brightness(bytes(wire), brightness)
    assert list(shown) == list(levels.astype(int))


def test_current_model_matches_the_driver(canvas):
    strip = Strip.from_driver(DRIVER_YAML)
    assert (strip.num_leds, strip.brightness, strip.max_level) == (381, 25, 25.0)
    f = Rainbow(canvas, level=25.0)(0.7)
    wire = b''.join(strip.to_wire(a).tobytes() for a in f.panels().values())
    driver = frame_math.estimate_current_ma(frame_math.apply_brightness(wire, 25), 381, 20.0, 1.0)
    assert strip.frame_ma(f) == pytest.approx(driver)
    assert strip.frame_ma(canvas.new_frame()) == strip.idle_ma == 381.0


# ----------------------------------------------------------------- pinned output
def _digest(frames):
    """Digest of the wire bytes: what the strip shows. Raw floats are not pinned: ``np.sin`` differs in the last bit
    between platforms (Windows and Linux CI disagreed on the fire), the rounding to levels hides that."""
    strip = Strip(381)
    h = hashlib.sha256()
    for f in frames:
        for a in f.panels().values():
            h.update(strip.to_wire(a).tobytes())
    return h.hexdigest()[:16]


def test_effects_on_the_rover_are_pinned(canvas):
    """Byte-exact wire output of the effects on the rover canvas: a change here changes the robot's face.

    rob_box_project pins the same pictures through its own scenes; update both on purpose, never one by accident."""
    got = {
        'rainbow': _digest(Rainbow(canvas)(i / 25) for i in range(50)),
        'fire': _digest(Fire(canvas)(i / 25) for i in range(50)),
        'matrix': _digest(MatrixRain(canvas, seed=3780)(i / 25) for i in range(50)),
        'slash': _digest(Slash(canvas)(i / 100, lit_frame(canvas)) for i in range(25)),
        'glitch': _digest(Glitch(canvas, seed=3780)(i / 25, lit_frame(canvas)) for i in range(9)),
        'power_down': _digest(PowerDown(canvas)(i / 50, lit_frame(canvas)) for i in range(31)),
    }
    assert got == PINNED


PINNED = {
    'rainbow': 'ea062bd83909c3aa',
    'fire': '9a76aa8b7f3138c2',
    'matrix': '8d46ce50b6076abe',
    'slash': '840a20d5212b6a97',
    'glitch': 'aa83ed6ecdc51dd4',
    'power_down': '5309f945eb9acd03',
}


def test_preview_draws_the_canvas(canvas):
    pytest.importorskip('PIL')
    from led_matrix_effects.preview import render

    img = render(canvas, Rainbow(canvas)(0.0), pitch=5, margin=1)
    assert img.size == ((canvas.width + 2) * 5, (canvas.height + 2) * 5)
    assert len(img.getcolors(1 << 16)) > 10
