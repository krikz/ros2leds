"""Emergency and road signals: beacons and flashers for an 8x8 round lamp, flash patterns, a fire, an ECG trace.

These are the building blocks of police, fire truck, ambulance, road service and roadworks scenes; an application
puts them on its own panels (a lamp picture on a headlight, six segment colours on a light bar). Patterns are pure
functions of ``t`` (seconds), so two lamps driven by the same pattern stay in sync.
"""
from __future__ import annotations

import math

import numpy as np

from .color import col

_YY, _XX = np.mgrid[0:8, 0:8]
_D = np.hypot(_XX - 3.5, _YY - 3.5)
_A = np.arctan2(_YY - 3.5, _XX - 3.5)
#: Distance of every LED of an 8x8 lamp from its centre, and the round lens of the lamp.
LAMP_R = _D
DISC = _D <= 3.95

RED = col(25, 0, 0)
BLUE = col(0, 5, 25)
WHITE = col(23, 23, 25)
AMBER = col(25, 11, 0)
YELLOW = col(25, 17, 0)


def beacon(t, color, rpm=100, phase=0.0, width=5.0):
    """Rotating-mirror beacon on an 8x8 lamp: a bright beam sweeps around the lens."""
    ang = t * rpm / 60 * math.tau + phase
    d = np.angle(np.exp(1j * (_A - ang)))
    k = np.clip(np.cos(d), 0, 1) ** width
    out = np.zeros((8, 8, 3))
    lens = 0.1 + 0.9 * k
    for c in range(3):
        out[..., c] = np.where(DISC, color[c] * lens, 0)
    core = _D < 1.2
    out[core] = color * 0.55 + col(25, 25, 25) * 0.25
    return out


def flasher(color, on):
    """Strobe on an 8x8 lamp: the whole lens with a white-hot centre when ``on``, a faint rim when off."""
    out = np.zeros((8, 8, 3))
    if on:
        out[DISC] = color
        out[_D < 1.6] = color * 0.6 + col(25, 25, 25) * 0.4
    else:
        out[(_D > 3.0) & DISC] = color * 0.08
    return out


def police_pattern(t):
    """2.4 s cycle: double wig-wag x2, centre-out sweep, fast alternate.

    Returns ``(left_on, right_on, segments)``: two flashers and six light-bar segment colours (None = dark)."""
    p = t % 2.4
    off = [None] * 6
    if p < 1.2:
        step = int((p % 0.6) / 0.075)            # L . L . R . R .
        left = step in (0, 2)
        right = step in (4, 6)
        segs = [RED if left else None] * 3 + [BLUE if right else None] * 3
        return left, right, segs
    if p < 1.8:
        k = int((p - 1.2) / 0.1)                 # 0..5
        segs = list(off)
        reach = k % 3
        segs[2 - reach] = RED
        segs[3 + reach] = BLUE
        return True, True, segs
    k = int((p - 1.8) / 0.075) % 2
    segs = [RED, RED, RED, None, None, None] if k == 0 else [None, None, None, BLUE, BLUE, BLUE]
    return k == 0, k == 1, segs


def doom_fire(state, rng, w=25, h=7):
    """One step of the classic heat-spread fire on a ``h x w`` grid (lists); the bottom row is the fuel.

    Draw the rows above the fuel; with ``h`` two rows taller than the panel the flames start below its edge."""
    state[h - 1] = [rng.uniform(0.75, 1.0) if rng.random() < 0.85 else 0.3 for _ in range(w)]
    for y in range(h - 1):
        for x in range(w):
            src_x = min(w - 1, max(0, x + rng.choice((-1, 0, 0, 1))))
            state[y][x] = max(0.0, state[y + 1][src_x] - rng.uniform(0.08, 0.26))
    return state


def fire_color(v):
    """Heat 0..1 of :func:`doom_fire` -> colour in levels: dark, deep red, orange, yellow-white."""
    if v < 0.18:
        return col(0, 0, 0)
    if v < 0.4:
        return col(14, 0, 0) * ((v - 0.18) / 0.22 * 0.7 + 0.3)
    if v < 0.65:
        return col(25, 5 + (v - 0.4) / 0.25 * 9, 0)
    return col(25, 15 + (v - 0.65) / 0.35 * 9, (v - 0.65) / 0.35 * 10)


#: Cells of a medical cross on an 8x8 lamp.
CROSS = [(y, x) for y in range(8) for x in range(8) if (3 <= x <= 4 and 1 <= y <= 6) or (3 <= y <= 4 and 1 <= x <= 6)]
#: One heartbeat of an ECG trace, samples of -1..1 (P wave, QRS, T wave), for a line sweeping across a panel.
ECG = [0, 0, 0, 0, 0, 0.15, 0.3, 0.15, 0, 0, -0.25, 1.0, -0.6, 0, 0, 0, 0.25, 0.4, 0.3, 0.1, 0, 0, 0, 0, 0, 0, 0, 0]
#: A ``>`` chevron, 4x5, for a crawling road-service arrow.
CHEVRON = ["##..", ".##.", "..##", ".##.", "##.."]

#: Roadworks strobe wig-wag: one cycle is a double flash on the left, then a double flash on the right.
ROADWORKS_CYCLE_S = 1.2
ROADWORKS_FLASHES = ((0.0, 0.08), (0.15, 0.23))   # (on, off) inside each half-cycle, s


def roadworks_pattern(t):
    """Which side flashes at ``t``: ``(left_on, right_on)``, never both."""
    p = t % ROADWORKS_CYCLE_S
    half = ROADWORKS_CYCLE_S / 2
    right = p >= half
    q = p - half if right else p
    on = any(a <= q < b for a, b in ROADWORKS_FLASHES)
    return on and not right, on and right


__all__ = ['AMBER', 'BLUE', 'CHEVRON', 'CROSS', 'DISC', 'ECG', 'LAMP_R', 'RED', 'ROADWORKS_CYCLE_S',
           'ROADWORKS_FLASHES', 'WHITE', 'YELLOW', 'beacon', 'doom_fire', 'fire_color', 'flasher', 'police_pattern',
           'roadworks_pattern']
