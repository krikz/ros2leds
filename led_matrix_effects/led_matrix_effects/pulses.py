"""One-shot pulses over a finished frame: katana slash, glitch, boot-up, old-TV power-down.

A pulse is called as ``pulse(t, frame) -> frame`` with ``t`` in seconds since the pulse started; it may change the
frame in place. After :attr:`duration` it returns the frame untouched — except :class:`PowerDown`, which keeps the
panels dark (the picture has been switched off). Before 0 a pulse has not started: the frame is untouched, except
:class:`Boot`, whose panels are still dark (nothing has been switched on yet).

Colours are given in levels of 25; ``levels`` (the brightest level of your strip, ``Strip.max_level``) rescales them.
"""
from __future__ import annotations

import math
import random
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

from .canvas import Canvas
from .color import BLACK, clamp, col
from .geometry import FULL, scaled
from .layout import Frame


class Slash:
    """Katana: a thin bright slanted blade sweeps the canvas left to right, the picture behind it darkened a moment."""

    def __init__(self, canvas: Canvas, duration: float = 0.24, slant: float = 1.5, core: float = 2.5,
                 trail: float = 20.0, shade: float = 0.8, color: np.ndarray = col(20, 23, 25),
                 levels: float = FULL) -> None:
        #: ``slant`` columns per row; ``core`` columns of bright blade; ``trail`` columns of shadow behind it (keep it
        #: longer than the blade's step per frame, or a panel may be skipped); ``shade``: how dark the shadow starts
        self.canvas, self.duration, self.slant, self.core = canvas, duration, slant, core
        self.trail, self.shade, self.color = trail, shade, scaled(color, levels)
        self._span = canvas.width + slant * canvas.height + trail

    def __call__(self, t: float, f: Frame) -> Frame:
        if not 0.0 <= t < self.duration:
            return f
        head = t / self.duration * self._span
        h = self.canvas.height
        for p in self.canvas.panels:
            X, Y = self.canvas.xy[p]
            d = (head - (X + self.slant * (h - 1 - Y)))[..., None]
            shade = np.where((d >= self.core) & (d < self.trail),
                             1.0 - self.shade * (1.0 - (d - self.core) / (self.trail - self.core)), 1.0)
            setattr(f, p, np.where((d >= 0.0) & (d < self.core), self.color, getattr(f, p) * shade))
        return f


GLITCH_DEAD = (BLACK, col(25, 0, 25), col(0, 25, 25), col(16, 16, 16))


class Glitch:
    """Glitch: panels and rows jerk sideways, red and blue tear apart, a few dead pixels; a new tear every frame.

    Deterministic: frame ``k`` (``int(t * fps)``) tears with ``random.Random(seed + k)``; ``drop`` is the share of
    frames shown black (a "dropped" frame)."""

    def __init__(self, canvas: Canvas, duration: float = 0.32, fps: float = 25, seed: int = 0, drop: float = 0.15,
                 dead: Sequence[np.ndarray] = GLITCH_DEAD, levels: float = FULL) -> None:
        self.canvas, self.duration, self.fps, self.seed, self.drop = canvas, duration, fps, seed, drop
        self.dead = dead if levels == FULL else tuple(scaled(c, levels) for c in dead)

    def __call__(self, t: float, f: Frame) -> Frame:
        if not 0.0 <= t < self.duration:
            return f
        rng = random.Random(self.seed + int(t * self.fps))
        if rng.random() < self.drop:
            return self.canvas.new_frame()
        for p in self.canvas.panels:
            a = getattr(f, p)
            h, w = a.shape[:2]
            if rng.random() < 0.35:
                a = np.roll(a, rng.choice((-2, -1, 1, 2)), axis=1)
            out = a.copy()
            for _ in range(rng.randint(1, 2)):
                y = rng.randrange(h)
                out[y] = np.roll(a[y], rng.choice((-3, -2, 2, 3)), axis=0)
            out[..., 0] = np.roll(out[..., 0], 1, axis=1)
            out[..., 2] = np.roll(out[..., 2], -1, axis=1)
            for _ in range(rng.randint(1, 3)):
                out[rng.randrange(h), rng.randrange(w)] = rng.choice(self.dead)
            setattr(f, p, out)
        return f


class Boot:
    """Switching on: panel after panel (in ``order``), row by row under a scan line; dark panels wait their turn."""

    def __init__(self, canvas: Canvas, order: Optional[Iterable[str]] = None, step_s: float = 0.22,
                 row_s: float = 0.03, line: np.ndarray = col(10, 12, 14), levels: float = FULL) -> None:
        self.canvas, self.step_s, self.row_s, self.line = canvas, step_s, row_s, scaled(line, levels)
        self.order: Tuple[str, ...] = tuple(order) if order is not None else canvas.panels
        if sorted(self.order) != sorted(canvas.panels):
            raise ValueError(f'boot order {self.order} is not a permutation of the panels {canvas.panels}')
        rows = max(h for h, _w in canvas.shapes.values())
        #: the last panel starts after ``step_s * (n - 1)`` and needs the rows of the tallest panel
        self.duration = step_s * (len(self.order) - 1) + rows * row_s

    def __call__(self, t: float, f: Frame) -> Frame:
        if t >= self.duration:
            return f
        for i, p in enumerate(self.order):
            a = getattr(f, p).copy()
            rows = (t - i * self.step_s) / self.row_s
            n = int(clamp(math.floor(rows), 0, a.shape[0]))
            a[n:] = 0.0
            if 0.0 <= rows < a.shape[0]:
                a[n] = self.line
            setattr(f, p, a)
        return f


class PowerDown:
    """Old TV switched off: the picture squeezes into a white line, the line into a dot, then dark (and stays dark).

    ``centre`` is the canvas point the picture collapses to; default — the middle of the canvas."""

    def __init__(self, canvas: Canvas, centre: Optional[Tuple[float, float]] = None, squeeze_s: float = 0.3,
                 duration: float = 0.6, line: np.ndarray = col(18, 18, 18), levels: float = FULL) -> None:
        self.canvas, self.squeeze_s, self.duration, self.line = canvas, squeeze_s, duration, scaled(line, levels)
        cx, cy = centre if centre is not None else ((canvas.width - 1) / 2.0, (canvas.height - 1) / 2.0)
        self.cx, self.cy = cx, cy

    def __call__(self, t: float, f: Frame) -> Frame:
        if t >= self.duration:
            return self.canvas.new_frame()
        p1 = clamp(t / self.squeeze_s, 0.0, 1.0)
        p2 = clamp((t - self.squeeze_s) / (self.duration - self.squeeze_s), 0.0, 1.0)
        half_h = max((1.0 - p1) * self.canvas.height / 2.0, 0.5)
        half_w = max((1.0 - p2) * self.canvas.width, 0.5)
        for p in self.canvas.panels:
            X, Y = self.canvas.xy[p]
            band = (np.abs(Y - self.cy) <= half_h) & (np.abs(X - self.cx) <= half_w)
            line = (np.abs(Y - self.cy) <= 0.5)[..., None] * self.line * p1
            setattr(f, p, np.where(band[..., None], np.maximum(getattr(f, p), line), 0.0))
        return f


__all__ = ['Boot', 'GLITCH_DEAD', 'Glitch', 'PowerDown', 'Slash']
