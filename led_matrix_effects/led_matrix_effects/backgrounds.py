"""Backgrounds over the whole canvas: rainbow wave, fire, falling "code" — and :func:`under` to put a picture on top.

A background is called with ``t`` (seconds) and returns a new frame. ``level`` is its brightest channel in levels:
keep it well below the foreground (the default 6 of 25 leaves eyes or text at least twice as bright) — it lights
every LED of the strip, so it also sets most of the current a frame draws. On a strip with more levels (driver
brightness above 25, ``Strip.max_level``) raise ``level`` in proportion.
"""
from __future__ import annotations

import numpy as np

from .canvas import Canvas
from .color import col, hue
from .layout import Frame

BG_LEVEL = 6.0


class Rainbow:
    """A rainbow wave passing slantwise through the whole canvas (not every LED recoloured at once)."""

    def __init__(self, canvas: Canvas, period_s: float = 4.0, wave_px: float = 26.0, slant: float = 0.8,
                 level: float = BG_LEVEL) -> None:
        self.canvas, self.period_s, self.wave_px, self.slant, self.level = canvas, period_s, wave_px, slant, level

    def __call__(self, t: float) -> Frame:
        shift = t / self.period_s
        return self.canvas.paint(lambda X, Y: hue((X + self.slant * Y) / self.wave_px - shift) * self.level)


class Fire:
    """Tongues of flame from the bottom of the canvas: red at the root, a yellow core, every column sways."""

    def __init__(self, canvas: Canvas, base: float = 4.0, swing: float = 2.6, level: float = BG_LEVEL) -> None:
        self.canvas, self.base, self.swing, self.level = canvas, base, swing, level

    def __call__(self, t: float) -> Frame:
        h = self.canvas.height

        def fn(X, Y):
            sway = (np.sin(X * 0.9 + t * 7.0) + np.sin(X * 0.37 - t * 4.3) + 0.6 * np.sin(X * 1.7 + t * 11.0)) / 2.6
            height = self.base + self.swing * sway
            heat = np.clip(1.0 - (h - 1 - Y) / np.maximum(height, 0.5), 0.0, 1.0)
            out = np.zeros(X.shape + (3,))
            out[..., 0] = np.clip(heat * 1.6, 0.0, 1.0)
            out[..., 1] = np.clip(heat * 1.6 - 0.7, 0.0, 1.0) * 0.7
            return out * self.level
        return self.canvas.paint(fn)


MATRIX_GREEN = col(0, 1.0, 0.15)
MATRIX_HEAD = col(0.45, 1.0, 0.45)


class MatrixRain:
    """Green drops of "code" falling down every canvas column, each column at its own speed; the head is paler.

    Speeds and phases are drawn once from ``numpy.random.RandomState(seed)``: the same seed, the same rain."""

    def __init__(self, canvas: Canvas, seed: int = 0, trail: float = 4.0, gap: float = 9.0,
                 speed: tuple = (5.0, 11.0), level: float = BG_LEVEL) -> None:
        self.canvas, self.trail, self.level = canvas, trail, level
        #: rows from one drop of a column to the next
        self.cycle = canvas.height + gap
        rng = np.random.RandomState(seed)
        speeds = rng.uniform(speed[0], speed[1], canvas.width)       # rows per second, per canvas column
        phases = rng.uniform(0.0, self.cycle, canvas.width)
        column = canvas.X.astype(int)
        self._speed, self._phase = speeds[column], phases[column]

    def __call__(self, t: float) -> Frame:
        d = (t * self._speed + self._phase) % self.cycle - self.canvas.Y
        k = np.where((d >= 0.0) & (d < self.trail), 1.0 - d / self.trail, 0.0)[..., None]
        head = ((d >= 0.0) & (d < 1.0))[..., None]
        return self.canvas.split(k * self.level * np.where(head, MATRIX_HEAD, MATRIX_GREEN))


def under(canvas: Canvas, f: Frame, bg: Frame, level: float = BG_LEVEL, dim: float = 0.25, lit_level: float = 3.0,
          through: float = 0.85) -> Frame:
    """Background ``bg`` under the picture ``f`` (``f`` is filled and returned).

    A pixel of ``f`` from ``lit_level`` up keeps its brightness (its brightest channel) and takes the background's
    hue by ``through`` of the background's strength there (``level`` is full strength): a wave passes through the
    picture too, and the picture stays the brightest. Elsewhere the background shows, dimmed to ``dim`` next to a
    lit pixel (4-neighbours on the same panel) — a dark outline that keeps the picture readable."""
    a, b = canvas.flat(f), canvas.flat(bg)
    lv = a.max(axis=1, keepdims=True)
    lit = lv[..., 0] >= lit_level
    bl = b.max(axis=1, keepdims=True)
    w = np.clip(bl / level, 0.0, 1.0) * through
    feat = a / np.maximum(lv, 1e-6) * (1.0 - w) + b / np.maximum(bl, 1e-6) * w
    feat *= lv / np.maximum(feat.max(axis=1, keepdims=True), 1e-6)
    dimmed = np.where(canvas.near(lit), dim, 1.0)[..., None]
    return canvas.split(np.where(lit[..., None], feat, b * dimmed), f)


__all__ = ['BG_LEVEL', 'Fire', 'MatrixRain', 'Rainbow', 'under']
