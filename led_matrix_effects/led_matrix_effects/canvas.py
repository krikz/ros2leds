"""The canvas: every LED of a layout with its (x, y) on the shared canvas, as flat numpy rows.

An effect that crosses panels is a few numpy operations over all LEDs at once (:attr:`Canvas.X`, :attr:`Canvas.Y`),
not the same operations repeated over every small panel — numpy's per-call cost dominates on tiny arrays.
:meth:`Canvas.flat` and :meth:`Canvas.split` convert between a :class:`~led_matrix_effects.layout.Frame` and the
flat ``(N, 3)`` row. Everything here is built once per layout.
"""
from __future__ import annotations

from typing import Callable, Dict, Optional, Tuple

import numpy as np

from .layout import Frame, Layout


class Canvas:
    """Canvas geometry of a :class:`Layout`.

    ``frame`` makes an empty frame of the layout; pass the application's own ``Frame`` subclass so that effects
    return frames of that class."""

    def __init__(self, layout: Layout, frame: Optional[Callable[[], Frame]] = None) -> None:
        self.layout = layout
        self.new_frame: Callable[[], Frame] = frame if frame is not None else (lambda: Frame(layout))
        #: panels in canvas order (the order of ``layout.at``)
        self.panels: Tuple[str, ...] = tuple(layout.at)
        self.shapes: Dict[str, Tuple[int, int]] = {p: layout.shape[p] for p in self.panels}
        #: panel -> canvas coordinates (X, Y) of its LEDs, arrays of the panel's shape
        self.xy: Dict[str, Tuple[np.ndarray, np.ndarray]] = {p: self._grid(p) for p in self.panels}
        self.width = int(max(self.xy[p][0].max() for p in self.panels)) + 1
        self.height = int(max(self.xy[p][1].max() for p in self.panels)) + 1
        self.spans: Dict[str, Tuple[int, int]] = {}
        n = 0
        for p in self.panels:
            h, w = self.shapes[p]
            self.spans[p] = (n, n + h * w)
            n = self.spans[p][1]
        #: canvas X and Y of every LED, panel after panel in canvas order
        self.X = np.concatenate([self.xy[p][0].ravel() for p in self.panels])
        self.Y = np.concatenate([self.xy[p][1].ravel() for p in self.panels])
        self._nb = self._neighbours()

    def _grid(self, panel: str) -> Tuple[np.ndarray, np.ndarray]:
        h, w = self.shapes[panel]
        x0, y0 = self.layout.at[panel]
        yy, xx = np.mgrid[0:h, 0:w]
        return (xx + x0).astype(float), (yy + y0).astype(float)

    def _neighbours(self) -> np.ndarray:
        """(4, N) flat index of each LED's up/down/left/right neighbour on its own panel; N (dark) at an edge."""
        n = len(self.X)
        nb = np.full((4, n), n)
        for p in self.panels:
            h, w = self.shapes[p]
            idx = np.arange(*self.spans[p]).reshape(h, w)
            nb[0, idx[1:].ravel()] = idx[:-1].ravel()
            nb[1, idx[:-1].ravel()] = idx[1:].ravel()
            nb[2, idx[:, 1:].ravel()] = idx[:, :-1].ravel()
            nb[3, idx[:, :-1].ravel()] = idx[:, 1:].ravel()
        return nb

    def flat(self, f: Frame) -> np.ndarray:
        """Frame -> ``(N, 3)`` levels, panel after panel in canvas order."""
        return np.concatenate([getattr(f, p).reshape(-1, 3) for p in self.panels])

    def split(self, flat: np.ndarray, f: Optional[Frame] = None) -> Frame:
        """``(N, 3)`` levels -> frame (``f`` is filled and returned; a new frame if None). Panels are views."""
        f = self.new_frame() if f is None else f
        for p in self.panels:
            a, b = self.spans[p]
            setattr(f, p, flat[a:b].reshape(self.shapes[p] + (3,)))
        return f

    def paint(self, fn: Callable[[np.ndarray, np.ndarray], np.ndarray]) -> Frame:
        """New frame from ``fn(X, Y) -> (N, 3)`` levels over the whole canvas."""
        return self.split(fn(self.X, self.Y))

    def near(self, lit: np.ndarray) -> np.ndarray:
        """Flat bool ``lit`` -> lit or next to a lit LED (4-neighbours on the same panel)."""
        padded = np.append(lit, False)
        return lit | padded[self._nb].any(axis=0)


__all__ = ['Canvas']
