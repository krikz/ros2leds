"""Preview pictures of frames: LEDs drawn as dots, quantised exactly like the driver shows them.

Needs Pillow (imported on first use, not at package import). For docs, review and offline checks::

    img = render(canvas, frame)                  # PIL.Image of the whole canvas
    save_gif([render(canvas, fx(t)) for t in ts], 'out.gif', fps=25)
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .canvas import Canvas
from .power import Strip

OFF_LED = (26, 26, 30)
BACKGROUND = (8, 8, 10)


def shown(levels: np.ndarray, strip: Strip) -> np.ndarray:
    """Levels -> what the strip shows, ``0..brightness`` per channel (wire bytes through the driver's rounding)."""
    return strip.to_wire(levels).astype(int) * strip.brightness // 255


def draw_panel(draw, x0: int, y0: int, levels: np.ndarray, strip: Strip, pitch: int = 7) -> None:
    """Draw one panel's ``(h, w, 3)`` levels at pixel ``(x0, y0)`` of a ``PIL.ImageDraw``; dark LEDs as dim dots."""
    q = shown(levels, strip)
    b = strip.brightness
    for y in range(q.shape[0]):
        for x in range(q.shape[1]):
            cx, cy = x0 + x * pitch, y0 + y * pitch
            if q[y, x].max() == 0:
                draw.ellipse([cx + 2, cy + 2, cx + pitch - 3, cy + pitch - 3], fill=OFF_LED)
            else:
                c = tuple(int(255 * (v / b) ** (1 / 2.2)) for v in q[y, x])
                draw.ellipse([cx + 1, cy + 1, cx + pitch - 2, cy + pitch - 2], fill=c)


def render(canvas: Canvas, frame, strip: Optional[Strip] = None, pitch: int = 7, margin: int = 1):
    """The whole canvas as a ``PIL.Image``: every panel at its canvas place, ``margin`` LEDs of border."""
    from PIL import Image, ImageDraw

    strip = strip if strip is not None else Strip(canvas.layout.n_leds)
    w, h = (canvas.width + 2 * margin) * pitch, (canvas.height + 2 * margin) * pitch
    im = Image.new('RGB', (w, h), BACKGROUND)
    d = ImageDraw.Draw(im)
    for p in canvas.panels:
        x, y = canvas.layout.at[p]
        draw_panel(d, (x + margin) * pitch, (y + margin) * pitch, getattr(frame, p), strip, pitch)
    return im


def save_gif(images: Sequence, path, fps: float = 25, colors: int = 64) -> None:
    """Animated GIF of ``images`` (PIL), looping, ``fps`` frames per second."""
    frames = [im.convert('P', palette=1, colors=colors) for im in images]   # 1 = Image.ADAPTIVE
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=int(round(1000 / fps)), loop=0,
                   optimize=False, disposal=1)


__all__ = ['draw_panel', 'render', 'save_gif', 'shown']
