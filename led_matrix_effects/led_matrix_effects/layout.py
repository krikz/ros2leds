"""Panels, their places on a shared canvas, and the frame that holds one picture of all of them.

A :class:`Layout` answers three questions about the LED panels an application draws on:

* **What panels are there and how big?** ``size[panel] = (width, height)``. Usually read from the
  ``led_matrix_compositor`` YAML (:meth:`Layout.from_compositor`): every logical group is a panel, or the
  application names its own panels and maps each one to a group (``group_of``).
* **Where is each panel on the canvas?** ``at[panel] = (x, y)`` of its top-left LED in one coordinate system
  shared by all panels, so an effect can run across them (a wave that passes from one headlight through the mouth
  to the other). The canvas is a picture of how the panels look from outside, gaps between boards included; it is
  the application's choice, the compositor does not know it. Default: panels side by side, one LED apart.
* **How many LEDs does the strip have?** ``n_leds`` (for the current model, :mod:`led_matrix_effects.power`).

A :class:`Frame` is one picture: an ``(height, width, 3)`` float array of levels per panel, reachable as an
attribute (``frame.mouth``) or through :meth:`Frame.panels`.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple, Union

import numpy as np

Size = Tuple[int, int]
Point = Tuple[int, int]
_RESERVED = {'panels', 'layout'}


def read_groups(config: Mapping) -> Tuple[Dict[str, Size], int]:
    """Compositor config -> ``({group: (width, height)}, LEDs in the strip)``.

    ``config`` is the parsed YAML, either the whole file (``led_matrix_compositor: ros__parameters: ...``) or its
    ``ros__parameters``. The arithmetic is the compositor's own: a group is ``arrangement`` panels of the size of its
    first physical panel."""
    if 'led_matrix_compositor' in config:
        config = config['led_matrix_compositor']
    if 'ros__parameters' in config:
        config = config['ros__parameters']
    panels = config['physical_panels']
    sizes = {}
    for g in config['logical_groups']:
        cols, rows = g['arrangement']
        first = panels[g['physical_indices'][0]]
        sizes[g['name']] = (int(first['width']) * int(cols), int(first['height']) * int(rows))
    return sizes, sum(int(p['width']) * int(p['height']) for p in panels)


def load_yaml(path: Union[str, Path]) -> Mapping:
    """Parse a YAML file (PyYAML is imported here, not at package import: the effects need only numpy)."""
    import yaml

    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


class Layout:
    """Panels, their sizes, their compositor groups and their places on the canvas (see the module doc)."""

    def __init__(self, size: Mapping[str, Size], at: Optional[Mapping[str, Point]] = None,
                 group_of: Optional[Mapping[str, str]] = None, n_leds: Optional[int] = None) -> None:
        #: panel -> (width, height); the order of this mapping is the order of :meth:`Frame.panels`
        self.size: Dict[str, Size] = {p: (int(w), int(h)) for p, (w, h) in size.items()}
        for p in self.size:
            if not p.isidentifier() or p.startswith('_') or p in _RESERVED:
                raise ValueError(f'panel name {p!r} must be a plain identifier (it is a Frame attribute)')
        if at is None:
            at, x = {}, 0
            for p, (w, _h) in self.size.items():
                at[p] = (x, 0)
                x += w + 1
        if set(at) != set(self.size):
            raise ValueError(f'canvas places {sorted(at)} do not match the panels {sorted(self.size)}')
        #: panel -> (x, y) of its top-left LED on the canvas; the order of this mapping is the canvas order
        self.at: Dict[str, Point] = {p: (int(x), int(y)) for p, (x, y) in at.items()}
        #: panel -> logical group of the compositor (the ``frame_id`` its image is published with)
        self.group_of: Dict[str, str] = dict(group_of) if group_of is not None else {p: p for p in self.size}
        if set(self.group_of) != set(self.size):
            raise ValueError(f'group_of {sorted(self.group_of)} does not match the panels {sorted(self.size)}')
        self.n_leds: int = int(n_leds) if n_leds is not None else sum(w * h for w, h in self.size.values())
        #: panel -> numpy shape ``(rows, columns)`` of its array
        self.shape: Dict[str, Tuple[int, int]] = {p: (h, w) for p, (w, h) in self.size.items()}
        self.panels: Tuple[str, ...] = tuple(self.size)

    @classmethod
    def from_compositor(cls, config: Union[str, Path, Mapping], group_of: Optional[Mapping[str, str]] = None,
                        at: Optional[Mapping[str, Point]] = None) -> 'Layout':
        """Layout from a compositor config (a YAML path or the parsed mapping).

        ``group_of`` names the application's panels: ``{panel: group}``, in the order the frame lists them; a group
        missing from the config is an error. Without it every logical group is a panel under its own name."""
        if not isinstance(config, Mapping):
            config = load_yaml(config)
        sizes, n_leds = read_groups(config)
        if group_of is None:
            group_of = {g: g for g in sizes}
        missing = sorted(set(group_of.values()) - set(sizes))
        if missing:
            raise ValueError(f'compositor config has no logical group(s) {missing}')
        return cls({p: sizes[g] for p, g in group_of.items()}, at=at, group_of=group_of, n_leds=n_leds)

    @property
    def group_size(self) -> Dict[str, Size]:
        """Logical group -> (width, height), the size the compositor expects an image of that group to have."""
        return {g: self.size[p] for p, g in self.group_of.items()}


class Frame:
    """One picture of every panel of a layout: ``frame.<panel>`` is an ``(h, w, 3)`` float array of levels.

    Panels are plain attributes, so an effect may replace a panel's array (``frame.mouth = arr``) as well as
    draw into it."""

    def __init__(self, layout: Layout) -> None:
        self.layout = layout
        for p, shape in layout.shape.items():
            setattr(self, p, np.zeros(shape + (3,)))

    def panels(self) -> Dict[str, np.ndarray]:
        """Panel -> array, in the layout's panel order."""
        return {p: getattr(self, p) for p in self.layout.panels}


__all__ = ['Frame', 'Layout', 'load_yaml', 'read_groups']
