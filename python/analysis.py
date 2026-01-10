#!/usr/bin/env python3
"""
Plot Pose2d samples from data.json (default) using matplotlib.

Axes match:

          +x
           |
           |
 +y  ------+------  -y
           |
           |
          -x

Mapping (field -> plot):
  plot_x = -field_y   (so +y goes left)
  plot_y =  field_x   (so +x goes up)

Usage:
  python plot_poses.py            # reads ./data.json
  python plot_poses.py other.json
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt


@dataclass(frozen=True)
class PoseSample:
    x: float
    y: float
    heading: float  # radians
    speed: float


DEFAULT_JSON = Path("data.json")


def load_samples(json_path: Path) -> List[PoseSample]:
    raw = json.loads(json_path.read_text(encoding="utf-8"))
    items = raw.get("data", [])
    if not items:
        raise ValueError("No samples found under key 'data'.")

    out: List[PoseSample] = []
    for i, it in enumerate(items):
        try:
            out.append(
                PoseSample(
                    x=float(it["posX"]),
                    y=float(it["posY"]),
                    heading=float(it["heading"]),
                    speed=float(it.get("speed", float("nan"))),
                )
            )
        except KeyError as e:
            raise ValueError(f"Missing key {e} in data[{i}]") from e
    return out


def field_to_plot_xy(x_field: float, y_field: float) -> Tuple[float, float]:
    return (-y_field, x_field)


def heading_to_plot_uv(heading_rad: float, length: float) -> Tuple[float, float]:
    dx_field = math.cos(heading_rad)
    dy_field = math.sin(heading_rad)
    dx_plot = -dy_field
    dy_plot = dx_field
    return (length * dx_plot, length * dy_plot)


def main() -> int:
    json_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_JSON
    if not json_path.exists():
        raise FileNotFoundError(f"Could not find JSON file: {json_path.resolve()}")

    samples = load_samples(json_path)

    # Defaults from your message
    start_pose = (50.0, -50.0, math.radians(-45.0))  # (x, y, heading)
    goal_xy = (50.0, -50.0)  # (x, y)

    # Convert samples to plot coords + heading vectors
    pxs: List[float] = []
    pys: List[float] = []
    us: List[float] = []
    vs: List[float] = []
    speeds = [s.speed for s in samples]

    arrow_len = 2.0
    for s in samples:
        px, py = field_to_plot_xy(s.x, s.y)
        u, v = heading_to_plot_uv(s.heading, arrow_len)
        pxs.append(px)
        pys.append(py)
        us.append(u)
        vs.append(v)

    fig, ax = plt.subplots(figsize=(8, 8))

    # Larger dots so speed coloring is easier to see
    point_size = 220  # increase/decrease to taste
    if all(math.isfinite(s) for s in speeds):
        sc = ax.scatter(pxs, pys, c=speeds, s=point_size)
        plt.colorbar(sc, ax=ax, label="speed")
    else:
        ax.scatter(pxs, pys, s=point_size)

    # Heading arrows at each point
    ax.quiver(pxs, pys, us, vs, angles="xy", scale_units="xy", scale=1)

    # Start pose marker + arrow + label
    sx, sy, sh = start_pose
    spx, spy = field_to_plot_xy(sx, sy)
    ax.scatter([spx], [spy], marker="o", s=260)
    su, sv = heading_to_plot_uv(sh, arrow_len * 1.4)
    ax.quiver([spx], [spy], [su], [sv], angles="xy", scale_units="xy", scale=1)
    ax.annotate(
        "Start position",
        xy=(spx, spy),
        xytext=(8, 10),
        textcoords="offset points",
        ha="left",
        va="bottom",
    )

    # Goal marker
    gx, gy = goal_xy
    gpx, gpy = field_to_plot_xy(gx, gy)
    ax.scatter([gpx], [gpy], marker="*", s=320)

    # Axes through origin (in plot coordinates)
    ax.axhline(0, linewidth=1)
    ax.axvline(0, linewidth=1)

    # Direction labels to match your diagram
    ax.text(0, 1.02, "+x", transform=ax.transAxes, ha="center", va="bottom")
    ax.text(0, -0.06, "-x", transform=ax.transAxes, ha="center", va="top")
    ax.text(-0.02, 0.5, "+y", transform=ax.transAxes, ha="right", va="center")
    ax.text(1.02, 0.5, "-y", transform=ax.transAxes, ha="left", va="center")

    ax.set_title(f"Pose samples (absolute) from {json_path.name}")
    ax.set_xlabel("y axis (left is +y, right is -y)")
    ax.set_ylabel("x axis (up is +x, down is -x)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
