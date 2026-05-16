from __future__ import annotations

from pathlib import Path

import numpy as np


DEFAULT_WORKSPACE_MIN = np.array([0.15, -0.45, -0.05], dtype=float)
DEFAULT_WORKSPACE_MAX = np.array([0.65, 0.45, 0.45], dtype=float)


def read_xyz_file(path: str | Path) -> np.ndarray:
    text = Path(path).read_text(encoding="utf-8").strip()
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError(f"Expected 3 xyz values in {path}, got {len(parts)}: {text!r}")
    xyz = np.array([float(value) for value in parts], dtype=float)
    validate_finite_xyz("target", xyz)
    return xyz


def validate_finite_xyz(name: str, xyz: np.ndarray) -> None:
    if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
        raise ValueError(f"{name} must be finite xyz, got {xyz.tolist()}")


def validate_workspace(name: str, xyz: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> None:
    validate_finite_xyz(name, xyz)
    if np.any(xyz < lower) or np.any(xyz > upper):
        raise ValueError(
            f"{name} {xyz.tolist()} outside workspace "
            f"min={lower.tolist()} max={upper.tolist()}"
        )


def parse_xyz(values: list[float] | tuple[float, float, float], name: str) -> np.ndarray:
    xyz = np.array(values, dtype=float)
    validate_finite_xyz(name, xyz)
    return xyz
