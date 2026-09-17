from __future__ import annotations

from pathlib import Path
from typing import Optional


def package_share_dir(package_name: str = "person_reid_tracker") -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def default_model_path(filename: str) -> str:
    share = package_share_dir()
    if share is not None:
        candidate = share / "models" / filename
        if candidate.exists():
            return str(candidate)
    # Source-tree fallback for direct python testing before colcon install.
    src_root = Path(__file__).resolve().parents[1]
    candidate = src_root / "model_assets" / filename
    return str(candidate)


def resolve_path(value: str, default_filename: str) -> str:
    if value is None or str(value).strip() == "":
        return default_model_path(default_filename)
    p = Path(str(value)).expanduser()
    return str(p)
