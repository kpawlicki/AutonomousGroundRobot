"""Config loader for the autonomous ground robot."""

import os
from pathlib import Path
from typing import Any, Dict, Optional

_DEFAULT_CONFIG_PATH = Path(__file__).parent / "default_config.yaml"


def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load configuration from a YAML file.

    If *path* is ``None`` the bundled ``default_config.yaml`` is used.
    The user config is deep-merged on top of the defaults so that only
    the keys that are explicitly set need to be provided.
    """
    try:
        import yaml  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise ImportError("PyYAML is required: pip install pyyaml") from exc

    with open(_DEFAULT_CONFIG_PATH) as f:
        defaults: Dict[str, Any] = yaml.safe_load(f) or {}

    if path is None:
        return defaults

    with open(path) as f:
        user: Dict[str, Any] = yaml.safe_load(f) or {}

    return _deep_merge(defaults, user)


def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result
