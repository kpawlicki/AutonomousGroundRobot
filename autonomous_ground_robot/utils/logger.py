"""Centralised logging for the autonomous ground robot."""

import logging
import sys

_LOG_FORMAT = "%(asctime)s  %(levelname)-8s  [%(name)s]  %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

logging.basicConfig(
    level=logging.INFO,
    format=_LOG_FORMAT,
    datefmt=_DATE_FORMAT,
    stream=sys.stdout,
)


def get_logger(name: str) -> logging.Logger:
    """Return a named logger under the *autonomous_ground_robot* hierarchy."""
    return logging.getLogger(f"agr.{name}")
