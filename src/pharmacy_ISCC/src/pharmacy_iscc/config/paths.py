# -*- coding: utf-8 -*-
"""Resolve resources in both catkin devel and install spaces."""
import os


def package_path(*parts):
    try:
        import rospkg
        root = rospkg.RosPack().get_path("pharmacy_iscc")
    except ImportError:
        root = _source_root()
    except rospkg.ResourceNotFound:
        root = _source_root()
    return os.path.join(root, *parts)


def _source_root():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    if not os.path.isfile(os.path.join(root, "package.xml")):
        raise RuntimeError("Source the pharmacy_ISCC catkin workspace before running nodes")
    return root
