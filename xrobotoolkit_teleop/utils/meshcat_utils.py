"""Configure meshcat / placo viewer for Docker."""

from __future__ import annotations

import os

_printed_viewer_url = False


def patch_placo_meshcat_host() -> None:
    """Print the Mac-accessible viewer URL once when running in Docker."""
    global _printed_viewer_url
    port = os.environ.get("MESHCAT_HOST_PORT")
    if not port:
        return

    import placo_utils.visualization as pv

    _orig = pv.get_viewer

    def get_viewer():
        global _printed_viewer_url
        viewer = _orig()
        if not _printed_viewer_url:
            print(f"Viewer URL (open on Mac host): http://localhost:{port}/static/")
            _printed_viewer_url = True
        return viewer

    pv.get_viewer = get_viewer
