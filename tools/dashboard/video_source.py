"""Discover VTD render windows and build read-only ffmpeg inputs."""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple
from urllib.parse import urlsplit, urlunsplit

import yaml


MAINRS_TITLES = ("mainRS", "cam1", "igVisualBroadcast")
BIRDRS_TITLES = ("birdRS",)
# Kept for callers which imported the former single-window preference list.
DEFAULT_WINDOW_TITLES = MAINRS_TITLES + BIRDRS_TITLES

_WINDOW_LINE_RE = re.compile(
    r'(?P<window_id>0x[0-9a-fA-F]+)\s+"(?P<title>[^"]*)":\s+\("(?P<class1>[^"]*)"\s+"(?P<class2>[^"]*)"\)\s+'
    r'(?P<width>\d+)x(?P<height>\d+)\+-?\d+\+-?\d+'
    r'(?:\s+\+(?P<absx>-?\d+)\+(?P<absy>-?\d+))?'
)


@dataclass(frozen=True)
class VideoSourceConfig:
    mode: str
    rtsp_url: Optional[str]
    framerate: int


def parse_window_tree(output: str) -> list[dict]:
    """Parse X11 tree rows without assigning a view name."""
    windows = []
    for line in output.splitlines():
        match = _WINDOW_LINE_RE.search(line)
        if not match:
            continue
        class1 = match.group("class1")
        class2 = match.group("class2")
        windows.append({
            "window_id": int(match.group("window_id"), 16),
            "title": match.group("title"),
            "class1": class1,
            "class2": class2,
            "width": int(match.group("width")),
            "height": int(match.group("height")),
            "x": int(match.group("absx")) if match.group("absx") else 0,
            "y": int(match.group("absy")) if match.group("absy") else 0,
            "is_render_surface": (
                "osg" in class1.casefold() or "osg" in class2.casefold()
            ),
        })
    return windows


def _title_match(title: str, expected: str) -> Optional[int]:
    """Return 0 for an exact title and 1 for a delimited occurrence."""
    actual = title.strip().casefold()
    wanted = expected.casefold()
    if actual == wanted:
        return 0
    pattern = rf"(?<![0-9A-Za-z]){re.escape(wanted)}(?![0-9A-Za-z])"
    return 1 if re.search(pattern, actual, re.IGNORECASE) else None


def _select_window(
    windows: Sequence[dict],
    titles: Sequence[str],
    excluded_ids: set[int] | None = None,
) -> Optional[dict]:
    excluded = excluded_ids or set()
    ranked = []
    for window in windows:
        if window["window_id"] in excluded:
            continue
        for title_rank, title in enumerate(titles):
            match_rank = _title_match(window["title"], title)
            if match_rank is None:
                continue
            ranked.append((
                title_rank,
                match_rank,
                not window["is_render_surface"],
                -window["width"] * window["height"],
                window["window_id"],
                window,
                title,
            ))
            break
    if not ranked:
        return None
    selected_entry = min(ranked, key=lambda item: item[:-2])
    selected = dict(selected_entry[-2])
    selected["matched_title"] = selected_entry[-1]
    return selected


def select_vtd_windows(windows: Sequence[dict]) -> dict[str, Optional[dict]]:
    """Assign distinct X11 surfaces to birdRS and mainRS.

    birdRS has no generic fallback. mainRS explicitly falls back from mainRS
    to cam1 and finally igVisualBroadcast. Selecting birdRS first and excluding
    its XID guarantees that the two labels never describe the same surface.
    """
    birdrs = _select_window(windows, BIRDRS_TITLES)
    excluded = {birdrs["window_id"]} if birdrs else set()
    mainrs = _select_window(windows, MAINRS_TITLES, excluded)
    return {"mainrs": mainrs, "birdrs": birdrs}


def _read_window_tree() -> Optional[str]:
    try:
        result = subprocess.run(
            ["xwininfo", "-root", "-tree"],
            capture_output=True,
            text=True,
            timeout=3,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    return result.stdout


def find_vtd_windows() -> dict[str, Optional[dict]]:
    """Discover the current pair of VTD render surfaces from the X11 tree."""
    output = _read_window_tree()
    if output is None:
        return {"mainrs": None, "birdrs": None}
    return select_vtd_windows(parse_window_tree(output))


def find_mainrs_window(titles: Sequence[str] = MAINRS_TITLES) -> Optional[dict]:
    """Compatibility helper returning one main view with explicit fallbacks."""
    output = _read_window_tree()
    if output is None:
        return None
    return _select_window(parse_window_tree(output), titles)


def configured_rtsp_url(network_config_path: Path) -> Optional[str]:
    """Build the VTD broadcast RTSP URL from network.yaml."""
    try:
        config = yaml.safe_load(network_config_path.read_text(encoding='utf-8')) or {}
        vtd = config.get('vtd', {})
        host = str(vtd.get('host_ip', '')).strip()
        port = int(vtd.get('rtsp_port', 8554))
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        return None
    if not host:
        return None
    return f'rtsp://{host}:{port}/broadcast'


def local_window_command(window: dict, framerate: int) -> list:
    """Build the exact ffmpeg command for an X11 VTD render surface."""
    display = os.environ.get('DISPLAY', ':0')
    return [
        'ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-f', 'x11grab',
        '-window_id', str(window['window_id']),
        '-video_size', f"{window['width']}x{window['height']}",
        '-framerate', str(framerate),
        '-i', display,
        '-an', '-f', 'mjpeg', '-q:v', '7', 'pipe:1',
    ]


def rtsp_command(url: str, framerate: int) -> list:
    """Build the exact ffmpeg command for the VTD RTSP broadcast."""
    return [
        'ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-rtsp_transport', 'tcp',
        '-rw_timeout', '5000000',
        '-i', url,
        '-an', '-vf', f'fps={framerate}',
        '-f', 'mjpeg', '-q:v', '7', 'pipe:1',
    ]


def redact_rtsp_url(url: str) -> str:
    """Remove RTSP credentials/tokens before exposure or persistence."""
    parts = urlsplit(url)
    netloc = parts.netloc
    if "@" in netloc:
        netloc = "<redacted>@" + netloc.rsplit("@", 1)[1]
    return urlunsplit((
        parts.scheme,
        netloc,
        parts.path,
        "<redacted>" if parts.query else "",
        "<redacted>" if parts.fragment else "",
    ))


def video_capture_command(
    config: VideoSourceConfig,
    window_finder: Callable[[], Optional[dict]] = find_mainrs_window,
    *,
    view: str = "mainrs",
) -> Tuple[Optional[list], Optional[dict], Optional[str]]:
    """Return an ffmpeg command, truthful source metadata, and an error."""
    if view not in {"mainrs", "birdrs"}:
        raise ValueError(f"unsupported video view: {view}")
    if shutil.which('ffmpeg') is None:
        return None, {"source": None, "view": view}, 'ffmpeg is not installed'
    if config.mode in {'auto', 'window'}:
        window = window_finder()
        if window is not None:
            source = {
                "source": "window",
                "view": view,
                "title": window["title"],
                "matched_title": window.get("matched_title"),
                "window_id": window["window_id"],
                "geometry": {
                    "x": window["x"],
                    "y": window["y"],
                    "width": window["width"],
                    "height": window["height"],
                },
            }
            return local_window_command(window, config.framerate), source, None
        if config.mode == 'window':
            return None, {"source": "window", "view": view}, f'{view} window not found'
    if config.mode in {'auto', 'rtsp'} and config.rtsp_url:
        source = {
            "source": "rtsp",
            "view": view,
            "url": redact_rtsp_url(config.rtsp_url),
        }
        return rtsp_command(config.rtsp_url, config.framerate), source, None
    return None, {"source": None, "view": view}, f'{view} video source is not configured'
